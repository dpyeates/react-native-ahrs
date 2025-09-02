package com.ahrs

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Handler
import android.os.Looper
import com.facebook.react.bridge.ReactApplicationContext
import com.facebook.react.module.annotations.ReactModule
import com.facebook.react.turbomodule.core.interfaces.TurboModule
import com.facebook.react.bridge.Arguments
import com.facebook.react.bridge.Promise
import com.facebook.react.bridge.ReactMethod

val Float.hz: Long get() = (1_000_000_000.0 / this).toLong()

@ReactModule(name = AhrsModule.NAME)
class AhrsModule(reactContext: ReactApplicationContext) :
  NativeAhrsSpec(reactContext), TurboModule, SensorEventListener {

  companion object {
    const val NAME = "NativeAhrs"

    init {
      System.loadLibrary("ahrs")
    }
  }

  // the following external functions are written in CPP and are loaded
  // from ahrs.so which is loaded above via the loadLibrary call
  private external fun initAhrs(platform: Int, rotation: Int, gain: Float)
  private external fun updateAhrs(
    deltaTime: Float,
    accel: FloatArray,
    gyro: FloatArray,
    mag: FloatArray
  )
  private external fun zeroAhrs()
  private external fun getAhrsRoll(): Float
  private external fun getAhrsPitch(): Float
  private external fun getAhrsHeading(): Float
  private external fun setAhrsInterfaceRotation(rotation: Int)

  private val sensorManager: SensorManager =
    reactContext.getSystemService(Context.SENSOR_SERVICE) as SensorManager
  private var handler: Handler = Handler(Looper.getMainLooper())
  private var gyroData: FloatArray = FloatArray(3)
  private var accelData: FloatArray = FloatArray(3)
  private var magData: FloatArray = FloatArray(3)
  private var gyroCopy: FloatArray = FloatArray(3)
  private var accelCopy: FloatArray = FloatArray(3)
  private var magCopy: FloatArray = FloatArray(3)
  private var running: Boolean = false
  private var rotation: Int = 0
  private var gain: Float = 3.0f
  private var rate: Float = 5.0f
  private var nextEmitTime: Long = 0L
  private val sensorLock = Any()
  private val gyroInDegPerSec = FloatArray(3)
  private val accelInG = FloatArray(3)
  private var lastFrameTimeNs = 0L
  private val targetFps = 60.0
  private val targetIntervalNs = (1_000_000_000.0 / targetFps).toLong()
  private var nextTargetTime = 0L

  override fun getName(): String = NAME

  override fun initialize() {
    resetAhrs()
  }

  override fun invalidate() {
    stopAhrs()
  }

  override fun startAhrs() {
    handler.postDelayed({
      // Register sensors to listen with approx 60Hz (16_667 microseconds)
      val sensorDelayUs = 16_667 // 1_000_000 / 60
      sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)?.let {
        sensorManager.registerListener(this, it, sensorDelayUs)
      }
      sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)?.let {
        sensorManager.registerListener(this, it, sensorDelayUs)
      }
      sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)?.let {
        sensorManager.registerListener(this, it, sensorDelayUs)
      }
      lastFrameTimeNs = 0L
      handler.post(fusionUpdateRunnable)
      running = true
    }, 500) // 100ms delay
  }

  override fun stopAhrs() {
    sensorManager.unregisterListener(this)
    handler.removeCallbacks(fusionUpdateRunnable)
    running = false
  }

  override fun resetAhrs() {
    initAhrs(1, rotation, gain)
  }

  override fun levelAhrs() {
    zeroAhrs()
  }

  override fun setAhrsGain(newGain: Double) {
    if (newGain > 0) {
      gain = newGain.toFloat()
      resetAhrs()
    }
  }

  override fun setAhrsRate(newRate: Double) {
    if (newRate in 1.0..60.0) {
      rate = newRate.toFloat()
    }
  }

  override fun setAhrsRotation(newRotation: String) {
    rotation = if (newRotation.lowercase() == "left") {
      -1
    } else if (newRotation.lowercase() == "right") {
      1
    } else {
      0
    }
    setAhrsInterfaceRotation(rotation)
  }

  override fun isSupported(promise: Promise) {
    val hasGyro = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE) != null
    val hasAccel = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) != null
    val hasMag = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD) != null
    promise.resolve(hasGyro && hasAccel && hasMag)
  }

  override fun onSensorChanged(event: SensorEvent) {
    when (event.sensor.type) {
      Sensor.TYPE_GYROSCOPE -> {
        // Android gives rad/s - convert to deg/s
        gyroInDegPerSec[0] = event.values[0] * 180.0f / Math.PI.toFloat()  // rad/s → deg/s
        gyroInDegPerSec[1] = event.values[1] * 180.0f / Math.PI.toFloat()
        gyroInDegPerSec[2] = event.values[2] * 180.0f / Math.PI.toFloat()
        synchronized(sensorLock) {
          System.arraycopy(gyroInDegPerSec, 0, gyroData, 0, 3)
        }
      }

      Sensor.TYPE_ACCELEROMETER -> {
        // Android gives m/s² - convert to g
        accelInG[0] = event.values[0] / SensorManager.GRAVITY_EARTH  // m/s² → g
        accelInG[1] = event.values[1] / SensorManager.GRAVITY_EARTH
        accelInG[2] = event.values[2] / SensorManager.GRAVITY_EARTH
        synchronized(sensorLock) {
          System.arraycopy(accelInG, 0, accelData, 0, 3)
        }
      }

      Sensor.TYPE_MAGNETIC_FIELD -> {
        synchronized(sensorLock) {
          System.arraycopy(event.values, 0, magData, 0, 3)
        }
      }
    }
  }

  override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}

  private val fusionUpdateRunnable = object : Runnable {
    override fun run() {
      val currentTimeNs = System.nanoTime()

      if (nextTargetTime == 0L) {
        nextTargetTime = currentTimeNs + targetIntervalNs
      }

      // Calculate accurate delta time
      val deltaTime = if (lastFrameTimeNs != 0L) {
        (currentTimeNs - lastFrameTimeNs) / 1_000_000_000.0f
      } else {
        1.0f / 60.0f
      }

      synchronized(sensorLock) {
        System.arraycopy(accelData, 0, accelCopy, 0, 3)
        System.arraycopy(gyroData, 0, gyroCopy, 0, 3)
        System.arraycopy(magData, 0, magCopy, 0, 3)
      }

      updateAhrs(deltaTime, accelCopy, gyroCopy, magCopy)

      if (currentTimeNs > nextEmitTime) {
        val map = Arguments.createMap()
        map.putDouble("roll", getAhrsRoll().toDouble())
        map.putDouble("pitch", getAhrsPitch().toDouble())
        map.putDouble("heading", getAhrsHeading().toDouble())
        emitOnAhrsUpdate(map)
        nextEmitTime = currentTimeNs + rate.hz
      }

      // Precise timing for next frame
      nextTargetTime += targetIntervalNs
      val delay = (nextTargetTime - System.nanoTime()) / 1_000_000L

      if (delay > 0) {
        handler.postDelayed(this, delay)
      } else {
        handler.post(this)
        nextTargetTime = System.nanoTime() + targetIntervalNs
      }

      lastFrameTimeNs = currentTimeNs
    }
  }
}
