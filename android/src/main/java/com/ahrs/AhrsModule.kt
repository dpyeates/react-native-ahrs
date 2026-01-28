package com.ahrs

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.util.Log
import com.facebook.react.BuildConfig
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import com.facebook.react.bridge.Arguments
import com.facebook.react.bridge.Promise
import com.facebook.react.bridge.ReactApplicationContext
import com.facebook.react.bridge.ReactMethod
import com.facebook.react.bridge.UiThreadUtil
import com.facebook.react.module.annotations.ReactModule
import com.facebook.react.turbomodule.core.interfaces.TurboModule
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.Response
import okhttp3.WebSocket
import okhttp3.WebSocketListener
import org.json.JSONObject
import java.util.concurrent.TimeUnit

val Float.hz: Long get() = (1_000_000_000.0 / this).toLong()

@ReactModule(name = AhrsModule.NAME)
class AhrsModule(reactContext: ReactApplicationContext) :
  NativeAhrsSpec(reactContext), TurboModule, SensorEventListener, LocationListener {

  companion object {
    const val NAME = "NativeAhrs"
    private const val TAG = "AhrsModule"
    
    private inline fun logIfDebug(level: Int, tag: String, message: String) {
      if (BuildConfig.DEBUG) {
        Log.println(level, tag, message)
      }
    }
    
    private fun logD(tag: String, message: String) = logIfDebug(Log.DEBUG, tag, message)
    private fun logI(tag: String, message: String) = logIfDebug(Log.INFO, tag, message)
    private fun logW(tag: String, message: String) = logIfDebug(Log.WARN, tag, message)
    private fun logE(tag: String, message: String) = logIfDebug(Log.ERROR, tag, message)
    private const val FRAME_RATE = 60 // Hz
    private const val INTERVAL_MS = 1000 / FRAME_RATE

    init {
      System.loadLibrary("ahrs")
    }
  }

  // JNI functions
  private external fun initAhrs(): Long
  private external fun destroyAhrs()
  private external fun updateAhrs(
    ekfPtr: Long,
    timestampUs: Long,
    accel: FloatArray,
    gyro: FloatArray,
    mag: FloatArray,
    gps: FloatArray?,
    baro: FloatArray?
  )
  private external fun resetAhrs(ekfPtr: Long)
  private external fun zeroAhrs(ekfPtr: Long)
  private external fun setAhrsRotation(rotation: Int)
  private external fun setMagneticDeclination(ekfPtr: Long, declination: Float)
  private external fun setQNH(ekfPtr: Long, qnh: Float)
  private external fun getGpsPosition(ekfPtr: Long): DoubleArray
  private external fun isPositionReliable(ekfPtr: Long): Boolean
  private external fun getAhrsOutput(ekfPtr: Long): FloatArray

  private val sensorManager: SensorManager =
    reactContext.getSystemService(Context.SENSOR_SERVICE) as SensorManager
  private val locationManager: LocationManager =
    reactContext.getSystemService(Context.LOCATION_SERVICE) as LocationManager
  private val handler: Handler = Handler(Looper.getMainLooper())

  // EKF instance pointer
  private var ekfPtr: Long = 0

  // Sensor data
  private val sensorLock = Any()
  private var gyroData: FloatArray = FloatArray(3)
  private var accelData: FloatArray = FloatArray(3)
  private var magData: FloatArray = FloatArray(3)
  private var lastFrameTimeNs: Long = 0
  private var nextTargetTime: Long = 0
  private val targetIntervalNs = (1_000_000_000.0 / FRAME_RATE).toLong()

  // GPS data
  private val locationLock = Any()
  private var locationData: FloatArray? = null
  private var lastUsedLocationTimestamp: Long = 0
  private var previousLocation: Location? = null

  // Barometer data
  private val baroLock = Any()
  private var baroData: FloatArray? = null
  private var lastUsedBaroTimestamp: Long = 0
  private var baroCalibrated: Boolean = false
  private var baroPressureOffset: Float = 0.0f
  private var baroCalibrationSamples: Int = 0
  private var baroCalibrationSum: Float = 0.0f

  // State
  private var running: Boolean = false
  private var rotation: Int = 0 // 0=vertical, 1=left, 2=right
  private var emitRateHz: Float = 5.0f
  private var nextEmitTime: Long = 0
  private var ekfAttitudeInitialized: Boolean = false

  // X-Plane WebSocket connection
  private var xplaneClient: OkHttpClient? = null
  private var xplaneWebSocket: WebSocket? = null
  private var xplaneConnected: Boolean = false
  private var xplaneHost: String? = null
  private var xplaneWasPaused: Boolean = false // Previous pause state for transition detection

  private fun headingOffsetDeg(): Float {
    return when (rotation) {
      1 -> 0.0f      // Left: portrait offset (-90°) + 90° CW
      2 -> -180.0f   // Right: portrait offset (-90°) - 90° CCW
      else -> -90.0f // Vertical: portrait offset
    }
  }

  private fun normalizeHeadingDeg(heading: Float): Float {
    var result = heading % 360.0f
    if (result < 0f) result += 360.0f
    return result
  }

  override fun getName(): String = NAME

  /**
   * Initializes the EKF instance
   * Called automatically by React Native when module is loaded
   */
  override fun initialize() {
    ekfPtr = initAhrs()
    if (ekfPtr == 0L) {
      logE(TAG, "Failed to initialize EKF")
    } else {
      logI(TAG, "AHRS initialized successfully")
    }
  }

  /**
   * Cleans up resources when module is invalidated
   * Called automatically by React Native when module is unloaded
   */
  override fun invalidate() {
    disconnectFromXPlane()
    stopAhrs()
    if (ekfPtr != 0L) {
      destroyAhrs()
      ekfPtr = 0L
    }
  }

  /**
   * Starts AHRS sensor processing
   * 
   * Begins collecting and fusing sensor data at 60Hz internally.
   * Data is emitted to React Native at the configured rate (default 5Hz).
   * 
   * Initializes:
   * - Gyroscope, accelerometer, magnetometer at 60Hz
   * - Barometer (if available) at ~1-2Hz
   * - GPS location updates (if permissions granted)
   * 
   * Requests location permissions if not yet determined.
   * Continues without GPS if permissions denied (IMU-only mode).
   */
  override fun startAhrs() {
    if (running) {
      logW(TAG, "AHRS already running")
      return
    }

    if (ekfPtr == 0L) {
      logE(TAG, "Cannot start: EKF not initialized")
      return
    }

    // Check and request location permissions following Android best practices
    UiThreadUtil.runOnUiThread {
      checkAndRequestLocationPermissions()
    }

    // Register sensors
    val sensorDelayUs = INTERVAL_MS * 1000
    sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)?.let {
      sensorManager.registerListener(this, it, sensorDelayUs)
    }
    sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)?.let {
      sensorManager.registerListener(this, it, sensorDelayUs)
    }
    sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)?.let {
      sensorManager.registerListener(this, it, sensorDelayUs)
    }

    // Check for barometer
    val pressureSensor = sensorManager.getDefaultSensor(Sensor.TYPE_PRESSURE)
    if (pressureSensor != null) {
      sensorManager.registerListener(this, pressureSensor, sensorDelayUs)
    } else {
      logW(TAG, "Pressure sensor not available")
    }

    lastFrameTimeNs = 0
    nextTargetTime = 0
    ekfAttitudeInitialized = false
    handler.post(fusionUpdateRunnable)
    running = true

    logI(TAG, "AHRS started")
  }

  /**
   * Stops AHRS sensor processing
   * 
   * Halts all sensor updates to save battery:
   * - Unregisters all sensor listeners
   * - Stops GPS location updates
   * - Stops fusion update loop
   * 
   * Resets state flags. Can be restarted later with startAhrs().
   */
  override fun stopAhrs() {
    if (!running) {
      logW(TAG, "AHRS not running")
      return
    }

    sensorManager.unregisterListener(this)
    try {
      locationManager.removeUpdates(this)
    } catch (e: SecurityException) {
      // Ignore
    }

    handler.removeCallbacks(fusionUpdateRunnable)
    running = false
    ekfAttitudeInitialized = false
    nextEmitTime = 0

    logI(TAG, "AHRS stopped")
  }

  /**
   * Resets the EKF filter to initial state
   * 
   * Clears all state estimates and covariances.
   * The filter will re-initialize from current sensor readings.
   * Requires 2-3 seconds for reconvergence.
   */
  override fun resetAhrs() {
    if (ekfPtr == 0L) {
      logE(TAG, "Cannot reset: EKF not initialized")
      return
    }
    resetAhrs(ekfPtr)
    ekfAttitudeInitialized = false
    logI(TAG, "AHRS reset")
  }

  /**
   * Levels the attitude reference
   * 
   * Captures the current attitude as the "zero" reference.
   * Sets roll=0° and pitch=0° based on current orientation.
   * Does not reset heading or other states.
   */
  override fun levelAhrs() {
    if (ekfPtr == 0L) {
      logE(TAG, "Cannot level: EKF not initialized")
      return
    }
    zeroAhrs(ekfPtr)
    logI(TAG, "AHRS leveled")
  }

  /**
   * Sets the output emission rate
   * 
   * Controls how often data is sent to React Native JavaScript.
   * Independent of internal sensor rate (60Hz).
   * Lower rates save battery and reduce JavaScript load.
   * 
   * @param newRate Rate in Hz, range [1, 60]. Values outside range are ignored.
   *                Default: 5 Hz
   */
  override fun setAhrsRate(newRate: Double) {
    if (newRate in 1.0..60.0) {
      emitRateHz = newRate.toFloat()
    }
  }

  /**
   * Sets device rotation/mounting orientation
   * 
   * Configures coordinate frame transformations from Android device sensors
   * to aviation body frame. Must match physical device mounting.
   * 
   * Supported values:
   * - "left", "landscape_left", "landscapeleft" -> Landscape Left (90° CCW)
   * - "right", "landscape_right", "landscaperight" -> Landscape Right (90° CW)
   * - "none", "vertical", "portrait" -> Portrait/Vertical (default)
   * 
   * When rotation changes, automatically resets the AHRS filter.
   * 
   * @param newRotation Rotation string (case-insensitive)
   */
  override fun setAhrsRotation(newRotation: String) {
    val oldRotation = rotation
    rotation = when (newRotation.lowercase()) {
      "left", "landscape_left", "landscapeleft" -> 1
      "right", "landscape_right", "landscaperight" -> 2
      else -> 0
    }
    setAhrsRotation(rotation)
    
    if (oldRotation != rotation) {
      ekfAttitudeInitialized = false
      logI(TAG, "Rotation changed to $rotation - forcing re-initialization")
    }
  }

  /**
   * Checks if AHRS is supported on this device
   * 
   * Verifies that all required sensors are available:
   * - Gyroscope
   * - Accelerometer
   * - Magnetometer
   * 
   * @param promise Promise resolver, called with true if supported, false otherwise
   */
  override fun isSupported(promise: Promise) {
    val hasGyro = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE) != null
    val hasAccel = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) != null
    val hasMag = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD) != null
    promise.resolve(hasGyro && hasAccel && hasMag)
  }

  /**
   * Sets magnetic declination for true heading calculation
   * 
   * Magnetic declination is the angle between magnetic north and true north.
   * Critical for navigation - converts magnetic heading to true heading.
   * 
   * @param declination Declination in degrees
   *                    Positive = east (magnetic north is east of true north)
   *                    Negative = west (magnetic north is west of true north)
   */
  override fun setMagneticDeclination(declination: Double) {
    if (ekfPtr == 0L) return
    setMagneticDeclination(ekfPtr, declination.toFloat())
    logI(TAG, "Magnetic declination set to ${declination}°")
  }

  /**
   * Sets QNH pressure for altitude correction
   * 
   * QNH is local sea-level pressure used by aviation.
   * Required for accurate altitude calculations from barometric pressure.
   * 
   * @param qnh Pressure in hectopascals (hPa = mbar)
   *            Range: [900, 1100] hPa
   *            Standard: 1013.25 hPa
   */
  override fun setQNH(qnh: Double) {
    if (ekfPtr == 0L) return
    setQNH(ekfPtr, qnh.toFloat())
    logI(TAG, "QNH set to ${qnh} hPa")
  }

  /**
   * Checks if position estimate is reliable
   * 
   * Considers GPS outage duration and position uncertainty.
   * 
   * @param promise Promise resolver, called with true if reliable, false otherwise
   */
  override fun isPositionReliable(promise: Promise) {
    if (ekfPtr == 0L) {
      promise.reject("EKF_NOT_INITIALIZED", "EKF not initialized")
      return
    }
    promise.resolve(isPositionReliable(ekfPtr))
  }

  // ============================================================================
  // X-PLANE WEBSOCKET CONNECTION
  // ============================================================================

  /**
   * Connects to X-Plane plugin via WebSocket
   * 
   * When connected, real device sensors are bypassed and X-Plane data feeds the EKF.
   * Requires AHRS to be running (call startAhrs first).
   * 
   * @param host Hostname or IP address of X-Plane computer (e.g., "192.168.1.100")
   */
  override fun connectToXPlane(host: String) {
    if (xplaneConnected) {
      logW(TAG, "Already connected to X-Plane")
      return
    }

    if (!running) {
      logW(TAG, "Cannot connect to X-Plane: AHRS not running. Call startAhrs first.")
      return
    }

    xplaneHost = host
    val url = "ws://$host:8765"
    
    logI(TAG, "Connecting to X-Plane at $url")

    // Create OkHttp client with timeouts
    xplaneClient = OkHttpClient.Builder()
      .connectTimeout(10, TimeUnit.SECONDS)
      .readTimeout(0, TimeUnit.MILLISECONDS) // No timeout for WebSocket
      .writeTimeout(10, TimeUnit.SECONDS)
      .build()

    val request = Request.Builder()
      .url(url)
      .build()

    xplaneWebSocket = xplaneClient?.newWebSocket(request, xplaneWebSocketListener)
  }

  /**
   * Disconnects from X-Plane plugin
   * 
   * Closes WebSocket connection and returns to using real device sensors.
   */
  override fun disconnectFromXPlane() {
    if (!xplaneConnected && xplaneWebSocket == null) {
      logW(TAG, "Not connected to X-Plane")
      return
    }

    logI(TAG, "Disconnecting from X-Plane")

    xplaneWebSocket?.close(1000, "User disconnected")
    xplaneWebSocket = null
    xplaneClient?.dispatcher?.executorService?.shutdown()
    xplaneClient = null

    val wasConnected = xplaneConnected
    xplaneConnected = false
    xplaneHost = null
    xplaneWasPaused = false

    if (wasConnected) {
      emitXPlaneConnectionChanged(false)
      
      // Reset EKF for real sensor initialization
      if (ekfPtr != 0L) {
        resetAhrs(ekfPtr)
        ekfAttitudeInitialized = false
      }
      
      logI(TAG, "Disconnected from X-Plane - returning to real sensors")
    }
  }

  private val xplaneWebSocketListener = object : WebSocketListener() {
    override fun onOpen(webSocket: WebSocket, response: Response) {
      logI(TAG, "Connected to X-Plane WebSocket")
      
      handler.post {
        xplaneConnected = true
        xplaneWasPaused = false
        emitXPlaneConnectionChanged(true)
        
        // Reset EKF for clean start with X-Plane data
        if (ekfPtr != 0L) {
          resetAhrs(ekfPtr)
          ekfAttitudeInitialized = false
        }
        
        logI(TAG, "X-Plane mode active - real sensors bypassed")
      }
    }

    override fun onMessage(webSocket: WebSocket, text: String) {
      handler.post {
        handleXPlaneMessage(text)
      }
    }

    override fun onClosing(webSocket: WebSocket, code: Int, reason: String) {
      logI(TAG, "X-Plane WebSocket closing: $code - $reason")
    }

    override fun onClosed(webSocket: WebSocket, code: Int, reason: String) {
      logI(TAG, "X-Plane WebSocket closed: $code - $reason")
      
      handler.post {
        val wasConnected = xplaneConnected
        xplaneConnected = false
        
        if (wasConnected) {
          emitXPlaneConnectionChanged(false)
          
          // Reset EKF for real sensor initialization
          if (ekfPtr != 0L) {
            resetAhrs(ekfPtr)
            ekfAttitudeInitialized = false
          }
          
          logI(TAG, "X-Plane disconnected - returning to real sensors")
        }
      }
    }

    override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
      logE(TAG, "X-Plane WebSocket error: ${t.message}")
      
      handler.post {
        if (xplaneConnected) {
          xplaneConnected = false
          emitXPlaneConnectionChanged(false)
        }
      }
    }
  }

  private fun handleXPlaneMessage(json: String) {
    if (json.isEmpty() || ekfPtr == 0L) return

    try {
      val data = JSONObject(json)
      
      // Extract timestamp
      val timestamp = if (data.has("t")) data.getLong("t") else System.currentTimeMillis() * 1000

      // Check if X-Plane is paused using metadata flag
      var isPaused = false
      if (data.has("metadata")) {
        val metadata = data.getJSONObject("metadata")
        if (metadata.has("sim_paused") && metadata.getBoolean("sim_paused")) {
          isPaused = true
        }
      }
      
      // Log pause state transitions
      if (isPaused != xplaneWasPaused) {
        if (isPaused) {
          logI(TAG, "⏸️ X-Plane PAUSED - skipping filter updates")
        } else {
          logI(TAG, "▶️ X-Plane RESUMED - resuming filter updates")
        }
        xplaneWasPaused = isPaused
      }
      
      if (isPaused) {
        // X-Plane is paused, skip filter update to prevent processing stale data
        return
      }

      // Extract sensor data (same format as recording/playback)
      val accel = FloatArray(3)
      val gyro = FloatArray(3)
      val mag = FloatArray(3)
      var gps: FloatArray? = null
      var baro: FloatArray? = null

      // Accelerometer (m/s²)
      if (data.has("acc")) {
        val acc = data.getJSONObject("acc")
        accel[0] = acc.getDouble("x").toFloat()
        accel[1] = acc.getDouble("y").toFloat()
        accel[2] = acc.getDouble("z").toFloat()
      }

      // Gyroscope (rad/s)
      if (data.has("gyro")) {
        val g = data.getJSONObject("gyro")
        gyro[0] = g.getDouble("x").toFloat()
        gyro[1] = g.getDouble("y").toFloat()
        gyro[2] = g.getDouble("z").toFloat()
      }

      // Magnetometer (Gauss)
      if (data.has("mag")) {
        val m = data.getJSONObject("mag")
        mag[0] = m.getDouble("x").toFloat()
        mag[1] = m.getDouble("y").toFloat()
        mag[2] = m.getDouble("z").toFloat()
      }

      // GPS
      if (data.has("gps")) {
        val g = data.getJSONObject("gps")
        val lat = g.getDouble("lat").toFloat()
        val lon = g.getDouble("lon").toFloat()
        val alt = g.getDouble("alt").toFloat()
        val trk = if (g.has("trk")) g.getDouble("trk").toFloat() else 0f
        val spd = if (g.has("spd")) g.getDouble("spd").toFloat() else 0f
        
        // Convert track/speed to NED velocity
        val trkRad = Math.toRadians(trk.toDouble())
        val velN = (spd * kotlin.math.cos(trkRad)).toFloat()
        val velE = (spd * kotlin.math.sin(trkRad)).toFloat()
        val velD = if (data.has("vs")) -data.getDouble("vs").toFloat() else 0f
        
        gps = floatArrayOf(
          lat, lon, alt,
          velN, velE, velD,
          1.0f,  // hdop (assume good)
          12.0f, // numSats (assume good)
          1.0f   // valid
        )
      }

      // Barometer (hPa)
      if (data.has("press")) {
        val press = data.getDouble("press").toFloat()
        baro = floatArrayOf(
          press,
          15.0f, // temperature (default)
          1.0f   // valid
        )
      }

      // Update EKF with X-Plane sensor data
      updateAhrs(ekfPtr, timestamp, accel, gyro, mag, gps, baro)

      // Emit output at configured rate
      val currentTimeNs = System.nanoTime()
      if (currentTimeNs > nextEmitTime) {
        val output = getAhrsOutput(ekfPtr)
        if (output != null && output.size >= 24) {
          val map = Arguments.createMap()
          val headingOffset = headingOffsetDeg()
          
          map.putDouble("roll", output[0].toDouble())
          map.putDouble("pitch", output[1].toDouble())
          map.putDouble("heading", normalizeHeadingDeg(output[2] + headingOffset).toDouble())
          map.putDouble("flightPathAngle", output[3].toDouble())
          map.putDouble("horizontalFlightPathAngle", output[4].toDouble())
          map.putDouble("trackAngle", output[5].toDouble())
          map.putDouble("horizontalSpeed", output[6].toDouble())
          map.putDouble("totalSpeed", output[7].toDouble())
          map.putDouble("altitude", output[8].toDouble())
          map.putDouble("altitudeQNE", output[9].toDouble())
          map.putDouble("altitudeQNH", output[10].toDouble())
          map.putDouble("verticalSpeed", output[11].toDouble())
          // Get barometric pressure from baroData if available
          val baroPressure = synchronized(baroLock) {
            baroData?.getOrNull(0)?.toDouble() ?: 0.0
          }
          map.putDouble("barometricPressure", baroPressure)
          map.putDouble("velocityNorth", output[12].toDouble())
          map.putDouble("velocityEast", output[13].toDouble())
          map.putDouble("velocityDown", output[14].toDouble())
          val gpsPos = getGpsPosition(ekfPtr)
          if (gpsPos[0] != 0.0 || gpsPos[1] != 0.0) {
            map.putDouble("latitude", gpsPos[0])
            map.putDouble("longitude", gpsPos[1])
          }
          map.putInt("flightPhase", output[15].toInt())
          map.putDouble("flightPhaseConfidence", output[16].toDouble())
          map.putBoolean("attitudeValid", output[17] > 0.5f)
          map.putBoolean("altitudeValid", output[18] > 0.5f)
          map.putBoolean("positionValid", output[19] > 0.5f)
          map.putBoolean("flightPhaseValid", output[20] > 0.5f)
          
          emitOnAhrsUpdate(map)
        }
        nextEmitTime = currentTimeNs + emitRateHz.hz
      }
    } catch (e: Exception) {
      logE(TAG, "Error parsing X-Plane message: ${e.message}")
    }
  }

  private fun emitXPlaneConnectionChanged(connected: Boolean) {
    val map = Arguments.createMap()
    map.putBoolean("connected", connected)
    map.putString("host", xplaneHost ?: "")
    emitOnXPlaneConnectionChanged(map)
  }

  override fun onSensorChanged(event: SensorEvent) {
    when (event.sensor.type) {
      Sensor.TYPE_GYROSCOPE -> {
        synchronized(sensorLock) {
          System.arraycopy(event.values, 0, gyroData, 0, 3)
        }
      }
      Sensor.TYPE_ACCELEROMETER -> {
        synchronized(sensorLock) {
          System.arraycopy(event.values, 0, accelData, 0, 3)
        }
      }
      Sensor.TYPE_MAGNETIC_FIELD -> {
        synchronized(sensorLock) {
          System.arraycopy(event.values, 0, magData, 0, 3)
        }
      }
      Sensor.TYPE_PRESSURE -> {
        val pressurePa = event.values[0]
        val pressureHpa = pressurePa / 100.0f

        // Calibrate barometer against GPS if available
        synchronized(locationLock) {
          val gps = locationData
          if (!baroCalibrated && gps != null && gps.size >= 9 && gps[8] > 0.5f) {
            val gpsAltMsl = gps[2]
            val pressureRatio = Math.pow(
              1.0 - gpsAltMsl / 44330.0,
              1.0 / 0.1903
            ).toFloat()

            if (pressureRatio > 0.1f && pressureRatio < 2.0f) {
              val seaLevelPressureHpa = pressureHpa / pressureRatio
              val offsetSample = seaLevelPressureHpa - pressureHpa

              baroCalibrationSum += offsetSample
              baroCalibrationSamples++

              if (baroCalibrationSamples >= 10) {
                baroPressureOffset = baroCalibrationSum / baroCalibrationSamples
                baroCalibrated = true
                logI(TAG, "Barometer calibrated: offset = ${baroPressureOffset} hPa")
                baroCalibrationSamples = 0
                baroCalibrationSum = 0.0f
              }
            }
          }
        }

        // Apply calibration if available
        val absolutePressureHpa = if (baroCalibrated) {
          pressureHpa + baroPressureOffset
        } else {
          pressureHpa
        }

        synchronized(baroLock) {
          baroData = floatArrayOf(
            absolutePressureHpa, // EKF expects hPa
            15.0f, // temperature (default)
            if (baroCalibrated) 1.0f else 0.0f // valid flag
          )
        }
      }
    }
  }

  override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}

  /**
   * Check location permissions and request if needed
   * Follows Android best practices for runtime permissions
   */
  private fun checkAndRequestLocationPermissions() {
    val fineLocationPermission = Manifest.permission.ACCESS_FINE_LOCATION
    val coarseLocationPermission = Manifest.permission.ACCESS_COARSE_LOCATION
    
    val fineLocationGranted = ContextCompat.checkSelfPermission(
      reactApplicationContext,
      fineLocationPermission
    ) == PackageManager.PERMISSION_GRANTED
    
    val coarseLocationGranted = ContextCompat.checkSelfPermission(
      reactApplicationContext,
      coarseLocationPermission
    ) == PackageManager.PERMISSION_GRANTED

    if (fineLocationGranted || coarseLocationGranted) {
      // Permissions granted - start location updates
      startLocationUpdates()
    } else {
      // Permissions not granted - request them
      // Note: In a React Native module, we typically rely on the app to request permissions
      // via a permission library (like react-native-permissions). However, we can still
      // try to request via ActivityCompat if we have access to the current activity.
      logW(TAG, "Location permissions not granted. Please request permissions in your app.")
      logI(TAG, "AHRS will continue without GPS - IMU-only mode")
      // Continue without GPS - AHRS can work with IMU sensors only
    }
  }

  /**
   * Start location updates if permissions are granted
   */
  private fun startLocationUpdates() {
    val fineLocationPermission = Manifest.permission.ACCESS_FINE_LOCATION
    val coarseLocationPermission = Manifest.permission.ACCESS_COARSE_LOCATION
    
    val hasFineLocation = ContextCompat.checkSelfPermission(
      reactApplicationContext,
      fineLocationPermission
    ) == PackageManager.PERMISSION_GRANTED
    
    val hasCoarseLocation = ContextCompat.checkSelfPermission(
      reactApplicationContext,
      coarseLocationPermission
    ) == PackageManager.PERMISSION_GRANTED

    if (!hasFineLocation && !hasCoarseLocation) {
      logW(TAG, "Location permissions not available")
      return
    }

    try {
      // Use GPS provider if fine location is available, otherwise use network
      if (hasFineLocation && locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER)) {
        if (ActivityCompat.checkSelfPermission(
            reactApplicationContext,
            fineLocationPermission
          ) == PackageManager.PERMISSION_GRANTED
        ) {
          locationManager.requestLocationUpdates(
            LocationManager.GPS_PROVIDER,
            1000L, // 1 second minimum interval
            1.0f,  // 1 meter minimum distance
            this
          )
          logI(TAG, "GPS location updates started")
        }
      }
      
      // Also use network provider as fallback
      if (locationManager.isProviderEnabled(LocationManager.NETWORK_PROVIDER)) {
        val permissionToCheck = if (hasFineLocation) fineLocationPermission else coarseLocationPermission
        if (ActivityCompat.checkSelfPermission(
            reactApplicationContext,
            permissionToCheck
          ) == PackageManager.PERMISSION_GRANTED
        ) {
          locationManager.requestLocationUpdates(
            LocationManager.NETWORK_PROVIDER,
            1000L,
            1.0f,
            this
          )
          logI(TAG, "Network location updates started")
        }
      }
    } catch (e: SecurityException) {
      logW(TAG, "SecurityException starting location updates: ${e.message}")
    } catch (e: Exception) {
      logE(TAG, "Error starting location updates: ${e.message}")
    }
  }

  override fun onLocationChanged(location: Location) {
    if (!isValidLocation(location)) return

    val timestampUs = (location.time * 1000).toLong() // Convert ms to us

    // Compute velocity from position if not available
    var velN = 0.0f
    var velE = 0.0f
    var velD = 0.0f
    var speed = 0.0f

    if (location.hasSpeed() && location.speedAccuracy >= 0) {
      speed = location.speed
      if (location.hasBearing() && location.bearingAccuracy >= 0) {
        val courseRad = Math.toRadians(location.bearing.toDouble()).toFloat()
        velN = speed * kotlin.math.cos(courseRad)
        velE = speed * kotlin.math.sin(courseRad)
      } else if (previousLocation != null) {
        computeVelocityFromPositions(location, previousLocation!!)?.let {
          velN = it[0]
          velE = it[1]
          speed = kotlin.math.sqrt(velN * velN + velE * velE)
        }
      }
    } else if (previousLocation != null) {
      computeVelocityFromPositions(location, previousLocation!!)?.let {
        velN = it[0]
        velE = it[1]
        speed = kotlin.math.sqrt(velN * velN + velE * velE)
      }
    }

    if (previousLocation != null) {
      val dt = (location.time - previousLocation!!.time) / 1000.0
      if (dt > 0 && dt < 10.0) {
        velD = ((previousLocation!!.altitude - location.altitude) / dt).toFloat()
        velD = velD.coerceIn(-20.0f, 20.0f)
      }
    }

    val horizontalAccuracy = location.accuracy
    val numSats = when {
      horizontalAccuracy < 0 -> 0
      horizontalAccuracy < 5.0f -> 12
      horizontalAccuracy < 10.0f -> 10
      horizontalAccuracy < 20.0f -> 8
      horizontalAccuracy < 50.0f -> 6
      else -> 4
    }

    val hdop = (horizontalAccuracy / 3.0f).coerceIn(1.0f, 20.0f)
    val vdop = (location.verticalAccuracy / 3.0f).coerceIn(1.0f, 20.0f)

    val valid = isValidLocation(location) && speed <= 200.0f

    synchronized(locationLock) {
      locationData = floatArrayOf(
        location.latitude.toFloat(),
        location.longitude.toFloat(),
        location.altitude.toFloat(),
        velN,
        velE,
        velD,
        hdop,
        numSats.toFloat(),
        if (valid) 1.0f else 0.0f
      )
    }

    previousLocation = location
  }

  override fun onProviderEnabled(provider: String) {}
  override fun onProviderDisabled(provider: String) {}
  override fun onStatusChanged(provider: String?, status: Int, extras: Bundle?) {}

  private fun isValidLocation(location: Location): Boolean {
    if (!location.hasAccuracy() || location.accuracy < 0) return false
    if (location.accuracy > 100.0f) return false
    val age = (System.currentTimeMillis() - location.time) / 1000.0
    if (age > 10.0 || age < -1.0) return false
    if (kotlin.math.abs(location.latitude) < 0.01 &&
        kotlin.math.abs(location.longitude) < 0.01) return false
    return true
  }

  private fun computeVelocityFromPositions(current: Location, previous: Location): FloatArray? {
    val dt = (current.time - previous.time) / 1000.0
    if (dt <= 0 || dt > 10.0) return null

    val earthRadius = 6378137.0
    val dLat = Math.toRadians(current.latitude - previous.latitude)
    val dLon = Math.toRadians(current.longitude - previous.longitude)
    val latAvg = Math.toRadians((current.latitude + previous.latitude) / 2.0)

    val northDisplacement = dLat * earthRadius
    val eastDisplacement = dLon * earthRadius * kotlin.math.cos(latAvg)

    val velN = (northDisplacement / dt).toFloat().coerceIn(-100.0f, 100.0f)
    val velE = (eastDisplacement / dt).toFloat().coerceIn(-100.0f, 100.0f)
    val velD = ((previous.altitude - current.altitude) / dt).toFloat().coerceIn(-20.0f, 20.0f)

    return floatArrayOf(velN, velE, velD)
  }

  private val fusionUpdateRunnable = object : Runnable {
    override fun run() {
      if (!running || ekfPtr == 0L) return
      
      // Skip real sensor processing when X-Plane is connected
      if (xplaneConnected) {
        // Still schedule next update to keep the loop alive
        handler.postDelayed(this, targetIntervalNs / 1_000_000L)
        return
      }

      val currentTimeNs = System.nanoTime()
      val timestampUs = currentTimeNs / 1000

      if (nextTargetTime == 0L) {
        nextTargetTime = currentTimeNs + targetIntervalNs
      }

      val deltaTime = if (lastFrameTimeNs != 0L) {
        (currentTimeNs - lastFrameTimeNs) / 1_000_000_000.0f
      } else {
        1.0f / FRAME_RATE
      }

      // Copy sensor data
      val accelCopy: FloatArray
      val gyroCopy: FloatArray
      val magCopy: FloatArray
      synchronized(sensorLock) {
        accelCopy = accelData.clone()
        gyroCopy = gyroData.clone()
        magCopy = magData.clone()
      }

      // Get GPS data if new
      var gpsCopy: FloatArray? = null
      synchronized(locationLock) {
        val gps = locationData
        if (gps != null && (gps[8] > 0.5f) && timestampUs != lastUsedLocationTimestamp) {
          gpsCopy = gps.clone()
          lastUsedLocationTimestamp = timestampUs
        }
      }

      // Get barometer data if new
      var baroCopy: FloatArray? = null
      synchronized(baroLock) {
        val baro = baroData
        if (baro != null && baro.size >= 3 && (baro[2] > 0.5f) && timestampUs != lastUsedBaroTimestamp) {
          baroCopy = baro.clone()
          lastUsedBaroTimestamp = timestampUs
        }
      }

      // Update EKF
      updateAhrs(ekfPtr, timestampUs, accelCopy, gyroCopy, magCopy, gpsCopy, baroCopy)

      // Emit output at configured rate
      if (currentTimeNs > nextEmitTime) {
        val output = getAhrsOutput(ekfPtr)
        if (output != null && output.size >= 24) {
          val map = Arguments.createMap()
          val headingOffset = headingOffsetDeg()
          
          map.putDouble("roll", output[0].toDouble())
          map.putDouble("pitch", output[1].toDouble())
          map.putDouble("heading", normalizeHeadingDeg(output[2] + headingOffset))
          map.putDouble("flightPathAngle", output[3].toDouble())
          map.putDouble("horizontalFlightPathAngle", output[4].toDouble())
          map.putDouble("trackAngle", output[5].toDouble())
          map.putDouble("horizontalSpeed", output[6].toDouble())
          map.putDouble("totalSpeed", output[7].toDouble())
          map.putDouble("altitude", output[8].toDouble())
          map.putDouble("altitudeQNE", output[9].toDouble())
          map.putDouble("altitudeQNH", output[10].toDouble())
          map.putDouble("verticalSpeed", output[11].toDouble())
          // Get barometric pressure from baroData if available
          val baroPressure = synchronized(baroLock) {
            baroData?.getOrNull(0)?.toDouble() ?: 0.0
          }
          map.putDouble("barometricPressure", baroPressure)
          map.putDouble("velocityNorth", output[12].toDouble())
          map.putDouble("velocityEast", output[13].toDouble())
          map.putDouble("velocityDown", output[14].toDouble())
          val gpsPos = getGpsPosition(ekfPtr)
          if (gpsPos[0] != 0.0 || gpsPos[1] != 0.0) {
            map.putDouble("latitude", gpsPos[0])
            map.putDouble("longitude", gpsPos[1])
          }
          map.putInt("flightPhase", output[15].toInt())
          map.putDouble("flightPhaseConfidence", output[16].toDouble())
          map.putBoolean("attitudeValid", output[17] > 0.5f)
          map.putBoolean("altitudeValid", output[18] > 0.5f)
          map.putBoolean("positionValid", output[19] > 0.5f)
          map.putBoolean("flightPhaseValid", output[20] > 0.5f)
          
          emitOnAhrsUpdate(map)
        }
        nextEmitTime = currentTimeNs + emitRateHz.hz
      }

      // Schedule next update
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
