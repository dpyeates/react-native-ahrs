<div align="center">

# react-native-ahrs

**High-Performance Attitude and Heading Reference System for React Native**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

![](https://github.com/dpyeates/react-native-ahrs/blob/main/example/react-native-ahrs.gif)

</div>

**react-native-ahrs** is an Attitude and Heading Reference System (AHRS) for React Native iOS and Android. It fuses the devices onboard accelerometer, gyroscope, magnetometer, GPS, and barometer data via an Extended Kalman Filter (EKF) to provide attitude, heading, position, velocity, and flight-phase estimates.

## Features

- **18-state Extended Kalman Filter (EKF)** – Sensor fusion (position, velocity, quaternion attitude, sensor biases) with quaternion dynamics and analytical Jacobians
- **Multi-sensor fusion** – IMU (gyro, accel, mag), GPS, and barometer
- **60 Hz updates** – Configurable output rate (1–60 Hz)
- **Turbo Modules** – No legacy bridge; synchronous native calls
- **iOS and Android** – Native implementations for both platforms
- **Rich output** – Attitude, heading, altitude, velocity, position, flight phase, validity flags
- **Flight phase detector** – FSM-based classification (ground → takeoff → climb → cruise → descent → approach → landing) with confidence and validity
- **Recording & playback** – Record sensor data to gzipped JSON; replay for testing
- **X-Plane integration** – Feed the AHRS from X-Plane via WebSocket (X-Plane plugin required)

## Requirements

- **React Native New Architecture** (Fabric + Turbo Modules)
- **iOS**: CocoaPods, iOS 13.0+
- **Android**: New Architecture enabled, API 21+

## Installation

```sh
yarn add react-native-ahrs
# or
npm install react-native-ahrs
```

**iOS** (CocoaPods):

```sh
cd ios && pod install
```

Ensure **New Architecture** is enabled in your app.

**iOS** – in your `Podfile`:

```ruby
use_react_native!(
  :path => config[:reactNativePath],
  :fabric_enabled => true
)
```

**Android** – in `gradle.properties`:

```properties
newArchEnabled=true
```

## Quick Start

```tsx
import React, { useEffect, useState } from 'react';
import { View, Text } from 'react-native';
import { Ahrs, type AhrsData } from 'react-native-ahrs';

export default function App() {
  const [data, setData] = useState<AhrsData | null>(null);

  useEffect(() => {
    const unsubscribe = Ahrs.addListener(setData);
    Ahrs.start();

    return () => {
      unsubscribe();
      Ahrs.stop();
    };
  }, []);

  if (!data) return null;

  return (
    <View>
      <Text>Roll: {data.roll.toFixed(1)}°</Text>
      <Text>Pitch: {data.pitch.toFixed(1)}°</Text>
      <Text>Heading: {data.heading.toFixed(0)}°</Text>
      <Text>Altitude: {data.altitudeQNH.toFixed(0)} m</Text>
      <Text>Ground speed: {data.groundSpeed.toFixed(1)} m/s</Text>
    </View>
  );
}
```

**Important:** Call `addListener` before `start`. Unsubscribe and call `stop` when done.

TypeScript types `AhrsData`, `AhrsRotation`, `RecordingFile`, `PlaybackStateEvent`, and `XPlaneConnectionEvent` are exported from `react-native-ahrs`.

## API Reference

### Lifecycle

| Method | Description |
|--------|-------------|
| `addListener(callback: (data: AhrsData) => void): () => void` | Subscribe to AHRS updates. Returns unsubscribe function. |
| `start(): void` | Start sensor fusion. Requires at least one listener. |
| `stop(): void` | Stop sensor fusion. |
| `reset(): void` | Reset EKF state; reconvergence takes a few seconds. |
| `level(): void` | Set current attitude as zero reference (roll/pitch). Call when device is level. |
| `removeAllListeners(): void` | Remove all listeners and stop. |

### Configuration

| Method | Description |
|--------|-------------|
| `setRate(rate: number): void` | Output rate in Hz (1–60). Default 5. |
| `setRotation(rotation)` | Device orientation: `'none'`, `'left'`, or `'right'`. |
| `setQNH(qnh: number): void` | Sea-level pressure in hPa (e.g. 1013.25) for baro altitude. |

### Status

| Method | Description |
|--------|-------------|
| `getStatus(): { isRunning: boolean; listenerCount: number }` | Current run state and listener count. |
| `isSupported(): Promise<boolean>` | Resolves to `true` if required sensors are available. |

### Recording & playback

| Method | Description |
|--------|-------------|
| `startRecording(): void` | Start recording sensor data to a gzipped JSON file. |
| `stopRecording(): void` | Stop and finalize the recording. |
| `getRecordingFiles(): Promise<RecordingFile[]>` | List recording files (filename, size, date). |
| `deleteRecording(filename: string): void` | Delete a recording file. |
| `playbackRecording(filename: string): void` | Play back a recording (AHRS must be running). |
| `stopPlayback(): void` | Stop playback. |
| `addPlaybackListener(callback): () => void` | Listen for playback started/stopped/completed. |
| `isPlaybackActive(): boolean` | Whether playback is active. |

### X-Plane

| Method | Description |
|--------|-------------|
| `connectToXPlane(host: string): void` | Connect to X-Plane plugin at `host` (e.g. `"192.168.1.100"`). |
| `disconnectFromXPlane(): void` | Disconnect from X-Plane. |
| `addXPlaneConnectionListener(callback): () => void` | Listen for connect/disconnect events. |
| `isXPlaneConnected(): boolean` | Whether X-Plane is connected. |
| `getXPlaneHost(): string \| null` | Connected host or `null`. |

## Output data (`AhrsData`)

Each update provides:

| Field | Type | Description |
|-------|------|-------------|
| `roll` | `number` | Roll angle (°), -180 to 180, positive = right wing down |
| `pitch` | `number` | Pitch angle (°), -90 to 90, positive = nose up |
| `heading` | `number` | Magnetic heading (°), 0–360, from EKF filter (or X-Plane when connected) |
| `magneticDeclination` | `number` | Magnetic declination (°) |
| `groundTrack` | `number` | Direction of travel (°), 0–360 |
| `groundSpeed` | `number` | Horizontal speed (m/s) |
| `flightPathAngle` | `number` | Vertical flight path angle (°) |
| `horizontalFlightPathAngle` | `number` | Sideslip/crab angle (°) |
| `altitude` | `number` | GPS altitude MSL (m) |
| `altitudeQNE` | `number` | Baro altitude, standard atmosphere (m) |
| `altitudeQNH` | `number` | Baro altitude, QNH (m) |
| `verticalSpeed` | `number` | Vertical speed (m/s), positive = climb |
| `barometricPressure` | `number` | Pressure (hPa) |
| `velocityNorth`, `velocityEast`, `velocityDown` | `number` | NED velocity (m/s) |
| `latitude`, `longitude` | `number?` | Position (°) |
| `flightPhase` | `number` | 0=GROUND, 1=TAKEOFF, 2=CLIMB, 3=CRUISE, 4=DESCENT, 5=APPROACH, 6=LANDING |
| `flightPhaseConfidence` | `number` | 0–1 |
| `attitudeValid` | `boolean` | Roll, pitch, heading reliable |
| `altitudeValid` | `boolean` | Altitude estimates reliable |
| `positionValid` | `boolean` | Position reliable |
| `flightPhaseValid` | `boolean` | Flight phase reliable |

Use the `*Valid` flags to decide when to trust attitude, altitude, position, or flight phase.

## Usage examples

### Basic attitude display

```tsx
const unsubscribe = Ahrs.addListener((data) => {
  if (data.attitudeValid) {
    console.log(`Roll: ${data.roll}° Pitch: ${data.pitch}° Heading: ${data.heading}°`);
  }
});
Ahrs.start();
// ... later: unsubscribe(); Ahrs.stop();
```

### Aviation-style with QNH and rate

```tsx
Ahrs.setQNH(1013.25);  // hPa from METAR/ATIS
Ahrs.setRate(10);      // 10 Hz
const unsubscribe = Ahrs.addListener((data) => {
  if (data.altitudeValid) {
    console.log(`Alt QNH: ${data.altitudeQNH}m, VS: ${data.verticalSpeed} m/s`);
  }
});
Ahrs.start();
```

### Check support before use

```tsx
const ok = await Ahrs.isSupported();
if (!ok) {
  Alert.alert('AHRS not supported', 'This device does not have the required sensors.');
  return;
}
```

## Example app

The repo includes an example app with attitude display, recording, playback, and X-Plane connection. From the repo root:

```sh
yarn example start    # start Metro bundler
yarn example ios      # run on iOS
yarn example android  # run on Android
```

## How it works

The library runs an **18-state Extended Kalman Filter (EKF)** in C++ (see `fusionml/src/uNavINS.cpp`). The state vector comprises position (NED), velocity (NED), attitude (quaternion), and sensor biases (accelerometer, gyroscope, magnetometer). The filter fuses:

- **Accelerometer** – gravity for roll/pitch
- **Gyroscope** – angular rate
- **Magnetometer** – full 3D mag measurement with WMM expected field
- **GPS** – position, velocity
- **Barometer** – altitude (QNE/QNH)

Updates are produced at up to 60 Hz internally; you choose the JS output rate (1–60 Hz) via `setRate`. Magnetic declination is derived from position using the [World Magnetic Model](https://www.ncei.noaa.gov/products/world-magnetic-model) (WMM) via [XYZgeomag](https://github.com/nhz2/XYZgeomag).

### Filter architecture

The filter is an **18-state Extended Kalman Filter** in the North–East–Down (NED) frame. The **state** comprises position (3), velocity (3), attitude as Euler angles roll/pitch/yaw (3), and bias states for the accelerometer (3), gyroscope (3), and magnetometer (3). Attitude is propagated internally using a quaternion; the state holds the equivalent Euler angles for the correction step.

**Prediction** runs at the IMU rate (e.g. 60 Hz). The process model integrates specific force and angular rate from the IMU to propagate position, velocity, and attitude. Gravity is applied in the navigation frame. The three bias states are modelled as first-order Markov processes with configurable time constants, so the filter can learn and track sensor biases over time. **Rest detection** runs on the raw IMU: when the device is stationary (low gyro and accel variance over about 1 s), the gyro bias time constant is shortened and process noise increased so bias converges much faster; when motion is detected again, the filter returns to normal time constants. IMU samples are also written into a short buffer for **sensor delay compensation**: GPS is typically 100–300 ms behind the IMU, so the filter estimates delay from the GPS update rate (0.5–10 Hz) and scales GPS measurement noise by that delay so stale fixes are trusted less (no state rewind).

**Measurements** are applied when available. **GPS** provides position and velocity (6 dimensions) in NED; the observation is the difference between GPS and filter state. The measurement noise (R) is **adaptive**: if the platform reports horizontal, vertical, or speed accuracy, the filter uses the larger of nominal or reported accuracy so that poor GPS is trusted less and good GPS uses nominal R, giving smooth behaviour from open sky to urban or degraded conditions. Delay scaling is applied on top of that. **Magnetometer** updates observe only heading (yaw): the expected Earth field in body frame is computed from the current attitude and the WMM expected field in NED; the measured field (after subtracting the estimated mag bias) is compared to that expectation to form a yaw error, which is used in a scalar update. Before accepting a mag update, the filter applies **enhanced magnetic rejection**: the measured field must pass a magnitude check (0.5–2× expected strength), an inclination check (within about ±20° of expected dip), and a temporal check (no sudden change &gt; 10 µT), so power lines, metal, and transients are rejected while a clean field is accepted. **Barometer** altitude is fused as a separate scalar measurement when GPS vertical accuracy is poor or unavailable (&gt;10 m or no fix), giving a backup altitude and smoother behaviour in urban or tree cover; QNH is user-set, with QNE as fallback.

**Covariance** is updated using the Joseph form for numerical stability, and the state transition and observation Jacobians are computed analytically. The filter also performs **covariance health monitoring**: it checks for NaN/Inf in the state and covariance, enforces reasonable bounds on position, velocity, and attitude variances (with separate handling for yaw, which can be large without mag), and checks that bias estimates stay within physical limits; it exposes `isHealthy()` and `getHealthStatus()` (0=healthy, 1=warning, 2=error, 3=critical) for diagnostics or optional reset logic. The result is attitude (roll, pitch, heading), position (lat/lon/alt), velocity (NED and derived ground speed/track), and the bias estimates, which are used to correct the raw IMU and mag inputs on the next step.

## Troubleshooting

| Issue | What to do |
|-------|------------|
| **"Attempt to start Ahrs without any callbacks registered"** | Call `addListener` before `start`. |
| **No updates** | Ensure `isSupported()` is true, location permission granted (for GPS), and `start()` was called. |
| **Noisy attitude** | Call `level()` when the device is level. Avoid magnetic interference. |
| **Altitude wrong** | Set `setQNH` to local sea-level pressure (e.g. from METAR). |

## Testing

- **Unit tests (Jest):** `yarn test`
- **fusionml C++ tests:** `yarn test:fusion` (requires `clang++`/`g++` and zlib)
- **All:** `yarn test:all`

See [`fusionml/tests/README.md`](fusionml/tests/README.md) for fusion test details.

## Attribution

This project uses **[XYZgeomag](https://github.com/nhz2/XYZgeomag)** ([MIT](https://opensource.org/licenses/MIT)) for the World Magnetic Model (WMM), and builds on **[uNavINS](https://github.com/FlyTheThings/uNavINS)** for the core inertial/GPS EKF. The filter has been extended from the original 15-state design to an 18-state formulation with magnetometer fusion, and we have added sensor delay compensation, GPS adaptive noise, enhanced magnetic rejection, rest detection, covariance health monitoring, and barometer fusion so that it behaves well with variable GPS quality, magnetic interference, and stationary periods. The base uNavINS propagation and fusion logic is preserved.

## Contributing

Contributions are welcome. See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

## License

MIT

---

Made with [create-react-native-library](https://github.com/callstack/react-native-builder-bob)
