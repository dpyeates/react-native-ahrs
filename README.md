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

The library runs an **18-state Extended Kalman Filter (EKF)** in C++. The state vector comprises position (NED), velocity (NED), attitude (quaternion), and sensor biases (accelerometer, gyroscope, magnetometer). The filter fuses:

- **Accelerometer** – gravity for roll/pitch
- **Gyroscope** – angular rate
- **Magnetometer** – full 3D mag measurement with WMM expected field
- **GPS** – position, velocity
- **Barometer** – altitude (QNE/QNH)

Updates are produced at up to 60 Hz internally; you choose the JS output rate (1–60 Hz) via `setRate`. Magnetic declination is derived from position using the [World Magnetic Model](https://www.ncei.noaa.gov/products/world-magnetic-model) (WMM) via [XYZgeomag](https://github.com/nhz2/XYZgeomag).

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

This project uses or draws from the following open-source work:

- **[XYZgeomag](https://github.com/nhz2/XYZgeomag)** ([MIT](https://opensource.org/licenses/MIT)) — Lightweight C++ header-only library for the World Magnetic Model (WMM). Used to compute magnetic declination from position. Compatible with WMM2025.
- **[uNavINS](https://github.com/FlyTheThings/uNavINS)** — Inertial navigation EKF (originally 15-state) for attitude, position, and velocity from IMU and GPS. This library uses modified, **18-state Extended Kalman Filter (EKF)** based on the uNav INS approach, extended with magnetometer bias states and full 3D magnetometer fusion.

## Contributing

Contributions are welcome. See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

## License

MIT

---

Made with [create-react-native-library](https://github.com/callstack/react-native-builder-bob)
