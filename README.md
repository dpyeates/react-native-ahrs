<div align="center">

# react-native-ahrs

</div>

react-native-ahrs is an Attitude And Heading Reference System (AHRS) for React Native iOS and Android devices. It uses the device's internal sensors (accelerometer, gyroscope, magnetometer) to provide roll, pitch and magnetic heading in real-time.

It is implemented using C/C++, JSI (JavaScript Interface), and Turbo Modules for maximum performance and low latency.

## Features

- High performance, low-latency sensor fusion (C/C++)
- JSI + C++ TurboModules (no legacy bridge, no Promises/async required)
- Fully synchronous native calls
- iOS and Android support

## Requirements

- React Native New Architecture only (Fabric + TurboModules). This package does not support the old bridge.
- iOS: CocoaPods
- Android: Android Gradle Plugin with New Architecture enabled

## Installation

```sh
yarn add react-native-ahrs
# iOS only
cd ios && pod install
```

> Note: Ensure New Architecture is enabled in your app before using this library.

## Usage

Basic example using hooks and a single listener:

```tsx
import React, { useEffect, useState } from 'react';
import { View, Text } from 'react-native';
import { Ahrs, type AhrsData } from 'react-native-ahrs';

export default function Example() {
  const [attitude, setAttitude] = useState<AhrsData>({
    roll: 0,
    pitch: 0,
    heading: 0,
  });

  useEffect(() => {
    // Register listener first
    const unsubscribe = Ahrs.addListener(setAttitude);

    // Start AHRS once a listener is registered
    Ahrs.start();

    // Optional configuration
    Ahrs.setRate(20); // 1–60 Hz
    Ahrs.setGain(3.0); // tune to preference/environment

    return () => {
      unsubscribe();
      Ahrs.stop();
    };
  }, []);

  return (
    <View>
      <Text>Pitch: {Math.round(attitude.pitch)}°</Text>
      <Text>Roll: {Math.round(attitude.roll)}°</Text>
      <Text>Heading: {Math.round(attitude.heading)}°</Text>
    </View>
  );
}
```

## API

All functions are available on the Ahrs singleton. Below you will find detailed behavior, parameters, return values, and examples for each function.

### API Quick Reference (Grid)

| Function              | Description                                     | Parameters                         | Returns / Notes                                                                |
| --------------------- | ----------------------------------------------- | ---------------------------------- | ------------------------------------------------------------------------------ |
| addListener(callback) | Subscribe to updates (roll, pitch, heading).    | callback: (data: AhrsData) => void | Returns unsubscribe(): () => void. Auto-stops when last listener unsubscribes. |
| removeAllListeners()  | Remove all listeners and stop AHRS.             | —                                  | Safe to call anytime.                                                          |
| start()               | Start native sensor fusion and emit updates.    | —                                  | No-op with warning if no listeners registered.                                 |
| stop()                | Stop native processing and updates.             | —                                  | Idempotent.                                                                    |
| reset()               | Reinitialize AHRS internal state.               | —                                  | Keeps rate/gain; useful after disturbances.                                    |
| level()               | Zero pitch and roll relative to current pose.   | —                                  | Heading unchanged.                                                             |
| setRate(newRate)      | Set update rate (Hz).                           | newRate: number (1–60)             | Values outside 1–60 warn and are ignored.                                      |
| setGain(gain)         | Set complementary filter gain.                  | gain: number                       | Typical start: 3.0; higher = more accel/mag influence.                         |
| setRotation(rotation) | Apply extra 90° step rotation for UI alignment. | rotation: 'none','left' ,'right'   | Use with your orientation/locking logic.                                       |
| getStatus()           | Get JS-side status.                             | —                                  | { isRunning: boolean, listenerCount: number }                                  |



#### addListener(callback: (data: AhrsData) => void): () => void

- Purpose: Subscribe to AHRS updates (roll, pitch, heading).
- Parameters: callback – a function receiving AhrsData at the configured rate.
- Returns: unsubscribe function to remove this callback. When the final listener unsubscribes, AHRS stops automatically.
- Behavior: Multiple listeners are supported; each receives the same AhrsData object values per tick.
- Notes: Register the listener before calling start(). Avoid long-running logic in the callback to keep UI responsive.
- Example:
  const unsubscribe = Ahrs.addListener((data) => { /_ use data.roll/pitch/heading _/ });
  // later
  unsubscribe();

#### removeAllListeners(): void

- Purpose: Remove every registered listener and stop AHRS if running.
- Behavior: Clears internal listener set and calls stop(). Safe to call even if there are no listeners.
- Use when: Tearing down a screen or resetting subscriptions globally.

#### start(): void

- Purpose: Begin native sensor fusion and start emitting updates to registered JS listeners.
- Behavior: No-ops with a warning if no listeners are registered. Subsequent calls while running are ignored.
- Side effects: Allocates/starts native sensors and AHRS processing.
- Errors: If native start fails, an error is thrown (and logged in DEV); wrap in try/catch if needed.

#### stop(): void

- Purpose: Stop native processing and halt updates.
- Behavior: Safe to call multiple times; subsequent calls while not running are ignored.
- When to call: On component unmount or when you temporarily don’t need updates to save battery/CPU.

#### reset(): void

- Purpose: Reinitialize AHRS internal state (e.g., reinitialize the filter/quaternion).
- Behavior: Does not change rate or gain settings. Useful after large disturbances or when you want to rebaseline.

#### level(): void

- Purpose: Zero pitch and roll relative to the current device orientation (i.e., treat current attitude as level).
- Behavior: Leaves heading unchanged. Use when the device rests on a surface that isn’t perfectly level but should be treated as such.
- UX tip: Provide a “Level” button that users can press while the device is in a desired reference pose.

#### setRate(newRate: number): void

- Purpose: Set the update rate in Hertz (Hz).
- Valid range: 1–60 Hz. Values outside this range trigger a console warning and are ignored.
- Behavior: Takes effect immediately for subsequent updates.
- Trade-offs: Higher rates increase responsiveness but use more CPU/battery; lower rates save power but increase latency.
- Example: Ahrs.setRate(20);

#### setGain(gain: number): void

- Purpose: Control the AHRS filter gain (complementary filter balance between gyro integration and accel/mag correction).
- Behavior: Larger gain increases responsiveness to accel/mag (less drift, but more sensitive to motion/magnetic noise). Smaller gain trusts gyro more (smoother during dynamic motion, but more drift over time).
- Typical starting point: 3.0. Tune per device/environment.
- Example: Ahrs.setGain(3.0);

#### setRotation(rotation: 'none' | 'left' | 'right'): void

- Purpose: Apply an extra 90° step rotation to align AHRS output with your UI orientation handling.
- Values:
  - 'none' – no extra rotation (portrait baseline).
  - 'left' – rotate 90° to match landscape-left UI.
  - 'right' – rotate 90° to match landscape-right UI.
- Behavior: Intended to keep roll/pitch intuitive when the app UI is rotated. Use together with your orientation/locking logic.
- Example: Ahrs.setRotation('left');

#### getStatus(): { isRunning: boolean; listenerCount: number }

- Purpose: Introspect current JS-side state.
- Fields:
  - isRunning – whether AHRS has been started and not yet stopped.
  - listenerCount – number of currently registered JS listeners.
- Use for: Debug UIs and guard logic (e.g., only show “Start” if not running).

## Common usage pattern

- const unsubscribe = Ahrs.addListener(setAttitude);
- Ahrs.setRate(20);
- Ahrs.setGain(3.0);
- Ahrs.start();
- // on unmount
- unsubscribe();
- Ahrs.stop();

## Types

### AhrsData

- roll: number – degrees, range roughly -180..+180 (− left, + right)
- pitch: number – degrees, range roughly -90..+90 (− down, + up)
- heading: number – degrees true, 0..360 (wraps around at 0/360)

## General lifecycle

- Register at least one listener via addListener before calling start. If you call start() with no listeners, a warning is logged and start is ignored.
- While running, updates are delivered synchronously on the JS thread via the registered callbacks.
- When the last listener is removed (including via the unsubscribe function), AHRS is automatically stopped.
- Stop AHRS in component unmount to release native resources.

## AHRS Algorithm

react-native-ahrs uses the revised AHRS algorithm presented in chapter 7 of [Madgwick's PhD thesis](https://x-io.co.uk/downloads/madgwick-phd-thesis.pdf).

This is a different algorithm to the better-known initial AHRS algorithm presented in chapter 3, commonly referred to as the "Madgwick algorithm".

For further background, reference implementations, and additional documentation, see the xioTechnologies Fusion library: [xioTechnologies/Fusion](https://github.com/xioTechnologies/Fusion).

The algorithm calculates the orientation as the integration of the gyroscope summed with a feedback term. The feedback term is equal to the error in the current measurement of orientation as determined by the other sensors, multiplied by a gain. The algorithm therefore functions as a complementary filter that combines high-pass filtered gyroscope measurements with low-pass filtered measurements from other sensors with a corner frequency determined by the gain. A low gain will 'trust' the gyroscope more and so be more susceptible to drift. A high gain will increase the influence of other sensors and the errors that result from accelerations and magnetic distortions. A gain of zero will ignore the other sensors so that the measurement of orientation is determined by only the gyroscope.
## Example App

See the example app in example/ for a complete UI demonstrating rate, gain and rotation controls.

## Troubleshooting

- “Attempt to start Ahrs without any callbacks registered”: Ensure addListener is called before start().
- No updates arriving: Confirm you didn’t unsubscribe all listeners; check getStatus().
- Noisy heading or pitch/roll: Try lowering gain during dynamic motion, or use level() while stationary.

## Contributing

If you need any specific function from react-native-ahrs, please request it through an issue and it will implemented in the nearest time, or feel free to open a PR and it will be added to the library promptly.

See the [contributing guide](CONTRIBUTING.md) to learn how to contribute to the repository and the development workflow.

## Limitations

- This package supports only the New Architecture (Fabric + TurboModules)

## License

MIT

---

Made with [create-react-native-library](https://github.com/callstack/react-native-builder-bob)
