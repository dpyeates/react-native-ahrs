/**
 * App.tsx
 *
 * Main application component for the React Native AHRS example app.
 * Provides a user interface for:
 * - Displaying real-time aircraft attitude and navigation data
 * - Controlling AHRS settings (rate, QNH, orientation)
 * - Recording and playback of flight data
 * - X-Plane flight simulator integration
 * - Flight phase detection display
 *
 * @module App
 */

import { useEffect, useState, useMemo, useRef, useCallback } from 'react';
import {
  Alert,
  KeyboardAvoidingView,
  Platform,
  StyleSheet,
  Text,
  TouchableHighlight,
  View,
  ScrollView,
  TextInput,
} from 'react-native';
import Slider from '@react-native-community/slider';
import AsyncStorage from '@react-native-async-storage/async-storage';
import RNOrientationDirector, {
  Orientation,
  OrientationType,
} from 'react-native-orientation-director';
import OrientationVisualizer from './OrientationVisualizer';
import { Ahrs, type AhrsData, type RecordingFile } from 'react-native-ahrs';

/** AsyncStorage key for persisting X-Plane host address */
const XPLANE_HOST_STORAGE_KEY = '@ahrs:xplane_host';

// Rate values (Hz) - valid output rates for AHRS
const RATE_VALUES: readonly number[] = [1, 5, 10, 20, 40, 60];

/**
 * Creates the initial attitude state with all values set to defaults.
 * Used for component initialization.
 * @returns AhrsData object with all fields initialized to zero/false
 */
function getInitialAttitude(): AhrsData {
  return {
    roll: 0,
    pitch: 0,
    heading: 0,
    magneticDeclination: 0,
    groundTrack: 0,
    groundSpeed: 0,
    flightPathAngle: 0,
    horizontalFlightPathAngle: 0,
    altitude: 0,
    altitudeQNE: 0,
    altitudeQNH: 0,
    verticalSpeed: 0,
    barometricPressure: 0,
    velocityNorth: 0,
    velocityEast: 0,
    velocityDown: 0,
    latitude: undefined,
    longitude: undefined,
    flightPhase: 0,
    flightPhaseConfidence: 0,
    attitudeValid: false,
    altitudeValid: false,
    positionValid: false,
    flightPhaseValid: false,
  };
}

/**
 * Creates a safe data object by ensuring all numeric fields have default values.
 * Prevents null/undefined values from causing rendering issues.
 * @param data - Raw AHRS data that may contain null/undefined values
 * @returns AhrsData object with all numeric fields guaranteed to have values
 */
function createSafeData(data: AhrsData): AhrsData {
  return {
    ...data,
    roll: data.roll ?? 0,
    pitch: data.pitch ?? 0,
    heading: data.heading ?? 0,
    magneticDeclination: data.magneticDeclination ?? 0,
    groundTrack: data.groundTrack ?? 0,
    groundSpeed: data.groundSpeed ?? 0,
    flightPathAngle: data.flightPathAngle ?? 0,
    horizontalFlightPathAngle: data.horizontalFlightPathAngle ?? 0,
    altitude: data.altitude ?? 0,
    altitudeQNE: data.altitudeQNE ?? 0,
    altitudeQNH: data.altitudeQNH ?? 0,
    verticalSpeed: data.verticalSpeed ?? 0,
    barometricPressure: data.barometricPressure ?? 0,
    velocityNorth: data.velocityNorth ?? 0,
    velocityEast: data.velocityEast ?? 0,
    velocityDown: data.velocityDown ?? 0,
    flightPhase: data.flightPhase ?? 0,
    flightPhaseConfidence: data.flightPhaseConfidence ?? 0,
  };
}

/**
 * Validates if a string is a valid IPv4 address.
 * @param ip - IP address string to validate
 * @returns true if valid IPv4 address, false otherwise
 */
function isValidIPv4(ip: string): boolean {
  const ipv4Regex =
    /^(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$/;
  return ipv4Regex.test(ip.trim());
}

/**
 * Filters input to only allow valid IPv4 address characters (digits and dots).
 * @param text - Input text
 * @returns Filtered text containing only valid IPv4 characters
 */
function filterIPv4Input(text: string): string {
  // Remove any characters that aren't digits or dots
  return text.replace(/[^0-9.]/g, '');
}

/**
 * Status indicator component that displays a colored dot with text.
 * Used throughout the app to show connection status, recording state, etc.
 * @param color - Color for both the dot and text
 * @param text - Text to display next to the dot
 */
function StatusIndicator({ color, text }: { color: string; text: string }) {
  return (
    <View style={styles.statusIndicatorContainer}>
      <View style={[styles.statusIndicatorDot, { backgroundColor: color }]} />
      <Text style={[styles.statusValue, { color }]}>{text}</Text>
    </View>
  );
}

// QNH pressure limits (hPa)
const QNH_MIN = 930;
const QNH_MAX = 1080;

// Status indicator dimensions
const STATUS_INDICATOR_SIZE = 12;
const STATUS_INDICATOR_RADIUS = 6;

// Color constants
const COLORS = {
  SUCCESS: '#4caf50',
  ERROR: '#f44336',
  WARNING: '#ff9800',
  PRIMARY: '#2196f3',
  PURPLE: '#9c27b0',
  DISABLED: '#ccc',
  TEXT_PRIMARY: '#333',
  TEXT_SECONDARY: '#666',
  BORDER: '#e0e0e0',
} as const;

/**
 * Main App component.
 * Manages AHRS state, UI controls, recording/playback, and X-Plane integration.
 * Renders the orientation visualizer and control panels.
 *
 * @returns React component for the AHRS example app
 */
export default function App() {
  const [currentOrientation, setCurrentOrientation] = useState<Orientation>(
    Orientation.portrait
  );
  const [attitude, setAttitude] = useState<AhrsData>(getInitialAttitude());
  const [rate, setRate] = useState<number>(RATE_VALUES[3] as number); // Default to 20 Hz
  const [isRecording, setIsRecording] = useState<boolean>(false);
  const [isPlaying, setIsPlaying] = useState<boolean>(false);
  const [recordingFiles, setRecordingFiles] = useState<RecordingFile[]>([]);

  // QNH pressure setting (hPa) - default to standard atmosphere
  const [qnh, setQnh] = useState<number>(1013.25);

  // X-Plane connection state
  const [xplaneHost, setXplaneHost] = useState<string>('');
  const [xplaneConnected, setXplaneConnected] = useState<boolean>(false);
  const [xplaneConnecting, setXplaneConnecting] = useState<boolean>(false);
  
  // Handle IP address input with filtering
  const handleXPlaneHostChange = (text: string) => {
    const filtered = filterIPv4Input(text);
    setXplaneHost(filtered);
  };
  
  // Check if current IP address is valid
  const isXPlaneHostValid = xplaneHost.trim() !== '' && isValidIPv4(xplaneHost.trim());

  // Load saved X-Plane host on mount
  useEffect(() => {
    AsyncStorage.getItem(XPLANE_HOST_STORAGE_KEY)
      .then((savedHost) => {
        if (savedHost) {
          setXplaneHost(savedHost);
        }
      })
      .catch((error) => {
        console.warn('Failed to load saved X-Plane host:', error);
      });
  }, []);

  /**
   * Converts orientation enum to human-readable string for display.
   */
  const rotationString = useMemo(() => {
    switch (currentOrientation) {
      default:
      case Orientation.portrait:
        return 'Portrait';
      case Orientation.landscapeLeft:
        return 'Landscape Left';
      case Orientation.landscapeRight:
        return 'Landscape Right';
      case Orientation.portraitUpsideDown:
        return 'Portrait Upside Down';
    }
  }, [currentOrientation]);

  /**
   * Converts flight phase enum to human-readable string for display.
   * Flight phases: 0=Ground, 1=Takeoff Roll, 2=Takeoff, 3=Climb, 4=Cruise,
   * 5=Descent, 6=Approach, 7=Landing, 8=Landing Roll
   */
  const flightPhaseString = useMemo(() => {
    switch (attitude.flightPhase) {
      case 0:
        return 'Ground';
      case 1:
        return 'Takeoff Roll';
      case 2:
        return 'Takeoff';
      case 3:
        return 'Climb';
      case 4:
        return 'Cruise';
      case 5:
        return 'Descent';
      case 6:
        return 'Approach';
      case 7:
        return 'Landing';
      case 8:
        return 'Landing Roll';
      default:
        return 'Unknown';
    }
  }, [attitude.flightPhase]);

  /**
   * Converts flight phase confidence (0-1) to percentage (0-100) for display.
   */
  const flightPhaseConfidencePercent = useMemo(() => {
    return Math.round(attitude.flightPhaseConfidence * 100);
  }, [attitude.flightPhaseConfidence]);

  /**
   * Rounds and formats attitude values for display.
   * Angles are rounded to integers, speeds/altitudes to 1 decimal place.
   */
  const roundedAttitude = useMemo(
    () => ({
      roll: Math.round(attitude.roll),
      pitch: Math.round(attitude.pitch),
      heading: Math.round(attitude.heading),
      flightPathAngle: (attitude.flightPathAngle ?? 0).toFixed(1),
      horizontalFlightPathAngle: (
        attitude.horizontalFlightPathAngle ?? 0
      ).toFixed(1),
      altitudeQNE: Math.round(attitude.altitudeQNE),
      altitudeQNH: Math.round(attitude.altitudeQNH),
      verticalSpeed: (attitude.verticalSpeed ?? 0).toFixed(1),
    }),
    [attitude]
  );

  const listenerUnsubscribeRef = useRef<(() => void) | null>(null);

  /**
   * Initialize app on mount:
   * - Lock screen orientation to portrait
   * - Start AHRS system
   * Cleanup on unmount:
   * - Stop AHRS
   * - Unlock screen orientation
   */
  useEffect(() => {
    // Initially lock to portrait orientation
    RNOrientationDirector.lockTo(Orientation.portrait, OrientationType.device);
    setCurrentOrientation(Orientation.portrait);
    // Start the AHRS system
    startAhrs().catch((error) => {
      console.error('Failed to start AHRS:', error);
    });
    // Cleanup function
    return () => {
      stopAhrs();
      RNOrientationDirector.unlock();
    };
  }, []);

  /**
   * Configures and starts the AHRS system.
   * Must be called in this order:
   * 1. Check device support
   * 2. Register listener (required before start())
   * 3. Configure QNH, rotation, and rate (where needed or accept defaults)
   * 4. Start the system
   */
  async function startAhrs(): Promise<void> {
    try {
      // 1. Check if this device supports AHRS
      const supported = await Ahrs.isSupported();
      if (!supported) {
        Alert.alert(
          'AHRS Not Supported',
          'This device does not support AHRS functionality. The app requires sensors (gyroscope, accelerometer, magnetometer) that are not available on this device.',
          [{ text: 'OK' }]
        );
        return;
      }

      // 2. Register the callback that will receive the AHRS updates FIRST
      // This must be done before starting, as start() requires at least one listener
      listenerUnsubscribeRef.current = Ahrs.addListener((data) => {
        // Ensure all numeric fields are not null/undefined
        const safeData = createSafeData(data);
        setAttitude(safeData);
      });

      // 3. Optionally set QNH (altimeter setting) - default is standard atmosphere (1013.25 hPa)
      Ahrs.setQNH(qnh);

      // 4. Optionally configure device rotation based on mounting orientation - default is portrait
      // Options: 'none', 'left', or 'right'
      Ahrs.setRotation('none');

      // 5. Optionally set the desired output rate (1, 5, 10, 20, 40, 60 Hz) - default is 5Hz
      // We set 20Hz here for the example app (user can change this in the UI)
      Ahrs.setRate(RATE_VALUES[3] as number);

      // 6. Start AHRS
      Ahrs.start();
    } catch (error) {
      console.error('Error starting AHRS:', error);
      throw error;
    }
  }

  /**
   * Stops the AHRS system and cleans up the listener.
   * Called on component unmount or when stopping the app.
   */
  async function stopAhrs() {
    Ahrs.stop();
    listenerUnsubscribeRef.current?.();
    listenerUnsubscribeRef.current = null;
  }

  /**
   * Levels the AHRS system to capture the current attitude as a "zero" reference.
   * This calibrates the system to treat the current orientation as level.
   */
  function handleLevelPress() {
    Ahrs.level();
  }

  /**
   * Handles QNH (altimeter setting) pressure change from slider.
   * Updates both local state and the AHRS system.
   * @param value - QNH pressure in hPa (930-1080)
   */
  function handleQnhChange(value: number) {
    setQnh(value);
    Ahrs.setQNH(value);
  }

  /**
   * Rotates the AHRS system and screen orientation.
   * Cycles through: Portrait → Landscape Left → Landscape Right → Portrait
   * Updates both the device orientation lock and AHRS rotation setting.
   */
  function handleRotatePress() {
    let nextOrientation: Orientation;
    let rotation: 'none' | 'left' | 'right';

    // Cycle through orientations based on the current state
    if (currentOrientation === Orientation.portrait) {
      nextOrientation = Orientation.landscapeLeft;
      rotation = 'left';
    } else if (currentOrientation === Orientation.landscapeLeft) {
      nextOrientation = Orientation.landscapeRight;
      rotation = 'right';
    } else if (currentOrientation === Orientation.landscapeRight) {
      nextOrientation = Orientation.portrait;
      rotation = 'none';
    } else {
      // Fallback: if in any other orientation, go to portrait
      nextOrientation = Orientation.portrait;
      rotation = 'none';
    }

    // Update state and lock orientation
    setCurrentOrientation(nextOrientation);
    Ahrs.setRotation(rotation);
    RNOrientationDirector.lockTo(nextOrientation, OrientationType.device);
  }

  /**
   * Gets the next rate value in the sequence based on direction.
   * Uses the RATE_VALUES array to find the next valid rate.
   * @param currentRate - Current rate value
   * @param direction - 'increase' or 'decrease'
   * @returns Next rate value in the sequence, or current if at limit
   */
  function getNextRate(
    currentRate: number,
    direction: 'increase' | 'decrease'
  ): number {
    const currentIndex = RATE_VALUES.indexOf(currentRate);
    if (currentIndex === -1) {
      // If current rate is not in valid values, default to 20 Hz (index 3)
      return RATE_VALUES[3] as number;
    }

    if (direction === 'increase') {
      if (currentIndex < RATE_VALUES.length - 1) {
        return RATE_VALUES[currentIndex + 1] as number;
      }
      return RATE_VALUES[RATE_VALUES.length - 1] as number;
    } else {
      if (currentIndex > 0) {
        return RATE_VALUES[currentIndex - 1] as number;
      }
      return RATE_VALUES[0] as number;
    }
  }

  /**
   * Updates the AHRS output rate when local rate state changes.
   * This effect ensures the native module stays in sync with UI state.
   */
  useEffect(() => {
    Ahrs.setRate(rate);
  }, [rate]);

  /**
   * Loads the list of available recording files from the AHRS system.
   * Called on mount and after recording/playback operations.
   */
  const loadRecordingFiles = useCallback(async () => {
    try {
      const files = await Ahrs.getRecordingFiles();
      setRecordingFiles(files);
    } catch (error) {
      console.error('Failed to load recording files:', error);
    }
  }, []);

  /**
   * Load recording files list on our component mount.
   */
  useEffect(() => {
    loadRecordingFiles();
  }, [loadRecordingFiles]);

  /**
   * Track playback state from native module.
   * Updates UI when playback starts/stops and refreshes the file list when playback ends.
   */
  useEffect(() => {
    const unsubscribe = Ahrs.addPlaybackListener((event) => {
      if (event.status === 'started') {
        setIsPlaying(true);
        return;
      }
      setIsPlaying(false);
      loadRecordingFiles();
    });
    return () => {
      unsubscribe();
    };
  }, [loadRecordingFiles]);

  /**
   * Track X-Plane connection state from the native module.
   * Updates connection status and saves the host address when connected.
   */
  useEffect(() => {
    const unsubscribe = Ahrs.addXPlaneConnectionListener((event) => {
      setXplaneConnected(event.connected);
      setXplaneConnecting(false);
      if (event.connected) {
        setXplaneHost(event.host);
      }
    });
    return () => {
      unsubscribe();
    };
  }, []);

  /**
   * Starts recording AHRS data to a file.
   */
  function handleStartRecording() {
    Ahrs.startRecording();
    setIsRecording(true);
  }

  /**
   * Stops recording and refreshes the file list.
   */
  function handleStopRecording() {
    Ahrs.stopRecording();
    setIsRecording(false);
    loadRecordingFiles();
  }

  /**
   * Starts playback of a recorded file.
   * @param filename - Name of the recording file to play
   */
  function handlePlayback(filename: string) {
    Ahrs.playbackRecording(filename);
    setIsPlaying(true);
  }

  /**
   * Stops playback of the current recording.
   */
  function handleStopPlayback() {
    Ahrs.stopPlayback();
  }

  /**
   * Deletes a recording file and refreshes the file list.
   * @param filename - Name of the recording file to delete
   */
  async function handleDeleteRecording(filename: string) {
    Ahrs.deleteRecording(filename);
    await loadRecordingFiles();
  }

  /**
   * Connects to X-Plane flight simulator.
   * Validates the host address, saves it for next time, and initiates connection.
   */
  function handleXPlaneConnect() {
    const host = xplaneHost.trim();
    if (!host) {
      Alert.alert('Invalid IP Address', 'Please enter a valid IPv4 address.');
      return;
    }
    
    if (!isValidIPv4(host)) {
      Alert.alert(
        'Invalid IP Address',
        'Please enter a valid IPv4 address (e.g., 192.168.1.100)'
      );
      return;
    }
    
    // Save the host for next time
    AsyncStorage.setItem(XPLANE_HOST_STORAGE_KEY, host).catch((error) => {
      console.warn('Failed to save X-Plane host:', error);
    });
    setXplaneConnecting(true);
    Ahrs.connectToXPlane(host);
  }

  /**
   * Disconnects from X-Plane flight simulator.
   */
  function handleXPlaneDisconnect() {
    Ahrs.disconnectFromXPlane();
  }

  return (
    <View
      style={[
        styles.container,
        {
          flexDirection:
            currentOrientation === Orientation.portrait ? 'column' : 'row',
        },
      ]}
    >
      <OrientationVisualizer
        attitude={{
          roll: attitude.roll,
          pitch: attitude.pitch,
          heading: attitude.heading,
          flightPathAngle: attitude.flightPathAngle ?? 0,
          horizontalFlightPathAngle: attitude.horizontalFlightPathAngle ?? 0,
        }}
      />
      <KeyboardAvoidingView
        style={styles.controlsContainer}
        behavior={Platform.OS === 'ios' ? 'padding' : 'height'}
        keyboardVerticalOffset={Platform.OS === 'ios' ? 0 : 20}
      >
        <ScrollView
          style={{ flex: 1 }}
          contentContainerStyle={styles.controlsContent}
          showsVerticalScrollIndicator={true}
          keyboardShouldPersistTaps="handled"
          keyboardDismissMode="on-drag"
        >
        {/* Status Section */}
        <View style={styles.statusSection}>
          <Text style={styles.sectionTitle}>Status</Text>
          <View style={styles.dataRow}>
            <Text style={styles.statusLabel}>Data Source:</Text>
            <Text
              style={[
                styles.statusValue,
                {
                  color: xplaneConnected ? COLORS.PURPLE : COLORS.TEXT_PRIMARY,
                },
              ]}
            >
              {xplaneConnected ? 'X-Plane' : 'Device Sensors'}
            </Text>
          </View>
          <View style={styles.dataRow}>
            <Text style={styles.statusLabel}>Orientation:</Text>
            <Text style={styles.statusValue}>{rotationString}</Text>
          </View>
          <View style={styles.dataRow}>
            <Text style={styles.statusLabel}>Rate:</Text>
            <Text style={styles.statusValue}>{rate} Hz</Text>
          </View>
        </View>

        {/* Flight Phase Section */}
        <View style={styles.dataSection}>
          <Text style={styles.sectionTitle}>Flight Phase</Text>
          <View style={styles.dataRow}>
            <Text style={styles.dataLabel}>Valid:</Text>
            <Text style={styles.dataValue}>
              {attitude.flightPhaseValid ? 'TRUE' : 'FALSE'}
            </Text>
          </View>
          <View style={styles.dataRow}>
            <Text style={styles.dataLabel}>Phase:</Text>
            <Text style={styles.dataValue}>{flightPhaseString}</Text>
          </View>
          <View style={styles.dataRow}>
            <Text style={styles.dataLabel}>Confidence:</Text>
            <Text style={styles.dataValue}>
              {flightPhaseConfidencePercent}%
            </Text>
          </View>
        </View>

        {/* Attitude Section */}
        <View style={styles.dataSection}>
          <Text style={styles.sectionTitle}>Attitude</Text>
          <View style={styles.dataRow}>
            <Text style={styles.dataLabel}>Valid:</Text>
            <Text style={styles.dataValue}>
              {attitude.attitudeValid ? 'TRUE' : 'FALSE'}
            </Text>
          </View>
          <View style={styles.dataRow}>
            <Text style={styles.dataLabel}>Roll:</Text>
            <Text style={styles.dataValue}>{roundedAttitude.roll}°</Text>
          </View>
          <View style={styles.dataRow}>
            <Text style={styles.dataLabel}>Pitch:</Text>
            <Text style={styles.dataValue}>{roundedAttitude.pitch}°</Text>
          </View>
          <View style={styles.dataRow}>
            <Text style={styles.dataLabel}>Heading:</Text>
            <Text style={styles.dataValue}>{roundedAttitude.heading}°</Text>
          </View>
        </View>

        {/* Altitude Section */}
        <View style={styles.dataSection}>
          <Text style={styles.sectionTitle}>Altitude</Text>
          <View style={styles.dataRow}>
            <Text style={styles.dataLabel}>Valid:</Text>
            <Text style={styles.dataValue}>
              {attitude.altitudeValid ? 'TRUE' : 'FALSE'}
            </Text>
          </View>
          <View style={styles.dataRow}>
            <Text style={styles.dataLabel}>V/S:</Text>
            <Text style={styles.dataValue}>
              {roundedAttitude.verticalSpeed} m/s
            </Text>
          </View>
          <View style={styles.dataRow}>
            <Text style={styles.dataLabel}>GPS:</Text>
            <Text style={styles.dataValue}>
              {Math.round(attitude.altitude)} m
            </Text>
          </View>
          <View style={styles.dataRow}>
            <Text style={styles.dataLabel}>QNE:</Text>
            <Text style={styles.dataValue}>
              {roundedAttitude.altitudeQNE} m
            </Text>
          </View>
          <View style={styles.dataRow}>
            <Text style={styles.dataLabel}>QNH:</Text>
            <Text style={styles.dataValue}>
              {roundedAttitude.altitudeQNH} m
            </Text>
          </View>
          <View style={styles.dataRow}>
            <Text style={styles.dataLabel}>Baro Pressure:</Text>
            <Text style={styles.dataValue}>
              {attitude.barometricPressure > 0
                ? `${attitude.barometricPressure.toFixed(1)} hPa`
                : 'N/A'}
            </Text>
          </View>
          <View style={styles.dataRow}>
            <Text style={styles.dataLabel}>QNH Pressure:</Text>
            <Text style={styles.dataValue}>{qnh.toFixed(1)} hPa</Text>
          </View>
          <View style={styles.sliderContainer}>
            <View style={styles.sliderLabelRow}>
              <Text style={styles.sliderLabel}>{QNH_MIN}</Text>
              <Text style={styles.sliderLabel}>{QNH_MAX}</Text>
            </View>
            <Slider
              style={styles.slider}
              minimumValue={QNH_MIN}
              maximumValue={QNH_MAX}
              value={qnh}
              onValueChange={handleQnhChange}
              step={0.1}
              minimumTrackTintColor="#007AFF"
              maximumTrackTintColor={COLORS.BORDER}
              thumbTintColor="#007AFF"
            />
          </View>
        </View>

        {/* Positional Data Section */}
        <View style={styles.dataSection}>
          <Text style={styles.sectionTitle}>Positional Data</Text>
          <View style={styles.dataRow}>
            <Text style={styles.dataLabel}>Valid:</Text>
            <Text style={styles.dataValue}>
              {attitude.positionValid ? 'TRUE' : 'FALSE'}
            </Text>
          </View>
          {attitude.latitude !== undefined &&
            attitude.longitude !== undefined && (
              <>
                <View style={styles.dataRow}>
                  <Text style={styles.dataLabel}>Latitude:</Text>
                  <Text style={styles.dataValue}>
                    {(attitude.latitude ?? 0).toFixed(6)}°
                  </Text>
                </View>
                <View style={styles.dataRow}>
                  <Text style={styles.dataLabel}>Longitude:</Text>
                  <Text style={styles.dataValue}>
                    {(attitude.longitude ?? 0).toFixed(6)}°
                  </Text>
                </View>
                <View style={styles.dataRow}>
                  <Text style={styles.dataLabel}>Mag Declination:</Text>
                  <Text style={styles.dataValue}>
                    {attitude.magneticDeclination >= 0 ? '+' : ''}
                    {attitude.magneticDeclination.toFixed(1)}°
                  </Text>
                </View>
              </>
            )}
          <View style={styles.dataRow}>
            <Text style={styles.dataLabel}>Ground Speed:</Text>
            <Text style={styles.dataValue}>
              {(attitude.groundSpeed ?? 0).toFixed(1)} m/s
            </Text>
          </View>
          <View style={styles.dataRow}>
            <Text style={styles.dataLabel}>Vertical FPA:</Text>
            <Text style={styles.dataValue}>
              {attitude.flightPathAngle !== undefined
                ? `${attitude.flightPathAngle >= 0 ? '+' : ''}${roundedAttitude.flightPathAngle}°`
                : 'N/A'}
            </Text>
          </View>
          <View style={styles.dataRow}>
            <Text style={styles.dataLabel}>Horizontal FPA:</Text>
            <Text style={styles.dataValue}>
              {attitude.horizontalFlightPathAngle !== undefined
                ? `${attitude.horizontalFlightPathAngle >= 0 ? '+' : ''}${roundedAttitude.horizontalFlightPathAngle}°`
                : 'N/A'}
            </Text>
          </View>
        </View>

        {/* X-Plane Connection Section */}
        <View style={styles.dataSection}>
          <Text style={styles.sectionTitle}>X-Plane Connection</Text>
          <View style={styles.statusRow}>
            <Text style={styles.statusLabel}>Status:</Text>
            <StatusIndicator
              color={
                xplaneConnected
                  ? COLORS.SUCCESS
                  : xplaneConnecting
                    ? COLORS.WARNING
                    : COLORS.TEXT_SECONDARY
              }
              text={
                xplaneConnected
                  ? 'CONNECTED'
                  : xplaneConnecting
                    ? 'CONNECTING...'
                    : 'DISCONNECTED'
              }
            />
          </View>
          {xplaneConnected && (
            <View style={styles.statusRow}>
              <Text style={styles.statusLabel}>Host:</Text>
              <Text style={styles.statusValue}>{xplaneHost}</Text>
            </View>
          )}
          {!xplaneConnected && (
            <View style={styles.inputRow}>
              <TextInput
                style={styles.textInput}
                placeholder="X-Plane IP (e.g., 192.168.1.100)"
                placeholderTextColor="#999"
                value={xplaneHost}
                onChangeText={handleXPlaneHostChange}
                autoCapitalize="none"
                autoCorrect={false}
                keyboardType={Platform.OS === 'ios' ? 'numbers-and-punctuation' : 'default'}
                returnKeyType="done"
                onSubmitEditing={handleXPlaneConnect}
                editable={!xplaneConnecting}
                blurOnSubmit={true}
              />
            </View>
          )}
          {!xplaneConnected ? (
            <TouchableHighlight
              onPress={handleXPlaneConnect}
              underlayColor="#e0e0e0"
              style={styles.buttonWrapper}
              disabled={xplaneConnecting || !isXPlaneHostValid}
            >
              <View
                style={[
                  styles.button,
                  {
                    backgroundColor:
                      xplaneConnecting || !isXPlaneHostValid
                        ? COLORS.DISABLED
                        : COLORS.PURPLE,
                  },
                ]}
              >
                <Text style={styles.buttonText}>
                  {xplaneConnecting ? 'Connecting...' : 'Connect to X-Plane'}
                </Text>
              </View>
            </TouchableHighlight>
          ) : (
            <TouchableHighlight
              onPress={handleXPlaneDisconnect}
              underlayColor="#e0e0e0"
              style={styles.buttonWrapper}
            >
              <View style={[styles.button, { backgroundColor: COLORS.PURPLE }]}>
                <Text style={styles.buttonText}>Disconnect from X-Plane</Text>
              </View>
            </TouchableHighlight>
          )}
        </View>

        {/* Controls Section */}
        <View style={styles.controlsSection}>
          <TouchableHighlight
            onPress={handleRotatePress}
            accessibilityLabel="Rotate orientation"
            accessibilityRole="button"
            style={styles.buttonWrapper}
          >
            <View style={styles.button}>
              <Text style={styles.buttonText}>Rotate</Text>
            </View>
          </TouchableHighlight>

          <View style={styles.buttonRow}>
            <TouchableHighlight
              onPress={handleLevelPress}
              underlayColor="#e0e0e0"
              accessibilityLabel="Level the AHRS system"
              accessibilityRole="button"
              style={styles.buttonWrapper}
            >
              <View style={styles.button}>
                <Text style={styles.buttonText}>Level</Text>
              </View>
            </TouchableHighlight>
            <TouchableHighlight
              onPress={() => Ahrs.reset()}
              underlayColor="#e0e0e0"
              style={styles.buttonWrapper}
            >
              <View style={styles.button}>
                <Text style={styles.buttonText}>Reset</Text>
              </View>
            </TouchableHighlight>
          </View>

          {/* Recording Controls */}
          <View style={styles.dataSection}>
            <Text style={styles.sectionTitle}>Recording</Text>
            {!isRecording ? (
              <TouchableHighlight
                onPress={handleStartRecording}
                underlayColor="#e0e0e0"
                style={styles.buttonWrapper}
                disabled={isPlaying}
              >
                <View
                  style={[
                    styles.button,
                    {
                      backgroundColor: isPlaying
                        ? COLORS.DISABLED
                        : COLORS.ERROR,
                    },
                  ]}
                >
                  <Text style={styles.buttonText}>Start Recording</Text>
                </View>
              </TouchableHighlight>
            ) : (
              <View>
                <View style={styles.statusRow}>
                  <Text style={styles.statusLabel}>Recording:</Text>
                  <StatusIndicator color={COLORS.ERROR} text="ACTIVE" />
                </View>
                <TouchableHighlight
                  onPress={handleStopRecording}
                  underlayColor="#e0e0e0"
                  style={styles.buttonWrapper}
                >
                  <View
                    style={[styles.button, { backgroundColor: COLORS.ERROR }]}
                  >
                    <Text style={styles.buttonText}>Stop Recording</Text>
                  </View>
                </TouchableHighlight>
              </View>
            )}
          </View>

          {/* Playback Controls */}
          {isPlaying && (
            <View style={styles.dataSection}>
              <Text style={styles.sectionTitle}>Playback</Text>
              <View style={styles.statusRow}>
                <Text style={styles.statusLabel}>Playing:</Text>
                <StatusIndicator color={COLORS.SUCCESS} text="ACTIVE" />
              </View>
              <TouchableHighlight
                onPress={handleStopPlayback}
                underlayColor="#e0e0e0"
                style={styles.buttonWrapper}
              >
                <View
                  style={[styles.button, { backgroundColor: COLORS.SUCCESS }]}
                >
                  <Text style={styles.buttonText}>Stop Playback</Text>
                </View>
              </TouchableHighlight>
            </View>
          )}

          {/* File Explorer */}
          <View style={styles.dataSection}>
            <Text style={styles.sectionTitle}>Recordings</Text>
            <TouchableHighlight
              onPress={loadRecordingFiles}
              underlayColor="#e0e0e0"
              style={styles.buttonWrapper}
            >
              <View
                style={[
                  styles.button,
                  { backgroundColor: COLORS.PRIMARY, padding: 10 },
                ]}
              >
                <Text style={styles.buttonText}>Refresh</Text>
              </View>
            </TouchableHighlight>
            {recordingFiles.length === 0 ? (
              <Text style={styles.dataLabel}>No recordings found</Text>
            ) : (
              recordingFiles.map((file) => (
                <View key={file.filename} style={styles.fileRow}>
                  <View style={{ flex: 1 }}>
                    <Text style={styles.dataLabel}>{file.filename}</Text>
                    <Text
                      style={[
                        styles.dataValue,
                        { fontSize: 12, fontWeight: '400' },
                      ]}
                    >
                      {(file.size / 1024).toFixed(1)} KB •{' '}
                      {new Date(file.date).toLocaleString()}
                    </Text>
                  </View>
                  <View style={{ flexDirection: 'row', gap: 8 }}>
                    <TouchableHighlight
                      onPress={() => handlePlayback(file.filename)}
                      underlayColor="#e0e0e0"
                      disabled={isRecording || isPlaying}
                      style={{ borderRadius: 6 }}
                    >
                      <View
                        style={[
                          styles.smallButton,
                          {
                            backgroundColor:
                              isRecording || isPlaying
                                ? COLORS.DISABLED
                                : COLORS.SUCCESS,
                          },
                        ]}
                      >
                        <Text style={styles.smallButtonText}>Play</Text>
                      </View>
                    </TouchableHighlight>
                    <TouchableHighlight
                      onPress={() => handleDeleteRecording(file.filename)}
                      underlayColor="#e0e0e0"
                      disabled={isRecording || isPlaying}
                      style={{ borderRadius: 6 }}
                    >
                      <View
                        style={[
                          styles.smallButton,
                          {
                            backgroundColor:
                              isRecording || isPlaying
                                ? COLORS.DISABLED
                                : COLORS.ERROR,
                          },
                        ]}
                      >
                        <Text style={styles.smallButtonText}>Delete</Text>
                      </View>
                    </TouchableHighlight>
                  </View>
                </View>
              ))
            )}
          </View>

          <View style={styles.buttonRow}>
            <TouchableHighlight
              onPress={() => setRate(getNextRate(rate, 'decrease'))}
              underlayColor="#e0e0e0"
              accessibilityLabel="Decrease rate"
              accessibilityRole="button"
              style={styles.buttonWrapper}
              disabled={rate <= (RATE_VALUES[0] as number)}
            >
              <View
                style={[
                  styles.button,
                  styles.rateButton,
                  {
                    backgroundColor:
                      rate <= (RATE_VALUES[0] as number)
                        ? COLORS.DISABLED
                        : COLORS.SUCCESS,
                  },
                ]}
              >
                <Text style={styles.buttonText}>−</Text>
              </View>
            </TouchableHighlight>
            <View style={styles.rateDisplay}>
              <Text style={styles.rateText}>{rate} Hz</Text>
            </View>
            <TouchableHighlight
              onPress={() => setRate(getNextRate(rate, 'increase'))}
              underlayColor="#e0e0e0"
              accessibilityLabel="Increase rate"
              accessibilityRole="button"
              style={styles.buttonWrapper}
              disabled={rate >= (RATE_VALUES[RATE_VALUES.length - 1] as number)}
            >
              <View
                style={[
                  styles.button,
                  styles.rateButton,
                  {
                    backgroundColor:
                      rate >= (RATE_VALUES[RATE_VALUES.length - 1] as number)
                        ? COLORS.DISABLED
                        : COLORS.SUCCESS,
                  },
                ]}
              >
                <Text style={styles.buttonText}>+</Text>
              </View>
            </TouchableHighlight>
          </View>
        </View>
        </ScrollView>
      </KeyboardAvoidingView>
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    alignItems: 'center',
    justifyContent: 'center',
    backgroundColor: '#f5f5f5',
    padding: 10,
  },
  controlsContainer: {
    flex: 1,
    width: '100%',
    maxWidth: 400,
  },
  controlsContent: {
    padding: 12,
    paddingBottom: 20,
  },
  statusSection: {
    backgroundColor: 'white',
    borderRadius: 8,
    padding: 12,
    marginBottom: 12,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 1 },
    shadowOpacity: 0.1,
    shadowRadius: 2,
    elevation: 2,
  },
  dataSection: {
    backgroundColor: 'white',
    borderRadius: 8,
    padding: 12,
    marginBottom: 12,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 1 },
    shadowOpacity: 0.1,
    shadowRadius: 2,
    elevation: 2,
  },
  controlsSection: {
    marginTop: 8,
  },
  sectionTitle: {
    fontSize: 16,
    fontWeight: '600',
    color: '#333',
    marginBottom: 8,
    borderBottomWidth: 1,
    borderBottomColor: '#e0e0e0',
    paddingBottom: 4,
  },
  statusRow: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    paddingVertical: 4,
  },
  dataRow: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    paddingVertical: 6,
  },
  statusLabel: {
    fontSize: 14,
    color: '#666',
    fontWeight: '500',
  },
  statusValue: {
    fontSize: 14,
    color: '#333',
    fontWeight: '600',
  },
  dataLabel: {
    fontSize: 14,
    color: '#666',
  },
  dataValue: {
    fontSize: 14,
    color: '#333',
    fontWeight: '600',
  },
  buttonWrapper: {
    marginVertical: 4,
  },
  button: {
    padding: 14,
    borderRadius: 8,
    alignItems: 'center',
    backgroundColor: '#2196f3',
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.15,
    shadowRadius: 3,
    elevation: 3,
  },
  buttonText: {
    color: 'white',
    fontSize: 16,
    fontWeight: '600',
  },
  buttonRow: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginVertical: 4,
    gap: 8,
  },
  rateButton: {
    flex: 1,
    maxWidth: 60,
    backgroundColor: COLORS.SUCCESS,
  },
  rateDisplay: {
    flex: 1,
    alignItems: 'center',
    padding: 12,
    backgroundColor: 'white',
    borderRadius: 8,
    borderWidth: 1,
    borderColor: '#e0e0e0',
  },
  rateText: {
    fontSize: 16,
    fontWeight: '600',
    color: '#333',
  },
  sliderContainer: {
    marginVertical: 12,
    paddingVertical: 8,
  },
  sliderLabelRow: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    marginBottom: 4,
  },
  sliderLabel: {
    fontSize: 12,
    color: '#666',
  },
  slider: {
    width: '100%',
    height: 40,
  },
  fileRow: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    paddingVertical: 8,
    borderBottomWidth: 1,
    borderBottomColor: '#e0e0e0',
  },
  smallButton: {
    padding: 8,
    borderRadius: 6,
    alignItems: 'center',
    minWidth: 60,
  },
  smallButtonText: {
    color: 'white',
    fontSize: 12,
    fontWeight: '600',
  },
  inputRow: {
    marginVertical: 8,
  },
  textInput: {
    borderWidth: 1,
    borderColor: '#e0e0e0',
    borderRadius: 8,
    padding: 12,
    fontSize: 14,
    backgroundColor: 'white',
    color: '#333',
  },
  statusIndicatorContainer: {
    flexDirection: 'row',
    alignItems: 'center',
  },
  statusIndicatorDot: {
    width: STATUS_INDICATOR_SIZE,
    height: STATUS_INDICATOR_SIZE,
    borderRadius: STATUS_INDICATOR_RADIUS,
    marginRight: 8,
  },
});
