/**
 * OrientationVisualizer.tsx
 *
 * A React Native component that renders a Primary Flight Display (PFD) showing
 * aircraft attitude, heading, and flight path information. Uses Skia for high-performance
 * 2D graphics rendering.
 *
 * Features:
 * - Artificial horizon with pitch ladder
 * - Roll indicator with compass rose
 * - Heading display with bug indicator
 * - Flight Path Vector (FPV) symbol
 * - Aircraft symbol overlay
 *
 * @module OrientationVisualizer
 */

import React, { useMemo } from 'react';
import { View, StyleSheet, useWindowDimensions } from 'react-native';
import {
  Canvas,
  Circle,
  Group,
  Line,
  Path,
  Rect,
  Text,
  useFont,
  type SkFont,
} from '@shopify/react-native-skia';

// Color constants
const COLORS = {
  SKY: '#4A90E2',
  GROUND: '#A0522D',
  TEXT: 'white',
  STROKE: 'white',
  HEADING_BUG: 'red',
  WARNING: 'red',
  FPV: 'lightgreen', // Flight path vector color (body frame method)
} as const;

// Dimension constants
const DIMENSIONS = {
  PITCH_SCALE: 10,
  HORIZONTAL_FPA_SCALE: 6,
  BOLD_MAJOR_PITCH_LINE_LENGTH: 50,
  MAJOR_PITCH_LINE_LENGTH: 30,
  MINOR_PITCH_LINE_LENGTH: 15,
  STROKE_WIDTH: 2,
  COMPASS_RADIUS_FACTOR: 0.95,
  HEADING_BUG_BASE_FACTOR: 0.85,
  HEADING_BUG_TOP_FACTOR: 0.95,
  HEADING_BUG_WIDTH: 7,
  HEADING_DISPLAY_Y_OFFSET: 0.6,
  HEADING_DISPLAY_HEIGHT: 30,
  HEADING_DISPLAY_WIDTH: 60,
  HEADING_TEXT_X_OFFSET: 0.12,
  HEADING_TEXT_Y_OFFSET: 0.72,
  AIRCRAFT_WING_LENGTH: 30,
  AIRCRAFT_WING_GAP: 5,
  AIRCRAFT_NOSE_LENGTH: 20,
  COMPASS_MAJOR_MARK_OFFSET: 15,
  COMPASS_MINOR_MARK_OFFSET: 7,
  COMPASS_TEXT_OFFSET: 20,
  HORIZON_LINE_WIDTH: 3,
  FPV_SYMBOL_SIZE: 20, // Size of FPV symbol (diamond/circle)
  FPV_SYMBOL_STROKE_WIDTH: 2,
  FPV_SYMBOL_SIZE_FACTOR: 0.75, // Factor to make FPV circle slightly smaller
  FPV_WING_EXTENSION: 10, // Extension length for FPV wing lines
  FPV_RUDDER_EXTENSION: 5, // Extension length for FPV rudder line
} as const;

// Font sizes
const FONT_SIZES = {
  PITCH_LABEL: 14,
  HEADING_DISPLAY: 24,
} as const;

// Pitch line intervals
const PITCH_INTERVALS = {
  MINOR: 2.5, // Every 2.5 degrees
  MAJOR: 5, // Every 5 degrees
  BOLD_MAJOR: 10, // Every 10 degrees
} as const;

// Heading mark intervals
const HEADING_INTERVALS = {
  MARK: 30, // Every 30 degrees
} as const;

// Value limits
const LIMITS = {
  ROLL_MIN: -180,
  ROLL_MAX: 180,
  PITCH_MIN: -90,
  PITCH_MAX: 90,
  HEADING_MIN: 0,
  HEADING_MAX: 360,
} as const;

/**
 * Aircraft attitude data interface
 */
interface Attitude {
  /** Roll angle in degrees (-180 to 180) */
  roll: number;
  /** Pitch angle in degrees (-90 to 90) */
  pitch: number;
  /** Heading angle in degrees (0 to 360) */
  heading: number;
  /** Flight path angle in degrees (vertical) */
  flightPathAngle: number;
  /** Horizontal flight path angle in degrees (sideslip/crab angle) */
  horizontalFlightPathAngle: number;
}

/**
 * Component props
 */
interface Props {
  /** Current aircraft attitude data */
  attitude: Attitude;
}

/**
 * Clamps a value between min and max
 * @param value - Value to clamp
 * @param min - Minimum value
 * @param max - Maximum value
 * @returns Clamped value
 */
function clamp(value: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, value));
}

/**
 * Normalizes heading angle to [0, 360) range
 * @param heading - Heading angle in degrees
 * @returns Normalized heading (0-360)
 */
function normalizeHeading(heading: number): number {
  return (
    ((heading % LIMITS.HEADING_MAX) + LIMITS.HEADING_MAX) % LIMITS.HEADING_MAX
  );
}

/**
 * Formats heading for display (e.g., "045°")
 * @param heading - Heading angle in degrees
 * @returns Formatted string with leading zeros
 */
function formatHeading(heading: number): string {
  const h = Math.round(heading);
  return h.toString().padStart(3, '0') + '°';
}

/**
 * Calculates the visible pitch range based on current pitch and viewport size.
 * Only pitch lines within this range are rendered for performance.
 * @param pitchOffset - Current pitch offset in pixels
 * @param pfdSize - Size of the PFD display in pixels
 * @returns Object with min and max pitch angles in degrees
 */
function getVisiblePitchRange(
  pitchOffset: number,
  pfdSize: number
): { min: number; max: number } {
  // Validate inputs to prevent NaN/invalid calculations
  if (!Number.isFinite(pitchOffset) || !Number.isFinite(pfdSize) || pfdSize <= 0) {
    // Return full range if inputs are invalid
    return {
      min: LIMITS.PITCH_MIN,
      max: LIMITS.PITCH_MAX,
    };
  }
  
  // Calculate how many degrees are visible above and below center
  const visibleDegrees = pfdSize / (2 * DIMENSIONS.PITCH_SCALE);
  // Round to 2.5-degree increments
  const minPitch =
    Math.floor(
      (pitchOffset / DIMENSIONS.PITCH_SCALE - visibleDegrees) /
        PITCH_INTERVALS.MINOR
    ) * PITCH_INTERVALS.MINOR;
  const maxPitch =
    Math.ceil(
      (pitchOffset / DIMENSIONS.PITCH_SCALE + visibleDegrees) /
        PITCH_INTERVALS.MINOR
    ) * PITCH_INTERVALS.MINOR;

  // Ensure valid range is returned
  const min = Number.isFinite(minPitch) ? Math.max(LIMITS.PITCH_MIN, minPitch) : LIMITS.PITCH_MIN;
  const max = Number.isFinite(maxPitch) ? Math.min(LIMITS.PITCH_MAX, maxPitch) : LIMITS.PITCH_MAX;

  return {
    min,
    max,
  };
}

/**
 * Renders a single pitch ladder line with optional label.
 * Lines are categorized as bold major (10°), major (5°), or minor (2.5°).
 * @param pitch - Pitch angle in degrees
 * @param center - Center X coordinate of the display
 * @param pitchOffset - Current pitch offset in pixels
 * @param font - Font for rendering pitch labels
 */
function PitchLine({
  pitch,
  center,
  pitchOffset,
  font,
}: {
  pitch: number;
  center: number;
  pitchOffset: number;
  font: SkFont | null;
}) {
  const y = center + pitchOffset - pitch * DIMENSIONS.PITCH_SCALE;

  // Skip if outside visible area (with some margin)
  if (y < -50 || y > center * 2 + 50) {
    return null;
  }

  // Determine three levels: bold major (10°), major (5°), minor (2.5°)
  const isBoldMajor = Math.abs(pitch % PITCH_INTERVALS.BOLD_MAJOR) < 0.01; // Account for floating point precision
  const isMajor =
    Math.abs(pitch % PITCH_INTERVALS.MAJOR) < 0.01 && !isBoldMajor;

  // Set line length and stroke width based on level
  let length: number;
  let strokeWidth: number;
  if (isBoldMajor) {
    length = DIMENSIONS.BOLD_MAJOR_PITCH_LINE_LENGTH;
    strokeWidth = DIMENSIONS.STROKE_WIDTH;
  } else if (isMajor) {
    length = DIMENSIONS.MAJOR_PITCH_LINE_LENGTH;
    strokeWidth = DIMENSIONS.STROKE_WIDTH;
  } else {
    length = DIMENSIONS.MINOR_PITCH_LINE_LENGTH;
    strokeWidth = DIMENSIONS.STROKE_WIDTH;
  }

  return (
    <Group key={`pitch${pitch}`}>
      {/* Left line */}
      <Line
        p1={{ x: center - length, y }}
        p2={{ x: center, y }}
        color={COLORS.STROKE}
        strokeWidth={strokeWidth}
      />
      {/* Right line */}
      <Line
        p1={{ x: center, y }}
        p2={{ x: center + length, y }}
        color={COLORS.STROKE}
        strokeWidth={strokeWidth}
      />
      {/* Text labels - only show on bold major bars (10-degree increments) */}
      {font && isBoldMajor && (
        <>
          <Text
            x={center - length - 25}
            y={y + 5}
            text={`${pitch}`}
            font={font}
            color={COLORS.TEXT}
          />
          <Text
            x={center + length + 5}
            y={y + 5}
            text={`${pitch}`}
            font={font}
            color={COLORS.TEXT}
          />
        </>
      )}
    </Group>
  );
}

/**
 * Renders the fixed aircraft symbol (wings and nose) at the center of the display.
 * This symbol does not rotate with roll - it represents the aircraft reference frame.
 * @param center - Center coordinate of the display
 */
function AircraftSymbol({ center }: { center: number }) {
  return (
    <Group>
      {/* Wings left */}
      <Line
        p1={{ x: center - DIMENSIONS.AIRCRAFT_WING_LENGTH, y: center }}
        p2={{ x: center - DIMENSIONS.AIRCRAFT_WING_GAP, y: center }}
        color={COLORS.STROKE}
        strokeWidth={DIMENSIONS.STROKE_WIDTH}
      />
      {/* Wings right */}
      <Line
        p1={{ x: center + DIMENSIONS.AIRCRAFT_WING_GAP, y: center }}
        p2={{ x: center + DIMENSIONS.AIRCRAFT_WING_LENGTH, y: center }}
        color={COLORS.STROKE}
        strokeWidth={DIMENSIONS.STROKE_WIDTH}
      />
      {/* Vertical line down (nose) */}
      <Line
        p1={{ x: center, y: center }}
        p2={{ x: center, y: center + DIMENSIONS.AIRCRAFT_NOSE_LENGTH }}
        color={COLORS.STROKE}
        strokeWidth={DIMENSIONS.STROKE_WIDTH}
      />
    </Group>
  );
}

/**
 * Renders the Flight Path Vector (FPV) symbol showing where the aircraft is actually going.
 * The FPV is positioned based on flight path angles and is NOT rotated with roll.
 * @param x - X coordinate for FPV symbol center
 * @param y - Y coordinate for FPV symbol center
 */
function FPVSymbol({ x, y }: { x: number; y: number }) {
  // Calculate FPV symbol radius using extracted constant
  const fpvRadius = (DIMENSIONS.FPV_SYMBOL_SIZE / 2) * DIMENSIONS.FPV_SYMBOL_SIZE_FACTOR;

  return (
    <Group>
      {/* FPV circle symbol - slightly smaller */}
      <Circle
        cx={x}
        cy={y}
        r={fpvRadius}
        color={COLORS.FPV}
        style="stroke"
        strokeWidth={DIMENSIONS.FPV_SYMBOL_STROKE_WIDTH}
      />
      {/* Wings - horizontal lines on left and right (longer) */}
      <Line
        p1={{
          x: x - fpvRadius - DIMENSIONS.FPV_WING_EXTENSION,
          y: y,
        }}
        p2={{
          x: x - fpvRadius,
          y: y,
        }}
        color={COLORS.FPV}
        strokeWidth={DIMENSIONS.FPV_SYMBOL_STROKE_WIDTH}
      />
      <Line
        p1={{
          x: x + fpvRadius,
          y: y,
        }}
        p2={{
          x: x + fpvRadius + DIMENSIONS.FPV_WING_EXTENSION,
          y: y,
        }}
        color={COLORS.FPV}
        strokeWidth={DIMENSIONS.FPV_SYMBOL_STROKE_WIDTH}
      />
      {/* Rudder - vertical line sticking up */}
      <Line
        p1={{
          x: x,
          y: y - fpvRadius,
        }}
        p2={{
          x: x,
          y: y - fpvRadius - DIMENSIONS.FPV_RUDDER_EXTENSION,
        }}
        color={COLORS.FPV}
        strokeWidth={DIMENSIONS.FPV_SYMBOL_STROKE_WIDTH}
      />
    </Group>
  );
}

/**
 * Renders the heading display box with formatted heading value.
 * @param center - Center coordinate of the display
 * @param heading - Current heading in degrees
 * @param font - Font for rendering heading text
 */
function HeadingDisplay({
  center,
  heading,
  font,
}: {
  center: number;
  heading: number;
  font: SkFont | null;
}) {
  const displayY = center + center * DIMENSIONS.HEADING_DISPLAY_Y_OFFSET;
  const textX = center - center * DIMENSIONS.HEADING_TEXT_X_OFFSET;
  const textY = center + center * DIMENSIONS.HEADING_TEXT_Y_OFFSET;
  const displayX = center - DIMENSIONS.HEADING_DISPLAY_WIDTH / 2;

  return (
    <Group>
      {/* Background with border - using a single Rect with both fill and stroke */}
      <Rect
        x={displayX}
        y={displayY}
        width={DIMENSIONS.HEADING_DISPLAY_WIDTH}
        height={DIMENSIONS.HEADING_DISPLAY_HEIGHT}
        color="black"
        style="fill"
      />
      <Rect
        x={displayX}
        y={displayY}
        width={DIMENSIONS.HEADING_DISPLAY_WIDTH}
        height={DIMENSIONS.HEADING_DISPLAY_HEIGHT}
        color={COLORS.STROKE}
        style="stroke"
        strokeWidth={DIMENSIONS.STROKE_WIDTH}
      />
      {font && (
        <Text
          x={textX}
          y={textY}
          text={formatHeading(heading)}
          font={font}
          color={COLORS.TEXT}
        />
      )}
    </Group>
  );
}

/**
 * Renders a single heading mark on the compass rose.
 * Major marks (N, E, S, W) are longer and labeled with letters.
 * @param heading - Heading angle in degrees
 * @param currentHeading - Current aircraft heading
 * @param center - Center coordinate of the display
 * @param font - Font for rendering heading labels
 */
function HeadingMark({
  heading,
  currentHeading,
  center,
  font,
}: {
  heading: number;
  currentHeading: number;
  center: number;
  font: SkFont | null;
}) {
  const angleRad = ((heading - currentHeading) * Math.PI) / 180;
  const radiusOuter = center * DIMENSIONS.COMPASS_RADIUS_FACTOR;
  const isMajor = heading % 90 === 0;
  const radiusInner =
    radiusOuter -
    (isMajor
      ? DIMENSIONS.COMPASS_MAJOR_MARK_OFFSET
      : DIMENSIONS.COMPASS_MINOR_MARK_OFFSET);

  const sin = Math.sin(angleRad);
  const cos = Math.cos(angleRad);

  const x1 = center + radiusInner * sin;
  const y1 = center - radiusInner * cos;
  const x2 = center + radiusOuter * sin;
  const y2 = center - radiusOuter * cos;

  const textRadius = radiusInner - DIMENSIONS.COMPASS_TEXT_OFFSET;
  const tx = center + textRadius * sin;
  const ty = center - textRadius * cos;

  // Determine label
  let label: string;
  switch (heading) {
    case 0:
      label = 'N';
      break;
    case 90:
      label = 'E';
      break;
    case 180:
      label = 'S';
      break;
    case 270:
      label = 'W';
      break;
    default:
      label = heading.toString();
  }

  return (
    <Group key={`headMark${heading}`}>
      <Line
        p1={{ x: x1, y: y1 }}
        p2={{ x: x2, y: y2 }}
        color={COLORS.STROKE}
        strokeWidth={DIMENSIONS.STROKE_WIDTH}
      />
      {font && (
        <Text x={tx} y={ty + 5} text={label} font={font} color={COLORS.TEXT} />
      )}
    </Group>
  );
}

/**
 * Main OrientationVisualizer component.
 * Renders a complete Primary Flight Display with artificial horizon, pitch ladder,
 * compass rose, heading display, and flight path vector.
 *
 * @param props - Component props containing attitude data
 * @returns React component rendering the PFD
 */
function OrientationVisualizer({ attitude }: Props) {
  const { width, height } = useWindowDimensions();

  // Load fonts once
  const pitchFont = useFont(
    require('./assets/Roboto-Regular.ttf'),
    FONT_SIZES.PITCH_LABEL
  );
  const headingFont = useFont(
    require('./assets/Roboto-Regular.ttf'),
    FONT_SIZES.HEADING_DISPLAY
  );

  // Memoize calculations for performance - only recalculate when dimensions or attitude change
  const { pfdSize, center, clampedValues, pitchOffset, rollRad } =
    useMemo(() => {
      // Use smallest dimension to ensure square display
      const size = Math.min(width, height);
      const c = size / 2;

      // Clamp values to valid ranges to prevent rendering issues
      // Handle NaN/undefined/null values by defaulting to 0
      const rollValue = Number.isFinite(attitude.roll) ? attitude.roll : 0;
      const pitchValue = Number.isFinite(attitude.pitch) ? attitude.pitch : 0;
      const headingValue = Number.isFinite(attitude.heading) ? attitude.heading : 0;
      
      const rollClamped = clamp(
        rollValue,
        LIMITS.ROLL_MIN,
        LIMITS.ROLL_MAX
      );
      const pitchClamped = clamp(
        pitchValue,
        LIMITS.PITCH_MIN,
        LIMITS.PITCH_MAX
      );
      const headingClamped = normalizeHeading(headingValue);

      // Convert pitch to pixel offset (positive pitch = up on display)
      // Ensure offset is always a valid number
      const offset = Number.isFinite(pitchClamped * DIMENSIONS.PITCH_SCALE)
        ? pitchClamped * DIMENSIONS.PITCH_SCALE
        : 0;

      // Convert roll to radians, negate for correct rotation direction
      const rollRadians = (-rollClamped * Math.PI) / 180;

      return {
        pfdSize: size,
        center: c,
        clampedValues: {
          roll: rollClamped,
          pitch: pitchClamped,
          heading: headingClamped,
        },
        pitchOffset: offset,
        rollRad: rollRadians,
      };
    }, [width, height, attitude.roll, attitude.pitch, attitude.heading]);

  // Memoize pitch lines - only render visible ones
  const pitchLines = useMemo(() => {
    if (!pitchFont) return null;

    const visibleRange = getVisiblePitchRange(pitchOffset, pfdSize);
    const lines: React.ReactElement[] = [];

    // Generate pitch lines in visible range (increment by 2.5 degrees)
    for (
      let p = visibleRange.min;
      p <= visibleRange.max;
      p += PITCH_INTERVALS.MINOR
    ) {
      if (Math.abs(p) < 0.01) continue; // Skip horizon line (handled separately)
      lines.push(
        <PitchLine
          key={`pitch${p}`}
          pitch={p}
          center={center}
          pitchOffset={pitchOffset}
          font={pitchFont}
        />
      );
    }

    return lines;
  }, [pitchOffset, center, pfdSize, pitchFont]);

  // Memoize heading marks
  const headingMarks = useMemo(() => {
    if (!pitchFont) return null;

    const marks: React.ReactElement[] = [];
    for (let h = 0; h < LIMITS.HEADING_MAX; h += HEADING_INTERVALS.MARK) {
      marks.push(
        <HeadingMark
          key={`headMark${h}`}
          heading={h}
          currentHeading={clampedValues.heading}
          center={center}
          font={pitchFont}
        />
      );
    }
    return marks;
  }, [clampedValues.heading, center, pitchFont]);

  // Calculate heading bug path (simple string template, no memoization needed)
  const topY = center - center * DIMENSIONS.COMPASS_RADIUS_FACTOR;
  const baseY = center - center * DIMENSIONS.HEADING_BUG_BASE_FACTOR;
  const headingBugPath = `M${center},${topY} L${center - DIMENSIONS.HEADING_BUG_WIDTH},${baseY} L${center + DIMENSIONS.HEADING_BUG_WIDTH},${baseY} Z`;

  /**
   * Calculate Flight Path Vector (FPV) position.
   * FPV shows where the aircraft is actually going, not where it's pointing.
   * Vertical FPA determines vertical offset (pitch direction).
   * Horizontal FPA (sideslip/crab angle) determines horizontal offset (roll direction).
   * Note: FPV is NOT rotated with roll - it shows actual trajectory in space.
   */
  const fpvPositions = useMemo(() => {
    const fpa_deg = attitude.flightPathAngle || 0;
    const horizontal_fpa_deg = attitude.horizontalFlightPathAngle || 0;

    // Vertical offset: FPA (positive = climbing = up on display)
    // Use same scale as pitch ladder (PITCH_SCALE)
    // Negate because positive pitch should move up on screen
    const fpv_y_offset = -fpa_deg * DIMENSIONS.PITCH_SCALE;

    // Horizontal offset: Body frame velocity (sideslip/crab angle)
    // Use separate horizontal scale (different sensitivity than vertical)
    const fpv_x_offset = horizontal_fpa_deg * DIMENSIONS.HORIZONTAL_FPA_SCALE;

    return {
      x: center + fpv_x_offset,
      y: center + pitchOffset + fpv_y_offset,
    };
  }, [
    center,
    pitchOffset,
    attitude.flightPathAngle,
    attitude.horizontalFlightPathAngle,
  ]);

  return (
    <View style={[styles.container, { backgroundColor: COLORS.SKY }]}>
      <Canvas style={{ width: pfdSize, height: pfdSize }}>
        {/* Artificial Horizon Group: rotate roll */}
        <Group
          origin={{ x: center, y: center }}
          transform={[{ rotate: rollRad }]}
        >
          {/* Ground - extends from horizon line all the way down to cover full pitch range (-90° to 0°) */}
          {/* Always render ground/horizon with safe fallback if pitchOffset is invalid */}
          <Rect
            x={0 - pfdSize / 2}
            y={Number.isFinite(center + pitchOffset) ? center + pitchOffset : center}
            width={pfdSize * 2}
            height={pfdSize * 3}
            color={COLORS.GROUND}
          />
          {/* Horizon line */}
          <Line
            p1={{ x: 0 - pfdSize / 2, y: Number.isFinite(center + pitchOffset) ? center + pitchOffset : center }}
            p2={{ x: pfdSize + pfdSize / 2, y: Number.isFinite(center + pitchOffset) ? center + pitchOffset : center }}
            color={COLORS.STROKE}
            strokeWidth={DIMENSIONS.HORIZON_LINE_WIDTH}
          />
          {/* Pitch ladder */}
          {pitchLines}
        </Group>

        {/* Aircraft symbol (fixed center) */}
        <AircraftSymbol center={center} />

        {/* Flight Path Vector symbol - shows where aircraft is actually going */}
        <FPVSymbol x={fpvPositions.x} y={fpvPositions.y} />

        {/* Heading compass circle */}
        <Circle
          cx={center}
          cy={center}
          r={center * DIMENSIONS.COMPASS_RADIUS_FACTOR}
          color={COLORS.STROKE}
          style="stroke"
          strokeWidth={DIMENSIONS.STROKE_WIDTH}
        />

        {/* Heading marks */}
        {headingMarks}

        {/* Heading bug - a red small triangle pointing upwards */}
        <Path path={headingBugPath} color={COLORS.HEADING_BUG} />

        {/* Heading numeric display box */}
        <HeadingDisplay
          center={center}
          heading={clampedValues.heading}
          font={headingFont}
        />
      </Canvas>
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    justifyContent: 'center',
    alignItems: 'center',
  },
});

// Memoize component to prevent unnecessary re-renders
export default React.memo(OrientationVisualizer);
