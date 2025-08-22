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
} from '@shopify/react-native-skia';

// Color constants
const COLORS = {
  SKY: '#4A90E2',
  GROUND: '#A0522D',
  TEXT: 'white',
  STROKE: 'white',
  HEADING_BUG: 'red',
};

// Dimension constants
const DIMENSIONS = {
  PITCH_SCALE: 4,
  MAJOR_PITCH_LINE_LENGTH: 40,
  MINOR_PITCH_LINE_LENGTH: 20,
  STROKE_WIDTH: 2,
  COMPASS_RADIUS_FACTOR: 0.95,
};

interface Props {
  attitude: { roll: number; pitch: number; heading: number };
}

export default function OrientationVisualizer(props: Props) {
  const { width, height } = useWindowDimensions();
  const { roll, pitch, heading } = props.attitude;
  const PFD_SIZE = Math.min(width, height);
  const CENTER = PFD_SIZE / 2;
  const rollClamped = Math.max(-180, Math.min(180, roll));
  const pitchClamped = Math.max(-90, Math.min(90, pitch));
  const headingClamped = ((heading % 360) + 360) % 360;
  const pitchOffset = pitchClamped * DIMENSIONS.PITCH_SCALE;
  const font = useFont(require('./assets/Roboto-Regular.ttf'), 14);
  const font2 = useFont(require('./assets/Roboto-Regular.ttf'), 24);

  function formatHeading(heading: number): string {
    const h = Math.round(heading);
    return h.toString().padStart(3, '0') + '°';
  }

  function renderText(
    x: number,
    y: number,
    text: string,
    fontToUse: any,
    color = COLORS.TEXT
  ) {
    if (!fontToUse) return null;
    return <Text x={x} y={y} text={text} font={fontToUse} color={color} />;
  }

  function renderPitchLines() {
    const lines = [];
    for (let p = -90; p <= 90; p += 10) {
      if (p === 0) continue;
      const y = CENTER + pitchOffset - p * DIMENSIONS.PITCH_SCALE;
      if (y < 0 || y > PFD_SIZE) continue;

      const length =
        p % 30 === 0
          ? DIMENSIONS.MAJOR_PITCH_LINE_LENGTH
          : DIMENSIONS.MINOR_PITCH_LINE_LENGTH;

      lines.push(
        <Group key={`pitch${p}`}>
          {/* Left line */}
          <Line
            p1={{ x: CENTER - length, y }}
            p2={{ x: CENTER, y }}
            color={COLORS.STROKE}
            strokeWidth={DIMENSIONS.STROKE_WIDTH}
          />
          {/* Right line */}
          <Line
            p1={{ x: CENTER, y }}
            p2={{ x: CENTER + length, y }}
            color={COLORS.STROKE}
            strokeWidth={DIMENSIONS.STROKE_WIDTH}
          />
          {/* Text labels */}
          {font && (
            <>
              {renderText(CENTER - length - 25, y + 5, `${p}`, font)}
              {renderText(CENTER + length + 5, y + 5, `${p}`, font)}
            </>
          )}
        </Group>
      );
    }
    return lines;
  }

  function headingMarks() {
    const marks = [];
    for (let h = 0; h < 360; h += 30) {
      const angleRad = ((h - headingClamped) * Math.PI) / 180;
      const radiusOuter = CENTER * DIMENSIONS.COMPASS_RADIUS_FACTOR;
      const radiusInner = radiusOuter - (h % 90 === 0 ? 15 : 7);
      const angleRadSin = Math.sin(angleRad);
      const angleRadCos = Math.cos(angleRad);

      const x1 = CENTER + radiusInner * angleRadSin;
      const y1 = CENTER - radiusInner * angleRadCos;
      const x2 = CENTER + radiusOuter * angleRadSin;
      const y2 = CENTER - radiusOuter * angleRadCos;

      const textRadius = radiusInner - 20;
      const tx = CENTER + textRadius * angleRadSin;
      const ty = CENTER - textRadius * angleRadCos;

      let label = h.toString();
      if (h === 0) label = 'N';
      else if (h === 90) label = 'E';
      else if (h === 180) label = 'S';
      else if (h === 270) label = 'W';

      marks.push(
        <Group key={`headMark${h}`}>
          <Line
            p1={{ x: x1, y: y1 }}
            p2={{ x: x2, y: y2 }}
            color={COLORS.STROKE}
            strokeWidth={DIMENSIONS.STROKE_WIDTH}
          />
          {font && renderText(tx, ty + 5, label, font)}
        </Group>
      );
    }
    return marks;
  }

  return (
    <View style={[styles.container, { backgroundColor: COLORS.SKY }]}>
      <Canvas style={{ width: PFD_SIZE, height: PFD_SIZE }}>
        {/* Artificial Horizon Group: rotate roll */}
        <Group
          origin={{ x: CENTER, y: CENTER }}
          transform={[{ rotate: (-rollClamped * Math.PI) / 180 }]}
        >
          {/* Ground */}
          <Rect
            x={0 - PFD_SIZE / 2}
            y={CENTER + pitchOffset}
            width={PFD_SIZE * 2}
            height={PFD_SIZE * 2}
            color={COLORS.GROUND}
          />
          {/* Horizon line */}
          <Line
            p1={{ x: 0 - PFD_SIZE / 2, y: CENTER + pitchOffset }}
            p2={{ x: PFD_SIZE + PFD_SIZE / 2, y: CENTER + pitchOffset }}
            color={COLORS.STROKE}
            strokeWidth={3}
          />
          {/* Pitch ladder */}
          {renderPitchLines()}
        </Group>

        {/* Aircraft symbol (fixed center) */}
        {/* Wings left */}
        <Line
          p1={{ x: CENTER - 30, y: CENTER }}
          p2={{ x: CENTER - 5, y: CENTER }}
          color={COLORS.STROKE}
          strokeWidth={DIMENSIONS.STROKE_WIDTH}
        />
        {/* Wings right */}
        <Line
          p1={{ x: CENTER + 5, y: CENTER }}
          p2={{ x: CENTER + 30, y: CENTER }}
          color={COLORS.STROKE}
          strokeWidth={DIMENSIONS.STROKE_WIDTH}
        />
        {/* Vertical line down */}
        <Line
          p1={{ x: CENTER, y: CENTER }}
          p2={{ x: CENTER, y: CENTER + 20 }}
          color={COLORS.STROKE}
          strokeWidth={DIMENSIONS.STROKE_WIDTH}
        />

        {/* Heading compass */}
        <Circle
          cx={CENTER}
          cy={CENTER}
          r={CENTER * DIMENSIONS.COMPASS_RADIUS_FACTOR}
          color={COLORS.STROKE}
          style="stroke"
          strokeWidth={DIMENSIONS.STROKE_WIDTH}
        />

        {/* Heading marks */}
        {headingMarks()}

        {/* Heading bug - a red small triangle pointing upwards */}
        <Path
          path={`
            M${CENTER},${CENTER - CENTER * DIMENSIONS.COMPASS_RADIUS_FACTOR}
            L${CENTER - 7},${CENTER - CENTER * 0.85}
            L${CENTER + 7},${CENTER - CENTER * 0.85}
            Z
          `}
          color={COLORS.HEADING_BUG}
        />

        {/* Heading numeric display box */}
        <>
          <Rect
            x={CENTER - 30}
            y={CENTER + CENTER * 0.75}
            width={60}
            height={30}
            color="black"
            style="fill"
            strokeWidth={DIMENSIONS.STROKE_WIDTH}
          />
          <Rect
            x={CENTER - 30}
            y={CENTER + CENTER * 0.75}
            width={60}
            height={30}
            color={COLORS.STROKE}
            style="stroke"
            strokeWidth={DIMENSIONS.STROKE_WIDTH}
          />
          {renderText(
            CENTER - CENTER * 0.12,
            CENTER + CENTER * 0.875,
            formatHeading(headingClamped),
            font2
          )}
        </>
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
