import { CodegenTypes, type TurboModule } from 'react-native';
import { TurboModuleRegistry } from 'react-native';

export type AhrsData = {
  roll: number; // in degrees, -left, +right, -180/+180
  pitch: number; // in degrees, -down, +up, -90/+90
  heading: number; // in degrees true, 0-360
};

export type Rotation = 'none' | 'left' | 'right';

export interface Spec extends TurboModule {
  startAhrs(): void;
  stopAhrs(): void;
  resetAhrs(): void;
  levelAhrs(): void;
  setAhrsRate(newRate: number): void;
  setAhrsGain(newGain: number): void;
  setAhrsRotation(newRotation: Rotation): void;
  readonly onAhrsUpdate: CodegenTypes.EventEmitter<AhrsData>;
}

export default TurboModuleRegistry.getEnforcing<Spec>('NativeAhrs');
