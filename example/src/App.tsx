import { useEffect, useState, useMemo } from 'react';
import { StyleSheet, Text, TouchableHighlight, View } from 'react-native';
import RNOrientationDirector, {
  Orientation,
  OrientationType,
  useInterfaceOrientation,
} from 'react-native-orientation-director';
import OrientationVisualizer from './OrientationVisualizer';
import { Ahrs, type AhrsData } from 'react-native-ahrs';

export default function App() {
  const interfaceOrientation = useInterfaceOrientation();
  const [attitude, setAttitude] = useState<AhrsData>({
    roll: 0,
    pitch: 0,
    heading: 0,
  });
  const [levelled, setLevelled] = useState<boolean>(false);
  const [gain, setGain] = useState<number>(3.0);
  const [rate, setRate] = useState<number>(20);
  const rotationString = useMemo(() => {
    switch (interfaceOrientation) {
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
  }, [interfaceOrientation]);
  const roundedAttitude = useMemo(
    () => ({
      roll: Math.round(attitude.roll),
      pitch: Math.round(attitude.pitch),
      heading: Math.round(attitude.heading),
    }),
    [attitude]
  );

  useEffect(() => {
    Ahrs.isSupported().then((supported) => {
      console.log('AHRS supported:', supported ? 'YES' : 'NO');
    });
    const remove = Ahrs.addListener((data) => setAttitude(data));
    Ahrs.start();
    Ahrs.setRate(20);
    RNOrientationDirector.lockTo(Orientation.portrait, OrientationType.device);
    return () => {
      remove?.();
      Ahrs.stop();
      RNOrientationDirector.unlock();
    };
  }, []);

  useEffect(() => {
    Ahrs.setRate(rate);
  }, [rate]);

  useEffect(() => {
    Ahrs.setGain(gain);
  }, [gain]);

  function handleLevelPress() {
    Ahrs.level();
    setLevelled(!levelled);
  }

  function handleRotatePress() {
    if (interfaceOrientation === Orientation.portrait) {
      Ahrs.setRotation('left');
      RNOrientationDirector.lockTo(
        Orientation.landscapeLeft,
        OrientationType.interface
      );
    } else if (interfaceOrientation === Orientation.landscapeLeft) {
      Ahrs.setRotation('right');
      RNOrientationDirector.lockTo(
        Orientation.landscapeRight,
        OrientationType.interface
      );
    } else {
      Ahrs.setRotation('none');
      RNOrientationDirector.lockTo(
        Orientation.portrait,
        OrientationType.interface
      );
    }
  }

  function handleGainDecrease() {
    const newGain = gain - 1;
    if (newGain > 0) {
      setGain(newGain);
    }
  }

  function handleGainIncrease() {
    const newGain = gain + 1;
    setGain(newGain);
  }

  function handleRateDecrease() {
    if (rate === 5) {
      setRate(1);
    } else if (rate === 10) {
      setRate(5);
    } else if (rate === 20) {
      setRate(10);
    } else if (rate === 40) {
      setRate(20);
    } else if (rate === 60) {
      setRate(40);
    }
  }

  function handleRateIncrease() {
    if (rate === 1) {
      setRate(5);
    } else if (rate === 5) {
      setRate(10);
    } else if (rate === 10) {
      setRate(20);
    } else if (rate === 20) {
      setRate(40);
    } else if (rate === 40) {
      setRate(60);
    }
  }

  return (
    <View
      style={[
        styles.container,
        {
          flexDirection:
            interfaceOrientation === Orientation.portrait ? 'column' : 'row',
        },
      ]}
    >
      <OrientationVisualizer attitude={attitude} />
      <View style={styles.container2}>
        <Text style={styles.textStyle}>Rotation: {rotationString}</Text>
        <Text style={styles.textStyle}>
          Pitch: {roundedAttitude.pitch} Roll: {roundedAttitude.roll}
        </Text>
        <Text style={styles.textStyle}>
          Gain: {gain} Rate: {rate}Hz
        </Text>
        <TouchableHighlight
          onPress={handleRotatePress}
          accessibilityLabel="Rotate orientation"
          accessibilityRole="button"
        >
          <View style={styles.button}>
            <Text style={styles.buttonText}>Rotate</Text>
          </View>
        </TouchableHighlight>
        <View style={styles.container3}>
          <TouchableHighlight
            onPress={handleLevelPress}
            underlayColor="white"
            accessibilityLabel="Level the AHRS system"
            accessibilityRole="button"
          >
            <View
              style={[
                styles.button,
                { backgroundColor: levelled ? 'green' : 'red' },
              ]}
            >
              <Text style={styles.buttonText}>
                {levelled ? 'Levelled' : 'Level'}
              </Text>
            </View>
          </TouchableHighlight>
          <TouchableHighlight
            onPress={Ahrs.reset}
            accessibilityLabel="Reset AHRS"
            accessibilityRole="button"
          >
            <View style={styles.button}>
              <Text style={styles.buttonText}>Reset</Text>
            </View>
          </TouchableHighlight>
        </View>
        <View style={styles.container3}>
          <TouchableHighlight
            onPress={handleGainIncrease}
            accessibilityLabel="Increase gain"
            accessibilityRole="button"
          >
            <View style={styles.button}>
              <Text style={styles.buttonText}>Gain +</Text>
            </View>
          </TouchableHighlight>
          <TouchableHighlight
            onPress={handleRateIncrease}
            accessibilityLabel="Increase rate"
            accessibilityRole="button"
          >
            <View style={styles.button}>
              <Text style={styles.buttonText}>Rate +</Text>
            </View>
          </TouchableHighlight>
        </View>
        <View style={styles.container3}>
          <TouchableHighlight
            onPress={handleGainDecrease}
            accessibilityLabel="Decrease gain"
            accessibilityRole="button"
          >
            <View style={styles.button}>
              <Text style={styles.buttonText}>Gain -</Text>
            </View>
          </TouchableHighlight>
          <TouchableHighlight
            onPress={handleRateDecrease}
            accessibilityLabel="Decrease rate"
            accessibilityRole="button"
          >
            <View style={styles.button}>
              <Text style={styles.buttonText}>Rate -</Text>
            </View>
          </TouchableHighlight>
        </View>
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    alignItems: 'center',
    justifyContent: 'center',
    backgroundColor: 'white',
  },
  container2: {
    justifyContent: 'center',
    alignItems: 'center',
  },
  container3: {
    flexDirection: 'row',
    justifyContent: 'center',
  },
  textStyle: {
    color: 'black',
    paddingLeft: 10,
  },
  button: {
    padding: 10,
    width: 120,
    margin: 5,
    alignItems: 'center',
    backgroundColor: 'grey',
  },
  buttonText: {
    color: 'white',
    fontSize: 20,
  },
});
