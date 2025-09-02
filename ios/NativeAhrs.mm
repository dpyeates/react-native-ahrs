#import "NativeAhrs.h"
#import "../fusion/FusionBridge.h"

@implementation NativeAhrs
RCT_EXPORT_MODULE()

- (instancetype)init {
  self = [super init];
  if (self) {
    self.running = false;
    self.nextEmitTime = 0;
    self.lastDisplayLinkTimestamp = 0;
    self.rotation = 0;
    self.gain = 3.0;
    self.rate = 5.0;
    self.motionManager = [[CMMotionManager alloc] init];
    self.motionManager.deviceMotionUpdateInterval = INTERVAL;
    self.motionManager.accelerometerUpdateInterval = INTERVAL;
    self.motionManager.gyroUpdateInterval = INTERVAL;
    self.motionManager.magnetometerUpdateInterval = INTERVAL;
    [self resetAhrs];
  }
  return self;
}

RCT_EXPORT_METHOD(startAhrs) {
  if (self.running) {
    NSLog(@"⚠️ AHRS already running");
    return;
  }
  self.running = YES;
  [self.motionManager startAccelerometerUpdates];
  [self.motionManager startGyroUpdates];
  [self.motionManager startDeviceMotionUpdatesUsingReferenceFrame:
   CMAttitudeReferenceFrameXMagneticNorthZVertical];
  self.displayLink = [CADisplayLink displayLinkWithTarget:self
                                                 selector:@selector(displayLinkUpdate:)];
  self.displayLink.preferredFramesPerSecond = FRAME_RATE;
  [self.displayLink addToRunLoop:[NSRunLoop mainRunLoop] forMode:NSDefaultRunLoopMode];
}

RCT_EXPORT_METHOD(stopAhrs) {
  if (!self.running) {
    NSLog(@"⚠️ AHRS not running");
    return;
  }
  self.running = NO;
  if (self.displayLink != nil) {
    [self.displayLink invalidate];
    self.displayLink = nil;
  }
  [self.motionManager stopDeviceMotionUpdates];
  [self.motionManager stopAccelerometerUpdates];
  [self.motionManager stopGyroUpdates];
  self.lastDisplayLinkTimestamp = 0;
}

- (void)displayLinkUpdate:(CADisplayLink *)displayLink {
  if (!self.running) {
    return;
  }

  if (self.lastDisplayLinkTimestamp != 0) {

    CFTimeInterval deltaTime = displayLink.timestamp - self.lastDisplayLinkTimestamp;

    float accelArray[3] = {
      // Use raw not processed
      (float)self.motionManager.accelerometerData.acceleration.x,
      (float)self.motionManager.accelerometerData.acceleration.y,
      (float)self.motionManager.accelerometerData.acceleration.z
    };

    float gyroArray[3] = {
      // Use raw not processed
      (float)(self.motionManager.gyroData.rotationRate.x * 180.0f / M_PI), // rad/s -> deg/s
      (float)(self.motionManager.gyroData.rotationRate.y * 180.0f / M_PI),
      (float)(self.motionManager.gyroData.rotationRate.z * 180.0f / M_PI)
    };

    float magnetArray[3] = {
      // Use calibrated mag field from device motion
      (float)self.motionManager.deviceMotion.magneticField.field.x,
      (float)self.motionManager.deviceMotion.magneticField.field.y,
      (float)self.motionManager.deviceMotion.magneticField.field.z
    };

    updateAhrs((float)deltaTime, accelArray, gyroArray, magnetArray);

    // Send data back to the Javascript RN side at our custom rate
    if (displayLink.timestamp > self.nextEmitTime) {
      [self emitOnAhrsUpdate:@{
        @"roll": @(getAhrsRoll()),
        @"pitch": @(getAhrsPitch()),
        @"heading": @(getAhrsHeading()),
      }];
      self.nextEmitTime = displayLink.timestamp + (1.0 / self.rate);
    }
  }

  self.lastDisplayLinkTimestamp = displayLink.timestamp;
}

RCT_EXPORT_METHOD(resetAhrs) {
  initAhrs(0, self.rotation, self.gain);
}

RCT_EXPORT_METHOD(levelAhrs) {
  zeroAhrs();
}

RCT_EXPORT_METHOD(setAhrsGain:(double)newGain) {
  if (newGain > 0) {
    self.gain = newGain;
    [self resetAhrs];
  }
}

RCT_EXPORT_METHOD(setAhrsRate:(double)newRate) {
  if (newRate >= 1 && newRate <= 60) {
    self.rate = newRate;
  }
}

RCT_EXPORT_METHOD(setAhrsRotation:(NSString *)newRotation) {
  if ([newRotation caseInsensitiveCompare:@"left"] == NSOrderedSame) {
    self.rotation = -1;
  } else if ([newRotation caseInsensitiveCompare:@"right"] == NSOrderedSame) {
    self.rotation = 1;
  } else {
    self.rotation = 0;
  }
  setAhrsInterfaceRotation(self.rotation);
}

RCT_EXPORT_METHOD(isSupported:(RCTPromiseResolveBlock)resolve
                  rejecter:(RCTPromiseRejectBlock)reject) {
  BOOL supported = self.motionManager.isDeviceMotionAvailable &&
                   self.motionManager.isAccelerometerAvailable &&
                   self.motionManager.isGyroAvailable &&
                   self.motionManager.isMagnetometerAvailable;
  resolve(@(supported));
}

#pragma mark - Lifecycle

- (std::shared_ptr<facebook::react::TurboModule>)getTurboModule:
(const facebook::react::ObjCTurboModule::InitParams &)params {
  return std::make_shared<facebook::react::NativeAhrsSpecJSI>(params);
}

- (void)dealloc {
  [self stopAhrs];
}

- (void)invalidate {
  [self stopAhrs];
}

@end
