#import <React/RCTComponent.h>
#import <CoreMotion/CoreMotion.h>
#import <QuartzCore/QuartzCore.h>
#import <NativeAhrsSpec/NativeAhrsSpec.h>

static const NSInteger FRAME_RATE = 60;
static const NSTimeInterval INTERVAL = (1.0 / (double)FRAME_RATE);

NS_ASSUME_NONNULL_BEGIN

@interface NativeAhrs : NativeAhrsSpecBase <NativeAhrsSpec>

// Core sensor properties
@property (nonatomic, strong, nullable) CMMotionManager *motionManager;
@property (nonatomic, strong, nullable) CADisplayLink *displayLink;
@property (nonatomic, assign) CFTimeInterval lastDisplayLinkTimestamp;
@property (nonatomic, assign) NSTimeInterval nextEmitTime;

// AHRS configuration
@property (nonatomic, assign) bool running;
@property (nonatomic, assign) int rotation;
@property (nonatomic, assign) float gain;
@property (nonatomic, assign) float rate;

@end

NS_ASSUME_NONNULL_END

