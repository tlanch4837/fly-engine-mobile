// iOS shell: a UIKit scene with a CAMetalLayer view, the Metal renderer, and touch / game controllers -> fly::Input.
// Everything else is the portable game (src/fly/game.h). Based on cityengine-ios's shell (same scene delegate name as Info.plist.in).
//
// Touch: left half of the screen is a floating stick (where the finger lands is centre; drag up = nose down, as a real stick
// pushed forward; left/right = roll). The right 18% of the screen is the throttle: the finger's height sets it (top = full).
// Two-finger tap: next camera. Three-finger tap: reset.
// Controllers (GameController framework: MFi, Xbox, PlayStation): left stick pitch/roll, triggers rudder, right stick Y
// throttle, Y / triangle camera, Options / menu reset.
#import <GameController/GameController.h>
#import <QuartzCore/CAMetalLayer.h>
#import <UIKit/UIKit.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "../../fly/game.h"
#include "../../render/renderer.h"

@interface FlyMetalView : UIView
@end
@implementation FlyMetalView
+ (Class)layerClass { return [CAMetalLayer class]; }
@end

@interface FlyViewController : UIViewController
- (void)resumeFrames;
- (void)pauseFrames;
@end

@implementation FlyViewController {
    CADisplayLink* _link;
    BOOL _started, _failed;
    std::unique_ptr<tbd::Renderer> _renderer;
    std::unique_ptr<fly::Game> _game;
    CFTimeInterval _last;
    // touch state
    UITouch* _stickTouch;
    CGPoint _stickOrigin, _stickNow;
    UITouch* _throttleTouch;
    float _throttle;
    BOOL _nextCamera, _reset;
    BOOL _padYWas, _padMenuWas;
}

- (void)loadView {
    FlyMetalView* view = [[FlyMetalView alloc] initWithFrame:CGRectZero];
    view.multipleTouchEnabled = YES;
    self.view = view;
    _throttle = -1;
    UITapGestureRecognizer* two = [[UITapGestureRecognizer alloc] initWithTarget:self action:@selector(twoTap)];
    two.numberOfTouchesRequired = 2;
    two.cancelsTouchesInView = NO;
    [view addGestureRecognizer:two];
    UITapGestureRecognizer* three = [[UITapGestureRecognizer alloc] initWithTarget:self action:@selector(threeTap)];
    three.numberOfTouchesRequired = 3;
    three.cancelsTouchesInView = NO;
    [view addGestureRecognizer:three];
}
- (void)twoTap { _nextCamera = YES; }
- (void)threeTap { _reset = YES; }

- (void)resumeFrames {
    if (_link || _failed) return;
    _link = [CADisplayLink displayLinkWithTarget:self selector:@selector(frame:)];
    // up to ProMotion's 120 Hz (needs CADisableMinimumFrameDurationOnPhone in Info.plist): the HUD shows what the phone holds.
    // A battery / heat setting can lower this later; the VR target is a steady 90.
    _link.preferredFrameRateRange = CAFrameRateRangeMake(30, 120, 120);
    [_link addToRunLoop:[NSRunLoop mainRunLoop] forMode:NSRunLoopCommonModes];
}
- (void)pauseFrames {
    [_link invalidate];
    _link = nil;
}

// The view must render at the screen's native scale, or iOS stretches a point-sized drawable (blurry HUD).
- (CGFloat)pixelScale {
    CGFloat s = 0.0;
    if (UIWindowScene* scene = self.view.window.windowScene) s = scene.screen.nativeScale;
    if (s <= 0.0) s = self.traitCollection.displayScale;
    if (s <= 0.0) s = 1.0;
    if (self.view.contentScaleFactor != s) self.view.contentScaleFactor = s;
    CAMetalLayer* layer = (CAMetalLayer*)self.view.layer;
    if (layer.contentsScale != s) layer.contentsScale = s;
    return s;
}

- (BOOL)startGame {
    CAMetalLayer* layer = (CAMetalLayer*)self.view.layer;
    const CGFloat scale = [self pixelScale];
    const int w = (int)(self.view.bounds.size.width * scale), h = (int)(self.view.bounds.size.height * scale);
    if (!self.view.window || w <= 0 || h <= 0) return NO;
    _renderer = tbd::CreateMetalRenderer();
    if (!_renderer || !_renderer->Init((__bridge void*)layer, w, h, false)) {
        tbd::Log("iOS: the Metal renderer failed to start");
        _failed = YES;
        [self pauseFrames];
        return NO;
    }
    fly::Settings settings;
    // data/ (airports, world, ui) is copied into the bundle by CMakeLists.txt; a data folder in Documents (Files app) wins, for quick iteration.
    NSString* docs = [NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES) firstObject];
    NSString* docData = [docs stringByAppendingPathComponent:@"data"];
    NSString* bundleData = [[[NSBundle mainBundle] resourcePath] stringByAppendingPathComponent:@"data"];
    const BOOL useDocs = [[NSFileManager defaultManager] fileExistsAtPath:[docData stringByAppendingPathComponent:@"airports"]];
    settings.dataDir = std::string((useDocs ? docData : bundleData).fileSystemRepresentation);
    settings.hudScale = (float)std::max<CGFloat>(2.0, scale * 1.2);
    settings.touchHints = YES;
    _game = std::make_unique<fly::Game>();
    _game->Resize(w, h);
    if (!_game->Start(*_renderer, settings)) { _failed = YES; [self pauseFrames]; return NO; }
    tbd::Log("iOS: Fly-engine started (%d x %d, scale %.1f)", w, h, (double)scale);
    _last = CACurrentMediaTime();
    return YES;
}

- (void)viewDidLayoutSubviews {
    [super viewDidLayoutSubviews];
    if (!_started) return;
    const CGFloat scale = [self pixelScale];
    const int w = (int)(self.view.bounds.size.width * scale), h = (int)(self.view.bounds.size.height * scale);
    _renderer->Resize(w, h);
    _game->Resize(w, h);
}

- (fly::Input)gatherInput {
    fly::Input in;
    const CGFloat stickRange = 70.0; // points of finger travel for full deflection
    if (_stickTouch) {
        in.roll = (float)std::clamp((_stickNow.x - _stickOrigin.x) / stickRange, -1.0, 1.0);
        in.pitch = (float)std::clamp((_stickNow.y - _stickOrigin.y) / stickRange, -1.0, 1.0); // finger down = stick back = nose up
    }
    in.throttleAbs = _throttle;
    in.nextCamera = _nextCamera;
    in.reset = _reset;
    _nextCamera = _reset = NO;
    // a connected controller overrides touch where it has input
    if (GCController* pad = GCController.current) {
        if (GCExtendedGamepad* g = pad.extendedGamepad) {
            const float lx = g.leftThumbstick.xAxis.value, ly = g.leftThumbstick.yAxis.value;
            if (std::abs(lx) > 0.1f || std::abs(ly) > 0.1f) { in.roll = lx; in.pitch = -ly; }
            const float yaw = g.rightTrigger.value - g.leftTrigger.value;
            if (std::abs(yaw) > 0.05f) in.yaw = yaw;
            const float ry = g.rightThumbstick.yAxis.value;
            if (std::abs(ry) > 0.12f) in.throttleRate = ry;
            const BOOL y = g.buttonY.pressed, menu = g.buttonMenu.pressed;
            in.nextCamera |= y && !_padYWas;
            in.reset |= menu && !_padMenuWas;
            _padYWas = y; _padMenuWas = menu;
        }
    }
    return in;
}

- (void)frame:(CADisplayLink*)link {
    if (!_started) {
        if (![self startGame]) return;
        _started = YES;
    }
    const CFTimeInterval now = CACurrentMediaTime();
    const float dt = (float)std::min(0.05, now - _last);
    _last = now;
    _game->Frame(*_renderer, dt, [self gatherInput]);
    _renderer->EndFrame();
    _throttle = -1; // the slider sets the throttle only while touched; a controller's rate can move it after
}

// ---- touches: stick on the left half, throttle on the right edge
- (void)touchesBegan:(NSSet<UITouch*>*)touches withEvent:(UIEvent*)event {
    const CGFloat W = self.view.bounds.size.width;
    for (UITouch* t in touches) {
        const CGPoint p = [t locationInView:self.view];
        if (p.x > W * 0.82 && !_throttleTouch) { _throttleTouch = t; [self throttleAt:p]; }
        else if (p.x < W * 0.5 && !_stickTouch) { _stickTouch = t; _stickOrigin = _stickNow = p; }
    }
}
- (void)throttleAt:(CGPoint)p {
    const CGFloat H = self.view.bounds.size.height, margin = H * 0.12;
    _throttle = (float)std::clamp(1.0 - (p.y - margin) / (H - 2 * margin), 0.0, 1.0);
}
- (void)touchesMoved:(NSSet<UITouch*>*)touches withEvent:(UIEvent*)event {
    for (UITouch* t in touches) {
        if (t == _stickTouch) _stickNow = [t locationInView:self.view];
        else if (t == _throttleTouch) [self throttleAt:[t locationInView:self.view]];
    }
}
- (void)touchesEnded:(NSSet<UITouch*>*)touches withEvent:(UIEvent*)event {
    for (UITouch* t in touches) {
        if (t == _stickTouch) _stickTouch = nil;
        if (t == _throttleTouch) _throttleTouch = nil;
    }
}
- (void)touchesCancelled:(NSSet<UITouch*>*)touches withEvent:(UIEvent*)event { [self touchesEnded:touches withEvent:event]; }

- (BOOL)prefersStatusBarHidden { return YES; }
- (BOOL)prefersHomeIndicatorAutoHidden { return YES; }
@end

@interface TbdSceneDelegate : UIResponder <UIWindowSceneDelegate>
@property(strong, nonatomic) UIWindow* window;
@end

@implementation TbdSceneDelegate {
    FlyViewController* _controller;
}
- (void)scene:(UIScene*)scene willConnectToSession:(UISceneSession*)session options:(UISceneConnectionOptions*)options {
    if (![scene isKindOfClass:[UIWindowScene class]]) return;
    _controller = [[FlyViewController alloc] init];
    self.window = [[UIWindow alloc] initWithWindowScene:(UIWindowScene*)scene];
    self.window.rootViewController = _controller;
    [self.window makeKeyAndVisible];
}
- (void)sceneDidBecomeActive:(UIScene*)scene { [_controller resumeFrames]; }
- (void)sceneWillResignActive:(UIScene*)scene { [_controller pauseFrames]; }
- (void)sceneDidDisconnect:(UIScene*)scene {
    [_controller pauseFrames];
    self.window = nil;
    _controller = nil;
}
@end

@interface TbdAppDelegate : UIResponder <UIApplicationDelegate>
@end
@implementation TbdAppDelegate
@end

int main(int argc, char* argv[]) {
    @autoreleasepool {
        return UIApplicationMain(argc, argv, nil, NSStringFromClass([TbdAppDelegate class]));
    }
}
