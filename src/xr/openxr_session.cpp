// OpenXR layer (see openxr_session.h). Vulkan graphics via XR_KHR_vulkan_enable2; seated (LOCAL) reference space;
// one swapchain per eye; Touch-style controller actions mapped onto fly::Input.
#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#define XR_USE_PLATFORM_WIN32
#endif
#include <vulkan/vulkan.h>
#define XR_USE_GRAPHICS_API_VULKAN
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include "openxr_session.h"

namespace fly::xr {

namespace {
bool Ok(XrResult r, const char* what) {
    if (XR_SUCCEEDED(r)) return true;
    tbd::Log("OpenXR: %s failed (%d)", what, (int)r);
    return false;
}
} // namespace

struct Session::Impl {
    XrInstance instance = XR_NULL_HANDLE;
    XrSystemId system = XR_NULL_SYSTEM_ID;
    XrSession session = XR_NULL_HANDLE;
    XrSpace space = XR_NULL_HANDLE;
    XrSessionState state = XR_SESSION_STATE_UNKNOWN;
    bool running = false, quit = false;
    XrViewConfigurationView viewConfig[2] = {{XR_TYPE_VIEW_CONFIGURATION_VIEW}, {XR_TYPE_VIEW_CONFIGURATION_VIEW}};
    struct Eye {
        XrSwapchain swapchain = XR_NULL_HANDLE;
        std::vector<XrSwapchainImageVulkan2KHR> images;
    } eyes[2];
    int64_t format = 0;
    int width = 0, height = 0;
    XrTime lastTime = 0;

    // functions from the extension
    PFN_xrGetVulkanGraphicsRequirements2KHR getRequirements = nullptr;
    PFN_xrCreateVulkanInstanceKHR createVkInstance = nullptr;
    PFN_xrGetVulkanGraphicsDevice2KHR getVkDevice = nullptr;
    PFN_xrCreateVulkanDeviceKHR createVkDevice = nullptr;
    tbd::Renderer::XrVulkanBridge bridge;

    // input
    XrActionSet actionSet = XR_NULL_HANDLE;
    XrAction stickRight = XR_NULL_HANDLE, stickLeft = XR_NULL_HANDLE, trigger = XR_NULL_HANDLE, cameraButton = XR_NULL_HANDLE,
             resetButton = XR_NULL_HANDLE;
    XrPath hands[2] = {XR_NULL_PATH, XR_NULL_PATH};
    bool camWas = false, resetWas = false;

    XrPath Path(const char* s) {
        XrPath p = XR_NULL_PATH;
        xrStringToPath(instance, s, &p);
        return p;
    }
    bool MakeAction(XrAction& out, const char* name, const char* label, XrActionType type, bool perHand) {
        XrActionCreateInfo ai{XR_TYPE_ACTION_CREATE_INFO};
        ai.actionType = type;
        std::strncpy(ai.actionName, name, XR_MAX_ACTION_NAME_SIZE - 1);
        std::strncpy(ai.localizedActionName, label, XR_MAX_LOCALIZED_ACTION_NAME_SIZE - 1);
        ai.countSubactionPaths = perHand ? 2 : 0;
        ai.subactionPaths = perHand ? hands : nullptr;
        return Ok(xrCreateAction(actionSet, &ai, &out), name);
    }
    bool CreateActions() {
        XrActionSetCreateInfo si{XR_TYPE_ACTION_SET_CREATE_INFO};
        std::strcpy(si.actionSetName, "flight");
        std::strcpy(si.localizedActionSetName, "Flight");
        if (!Ok(xrCreateActionSet(instance, &si, &actionSet), "xrCreateActionSet")) return false;
        hands[0] = Path("/user/hand/left");
        hands[1] = Path("/user/hand/right");
        // Right stick flies (pitch / roll), left stick is throttle (y) and rudder (x). A / X: camera. B / Y: reset.
        if (!MakeAction(stickRight, "stick", "Stick (pitch and roll)", XR_ACTION_TYPE_VECTOR2F_INPUT, false) ||
            !MakeAction(stickLeft, "throttle_rudder", "Throttle and rudder", XR_ACTION_TYPE_VECTOR2F_INPUT, false) ||
            !MakeAction(cameraButton, "camera", "Next camera", XR_ACTION_TYPE_BOOLEAN_INPUT, false) ||
            !MakeAction(resetButton, "reset", "Reset", XR_ACTION_TYPE_BOOLEAN_INPUT, false))
            return false;
        const XrActionSuggestedBinding touch[] = {
            {stickRight, Path("/user/hand/right/input/thumbstick")},
            {stickLeft, Path("/user/hand/left/input/thumbstick")},
            {cameraButton, Path("/user/hand/right/input/a/click")},
            {cameraButton, Path("/user/hand/left/input/x/click")},
            {resetButton, Path("/user/hand/right/input/b/click")},
            {resetButton, Path("/user/hand/left/input/y/click")},
        };
        XrInteractionProfileSuggestedBinding sb{XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
        sb.interactionProfile = Path("/interaction_profiles/oculus/touch_controller");
        sb.countSuggestedBindings = (uint32_t)(sizeof(touch) / sizeof(touch[0]));
        sb.suggestedBindings = touch;
        Ok(xrSuggestInteractionProfileBindings(instance, &sb), "touch bindings");
        // Valve Index and similar: same layout (thumbsticks, A / B on both hands).
        const XrActionSuggestedBinding index[] = {
            {stickRight, Path("/user/hand/right/input/thumbstick")},
            {stickLeft, Path("/user/hand/left/input/thumbstick")},
            {cameraButton, Path("/user/hand/right/input/a/click")},
            {resetButton, Path("/user/hand/right/input/b/click")},
        };
        sb.interactionProfile = Path("/interaction_profiles/valve/index_controller");
        sb.countSuggestedBindings = (uint32_t)(sizeof(index) / sizeof(index[0]));
        sb.suggestedBindings = index;
        Ok(xrSuggestInteractionProfileBindings(instance, &sb), "index bindings");
        XrSessionActionSetsAttachInfo ai{XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO};
        ai.countActionSets = 1;
        ai.actionSets = &actionSet;
        return Ok(xrAttachSessionActionSets(session, &ai), "xrAttachSessionActionSets");
    }
    XrVector2f Stick(XrAction a) {
        XrActionStateGetInfo gi{XR_TYPE_ACTION_STATE_GET_INFO};
        gi.action = a;
        XrActionStateVector2f st{XR_TYPE_ACTION_STATE_VECTOR2F};
        if (XR_SUCCEEDED(xrGetActionStateVector2f(session, &gi, &st)) && st.isActive) return st.currentState;
        return {0, 0};
    }
    bool Button(XrAction a) {
        XrActionStateGetInfo gi{XR_TYPE_ACTION_STATE_GET_INFO};
        gi.action = a;
        XrActionStateBoolean st{XR_TYPE_ACTION_STATE_BOOLEAN};
        return XR_SUCCEEDED(xrGetActionStateBoolean(session, &gi, &st)) && st.isActive && st.currentState;
    }

    void PollEvents() {
        XrEventDataBuffer ev{XR_TYPE_EVENT_DATA_BUFFER};
        while (xrPollEvent(instance, &ev) == XR_SUCCESS) {
            if (ev.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
                const auto& sc = *reinterpret_cast<XrEventDataSessionStateChanged*>(&ev);
                state = sc.state;
                tbd::Log("OpenXR: session state %d", (int)state);
                if (state == XR_SESSION_STATE_READY) {
                    XrSessionBeginInfo bi{XR_TYPE_SESSION_BEGIN_INFO};
                    bi.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
                    running = Ok(xrBeginSession(session, &bi), "xrBeginSession");
                } else if (state == XR_SESSION_STATE_STOPPING) {
                    xrEndSession(session);
                    running = false;
                } else if (state == XR_SESSION_STATE_EXITING || state == XR_SESSION_STATE_LOSS_PENDING) {
                    quit = true;
                }
            } else if (ev.type == XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING) {
                quit = true;
            }
            ev = {XR_TYPE_EVENT_DATA_BUFFER};
        }
    }
};

Session::Session() : m_(std::make_unique<Impl>()) {}

Session::~Session() {
    Impl& x = *m_;
    for (auto& e : x.eyes)
        if (e.swapchain) xrDestroySwapchain(e.swapchain);
    if (x.actionSet) xrDestroyActionSet(x.actionSet);
    if (x.space) xrDestroySpace(x.space);
    if (x.session) xrDestroySession(x.session);
    if (x.instance) xrDestroyInstance(x.instance);
}

bool Session::CreateInstance() {
    Impl& x = *m_;
    const char* extensions[] = {XR_KHR_VULKAN_ENABLE2_EXTENSION_NAME};
    XrInstanceCreateInfo ci{XR_TYPE_INSTANCE_CREATE_INFO};
    std::strcpy(ci.applicationInfo.applicationName, "Fly-engine");
    std::strcpy(ci.applicationInfo.engineName, "Fly-engine");
    ci.applicationInfo.apiVersion = XR_API_VERSION_1_0; // 1.0 is what every runtime accepts
    ci.enabledExtensionCount = 1;
    ci.enabledExtensionNames = extensions;
    if (!Ok(xrCreateInstance(&ci, &x.instance), "xrCreateInstance (is an OpenXR runtime installed and set active?)")) return false;
    XrInstanceProperties ip{XR_TYPE_INSTANCE_PROPERTIES};
    if (XR_SUCCEEDED(xrGetInstanceProperties(x.instance, &ip))) tbd::Log("OpenXR: runtime %s", ip.runtimeName);
    XrSystemGetInfo si{XR_TYPE_SYSTEM_GET_INFO};
    si.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    if (!Ok(xrGetSystem(x.instance, &si, &x.system), "xrGetSystem (headset connected and awake?)")) return false;
    XrSystemProperties sp{XR_TYPE_SYSTEM_PROPERTIES};
    if (XR_SUCCEEDED(xrGetSystemProperties(x.instance, x.system, &sp))) tbd::Log("OpenXR: headset %s", sp.systemName);

    uint32_t n = 0;
    xrEnumerateViewConfigurationViews(x.instance, x.system, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 2, &n, x.viewConfig);
    x.width = (int)x.viewConfig[0].recommendedImageRectWidth;
    x.height = (int)x.viewConfig[0].recommendedImageRectHeight;
    tbd::Log("OpenXR: %d x %d per eye", x.width, x.height);

    xrGetInstanceProcAddr(x.instance, "xrGetVulkanGraphicsRequirements2KHR", (PFN_xrVoidFunction*)&x.getRequirements);
    xrGetInstanceProcAddr(x.instance, "xrCreateVulkanInstanceKHR", (PFN_xrVoidFunction*)&x.createVkInstance);
    xrGetInstanceProcAddr(x.instance, "xrGetVulkanGraphicsDevice2KHR", (PFN_xrVoidFunction*)&x.getVkDevice);
    xrGetInstanceProcAddr(x.instance, "xrCreateVulkanDeviceKHR", (PFN_xrVoidFunction*)&x.createVkDevice);
    if (!x.getRequirements || !x.createVkInstance || !x.getVkDevice || !x.createVkDevice) {
        tbd::Log("OpenXR: XR_KHR_vulkan_enable2 functions missing");
        return false;
    }
    XrGraphicsRequirementsVulkan2KHR req{XR_TYPE_GRAPHICS_REQUIREMENTS_VULKAN2_KHR};
    if (!Ok(x.getRequirements(x.instance, x.system, &req), "graphics requirements")) return false; // required before the session

    Impl* self = &x;
    x.bridge.createInstance = [self](const void* vkInfo, void** out) -> int {
        XrVulkanInstanceCreateInfoKHR ci{XR_TYPE_VULKAN_INSTANCE_CREATE_INFO_KHR};
        ci.systemId = self->system;
        ci.pfnGetInstanceProcAddr = &vkGetInstanceProcAddr;
        ci.vulkanCreateInfo = static_cast<const VkInstanceCreateInfo*>(vkInfo);
        VkInstance inst = VK_NULL_HANDLE;
        VkResult vr = VK_SUCCESS;
        if (!Ok(self->createVkInstance(self->instance, &ci, &inst, &vr), "xrCreateVulkanInstanceKHR")) return VK_ERROR_INITIALIZATION_FAILED;
        *out = inst;
        return vr;
    };
    x.bridge.physicalDevice = [self](void* inst) -> void* {
        XrVulkanGraphicsDeviceGetInfoKHR gi{XR_TYPE_VULKAN_GRAPHICS_DEVICE_GET_INFO_KHR};
        gi.systemId = self->system;
        gi.vulkanInstance = (VkInstance)inst;
        VkPhysicalDevice phys = VK_NULL_HANDLE;
        if (!Ok(self->getVkDevice(self->instance, &gi, &phys), "xrGetVulkanGraphicsDevice2KHR")) return nullptr;
        return phys;
    };
    x.bridge.createDevice = [self](void* phys, const void* vkInfo, void** out) -> int {
        XrVulkanDeviceCreateInfoKHR ci{XR_TYPE_VULKAN_DEVICE_CREATE_INFO_KHR};
        ci.systemId = self->system;
        ci.pfnGetInstanceProcAddr = &vkGetInstanceProcAddr;
        ci.vulkanPhysicalDevice = (VkPhysicalDevice)phys;
        ci.vulkanCreateInfo = static_cast<const VkDeviceCreateInfo*>(vkInfo);
        VkDevice dev = VK_NULL_HANDLE;
        VkResult vr = VK_SUCCESS;
        if (!Ok(self->createVkDevice(self->instance, &ci, &dev, &vr), "xrCreateVulkanDeviceKHR")) return VK_ERROR_INITIALIZATION_FAILED;
        *out = dev;
        return vr;
    };
    return true;
}

const tbd::Renderer::XrVulkanBridge* Session::Bridge() const { return &m_->bridge; }
void Session::EyeSize(int& width, int& height) const { width = m_->width; height = m_->height; }

bool Session::Start(tbd::Renderer& r) {
    Impl& x = *m_;
    void *inst = nullptr, *phys = nullptr, *dev = nullptr;
    uint32_t family = 0, queueIndex = 0;
    if (!r.NativeHandles(&inst, &phys, &dev, &family, &queueIndex)) { tbd::Log("OpenXR: the renderer is not Vulkan"); return false; }
    XrGraphicsBindingVulkan2KHR binding{XR_TYPE_GRAPHICS_BINDING_VULKAN2_KHR};
    binding.instance = (VkInstance)inst;
    binding.physicalDevice = (VkPhysicalDevice)phys;
    binding.device = (VkDevice)dev;
    binding.queueFamilyIndex = family;
    binding.queueIndex = queueIndex;
    XrSessionCreateInfo ci{XR_TYPE_SESSION_CREATE_INFO};
    ci.next = &binding;
    ci.systemId = x.system;
    if (!Ok(xrCreateSession(x.instance, &ci, &x.session), "xrCreateSession")) return false;

    XrReferenceSpaceCreateInfo rs{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    rs.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL; // seated: origin where the head was at start (recentre from the system menu)
    rs.poseInReferenceSpace.orientation.w = 1;
    if (!Ok(xrCreateReferenceSpace(x.session, &rs, &x.space), "xrCreateReferenceSpace")) return false;

    // Swapchain format: B8G8R8A8_SRGB has the same bits as the renderer's B8G8R8A8_UNORM output, so a plain copy is exact.
    uint32_t n = 0;
    xrEnumerateSwapchainFormats(x.session, 0, &n, nullptr);
    std::vector<int64_t> formats(n);
    xrEnumerateSwapchainFormats(x.session, n, &n, formats.data());
    x.format = 0;
    for (int64_t f : formats) if (f == VK_FORMAT_B8G8R8A8_SRGB) x.format = f;
    if (!x.format) {
        tbd::Log("OpenXR: the runtime offers no B8G8R8A8_SRGB swapchain (formats: %u); not supported yet", n);
        return false;
    }
    for (auto& e : x.eyes) {
        XrSwapchainCreateInfo sc{XR_TYPE_SWAPCHAIN_CREATE_INFO};
        sc.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT;
        sc.format = x.format;
        sc.sampleCount = 1;
        sc.width = (uint32_t)x.width;
        sc.height = (uint32_t)x.height;
        sc.faceCount = 1;
        sc.arraySize = 1;
        sc.mipCount = 1;
        if (!Ok(xrCreateSwapchain(x.session, &sc, &e.swapchain), "xrCreateSwapchain")) return false;
        uint32_t count = 0;
        xrEnumerateSwapchainImages(e.swapchain, 0, &count, nullptr);
        e.images.assign(count, {XR_TYPE_SWAPCHAIN_IMAGE_VULKAN2_KHR});
        xrEnumerateSwapchainImages(e.swapchain, count, &count, reinterpret_cast<XrSwapchainImageBaseHeader*>(e.images.data()));
    }
    r.SetTemporalHistory(false); // eyes alternate frames: TAA would blend one eye into the other
    return x.CreateActions();
}

bool Session::Frame(tbd::Renderer& r, Game& game, const Input& extra) {
    Impl& x = *m_;
    x.PollEvents();
    if (x.quit) return false;
    if (!x.running) return true; // idle (headset off the face, menu open): the shell keeps pumping messages

    XrFrameWaitInfo wi{XR_TYPE_FRAME_WAIT_INFO};
    XrFrameState fs{XR_TYPE_FRAME_STATE};
    if (!Ok(xrWaitFrame(x.session, &wi, &fs), "xrWaitFrame")) return true;
    XrFrameBeginInfo bi{XR_TYPE_FRAME_BEGIN_INFO};
    if (!Ok(xrBeginFrame(x.session, &bi), "xrBeginFrame")) return true;

    // input: controllers first, other devices where the controllers are idle
    Input in = extra;
    const XrActiveActionSet active{x.actionSet, XR_NULL_PATH};
    XrActionsSyncInfo sync{XR_TYPE_ACTIONS_SYNC_INFO};
    sync.countActiveActionSets = 1;
    sync.activeActionSets = &active;
    if (XR_SUCCEEDED(xrSyncActions(x.session, &sync))) {
        const XrVector2f right = x.Stick(x.stickRight), left = x.Stick(x.stickLeft);
        if (std::abs(right.x) > 0.08f || std::abs(right.y) > 0.08f) { in.roll = right.x; in.pitch = -right.y; } // stick back = nose up
        if (std::abs(left.x) > 0.15f) in.yaw = left.x;
        if (std::abs(left.y) > 0.1f) in.throttleRate = left.y;
        const bool cam = x.Button(x.cameraButton), reset = x.Button(x.resetButton);
        in.nextCamera |= cam && !x.camWas;
        in.reset |= reset && !x.resetWas;
        x.camWas = cam; x.resetWas = reset;
    }
    const float dt = x.lastTime ? (float)std::clamp((fs.predictedDisplayTime - x.lastTime) * 1e-9, 0.0, 0.05) : 1.0f / 72.0f;
    x.lastTime = fs.predictedDisplayTime;
    game.Simulate(dt, in);

    XrCompositionLayerProjectionView projViews[2] = {{XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW}, {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW}};
    XrCompositionLayerProjection layer{XR_TYPE_COMPOSITION_LAYER_PROJECTION};
    const XrCompositionLayerBaseHeader* layers[1] = {reinterpret_cast<XrCompositionLayerBaseHeader*>(&layer)};
    uint32_t layerCount = 0;
    if (fs.shouldRender) {
        XrViewLocateInfo li{XR_TYPE_VIEW_LOCATE_INFO};
        li.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
        li.displayTime = fs.predictedDisplayTime;
        li.space = x.space;
        XrViewState vs{XR_TYPE_VIEW_STATE};
        XrView views[2] = {{XR_TYPE_VIEW}, {XR_TYPE_VIEW}};
        uint32_t n = 0;
        if (XR_SUCCEEDED(xrLocateViews(x.session, &li, &vs, 2, &n, views)) && n == 2 &&
            (vs.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT)) {
            for (int e = 0; e < 2; ++e) {
                Impl::Eye& eye = x.eyes[e];
                uint32_t index = 0;
                XrSwapchainImageAcquireInfo ai{XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
                if (!Ok(xrAcquireSwapchainImage(eye.swapchain, &ai, &index), "acquire")) continue;
                XrSwapchainImageWaitInfo wait{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
                wait.timeout = XR_INFINITE_DURATION;
                xrWaitSwapchainImage(eye.swapchain, &wait);

                const XrView& v = views[e];
                EyeView ev;
                ev.position = {v.pose.position.x, v.pose.position.y, v.pose.position.z};
                ev.orientation = {v.pose.orientation.x, v.pose.orientation.y, v.pose.orientation.z, v.pose.orientation.w};
                ev.tanLeft = std::tan(v.fov.angleLeft); ev.tanRight = std::tan(v.fov.angleRight);
                ev.tanUp = std::tan(v.fov.angleUp); ev.tanDown = std::tan(v.fov.angleDown);
                game.Render(r, &ev);
                r.SetEndFrameCopyTarget((uint64_t)eye.images[index].image);
                r.EndFrame();

                XrSwapchainImageReleaseInfo ri{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
                xrReleaseSwapchainImage(eye.swapchain, &ri);
                projViews[e].pose = v.pose;
                projViews[e].fov = v.fov;
                projViews[e].subImage.swapchain = eye.swapchain;
                projViews[e].subImage.imageRect = {{0, 0}, {x.width, x.height}};
            }
            layer.space = x.space;
            layer.viewCount = 2;
            layer.views = projViews;
            layerCount = 1;
        }
    }
    XrFrameEndInfo ei{XR_TYPE_FRAME_END_INFO};
    ei.displayTime = fs.predictedDisplayTime;
    ei.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    ei.layerCount = layerCount;
    ei.layers = layerCount ? layers : nullptr;
    Ok(xrEndFrame(x.session, &ei), "xrEndFrame");
    return true;
}

} // namespace fly::xr
