// Graphics API abstraction. One interface, two backends (Direct3D 12 and Vulkan) that draw the
// same thing the same way. Deliberately small: unindexed vertex meshes, one shader, four pipeline
// states (triangles or lines, with or without depth test). The backends wait for the GPU at the end
// of every frame (one frame in flight): simple first, pipelined later.
#pragma once
#include <atomic>
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <vector>
#include "../platform/platform.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "../vecmath.h"
#include "../floating_origin.h"
#include "vertex.h"    // struct Vertex (its own header so platform-free code can build vertex lists)
#include "ui_vertex.h" // struct UiVertex, the smooth UI path's vertex (DrawUi)

// The open scene's fixed exposure, 0 = auto (Scene::exposure). Defined by the platform shell; the renderers read it.
extern float g_sceneExposure;

namespace tbd {

enum class Topology { Triangles, Lines };

constexpr uint32_t kShadowMapSize = 4096;      // the sun's shadow map is square
constexpr uint32_t kTightShadowMapSize = 2048; // so is the tight character map, which covers only a few meters

// Which shadow map a shadow pass renders into: the wide sun map, or the tight one around the camera's focus.
enum class ShadowMap { Sun, Tight };

// The baked city shadows' ring (Renderer::BakeCityShadow; fly/city_shadow.h, shaders/city_shadow.hlsl and city_shadow.hlsli use the
// same numbers): texels across, the block it is drawn and baked in, metres per texel, and how a height is stored as depth.
constexpr uint32_t kCityShadowSize = 2560, kCityShadowBlock = 128;
constexpr float kCityShadowCell = 2.0f, kCityShadowTop = 1500.0f, kCityShadowFar = 2000.0f;

// The scene's light rig, uploaded once per frame; mirrors LightingData in shaders/lighting.hlsli. Colors are
// already multiplied by their intensity. The defaults are the engine's original fixed light, unshadowed.
struct Lighting {
    float sunDir[4] = {0.4397f, 0.8305f, 0.3420f, 0.0f};      // xyz: unit vector toward the sun
    float sunColor[4] = {0.85f, 0.816f, 0.748f, 0.0f};        // rgb; zero when the sun is down
    float skyColor[4] = {0.378f, 0.468f, 0.612f, 0.0f};       // rgb: ambient light from straight up
    float groundColor[4] = {0.234f, 0.198f, 0.162f, 0.0f};    // rgb: ambient light from straight down
    float shadowVP[16] = {};                                  // world -> the sun's shadow map clip space
    float shadowParams[4] = {0.0f, 1.0f / kShadowMapSize, 0.0f, 0.0f}; // x: metres the depth spans, y: texel in uv, z: normal offset (world), w: on
    float eye[4] = {};                                        // xyz: camera position, for specular
    float tightVP[16] = {};                                   // world -> the tight character map's clip space
    float tightParams[4] = {0.0f, 1.0f / kTightShadowMapSize, 0.0f, 0.0f}; // x: metres the depth spans, y: texel in uv, z: normal offset (world), w: on
    float skySh[4][4] = {};    // sky and ground radiance, first order: mean rgb, then its rgb gradient along x, y, z
    float skyGrad[3][4] = {};  // rgb radiance for reflections: zenith, horizon, ground
    float probeOrigin[4] = {}; // irradiance probes: xyz grid cell of the lowest corner, w spacing across
    float probeGrid[4] = {};   // cells x, y, z; w spacing up
    float probeUpdate[4] = {}; // first probe this frame, probes per frame, rays per probe, on
    float probePrev[4] = {};   // last frame's probeOrigin
    float lightDebug[4] = {};  // x: which lighting component to view (TBD_LIGHTDEBUG); 0 = the final image
    float cloudMap[4] = {};    // cloud shadow map: centre x, z, half extent (m), on
    float captureVP[16] = {};  // world -> the lit capture from the sun (BeginCapture); element 15 is 0 until one exists
};
static_assert(sizeof(Lighting) == 512, "backends and lighting.hlsli hard-code this layout");

// The irradiance probe grid (DDGI-style, see post.hlsl): cells around the camera's focus, updated a slice at a time.
constexpr uint32_t kProbeX = 24, kProbeY = 8, kProbeZ = 24;
constexpr uint32_t kProbeRays = 16, kProbesPerFrame = 192;
constexpr float kProbeSpacing = 4.0f, kProbeSpacingY = 3.0f;
constexpr uint32_t kProbeCount = kProbeX * kProbeY * kProbeZ; // in each of the two cascades; the second is four times as wide (probe.hlsli derives its origin)
constexpr uint32_t kProbeCascades = 2, kProbeTotal = kProbeCount * kProbeCascades, kProbeAtlasH = kProbeZ * kProbeY * kProbeCascades;
constexpr uint32_t kCaptureSize = 512; // the lit capture from the sun that probe rays read: fits inside the scene's smallest depth buffer
constexpr uint32_t kCloudShadowSize = 256;
constexpr float kCloudShadowExtent = 15000.0f; // half width in metres

// The lighting fields the post passes read; mirrors the tail of PostParams in post.hlsl.
struct PostLightBlock {
    float shadowVP[16], sunColor[4], skySh[4][4], skyGrad[3][4], probeOrigin[4], probeGrid[4], probeUpdate[4], probePrev[4], lightDebug[4], cloudMap[4], captureVP[16];
};
static_assert(sizeof(PostLightBlock) == 352, "post.hlsl hard-codes this layout");
// `age`: frames since the probe atlas was made. lightDebug y: atlas just made, z: first pass over all probes, w: sun map valid.
inline void FillPostLight(const Lighting& l, uint32_t age, PostLightBlock& b) {
    std::copy(l.shadowVP, l.shadowVP + 16, b.shadowVP);
    std::copy(l.sunColor, l.sunColor + 4, b.sunColor);
    std::copy(&l.skySh[0][0], &l.skySh[0][0] + 16, &b.skySh[0][0]);
    std::copy(&l.skyGrad[0][0], &l.skyGrad[0][0] + 12, &b.skyGrad[0][0]);
    std::copy(l.probeOrigin, l.probeOrigin + 4, b.probeOrigin);
    std::copy(l.probeGrid, l.probeGrid + 4, b.probeGrid);
    std::copy(l.probeUpdate, l.probeUpdate + 4, b.probeUpdate);
    std::copy(l.probePrev, l.probePrev + 4, b.probePrev);
    std::copy(l.lightDebug, l.lightDebug + 4, b.lightDebug);
    std::copy(l.cloudMap, l.cloudMap + 4, b.cloudMap);
    std::copy(l.captureVP, l.captureVP + 16, b.captureVP);
    b.probeUpdate[0] = (float)(((uint64_t)age * kProbesPerFrame) % kProbeTotal);
    b.lightDebug[1] = age == 0 ? 1.0f : 0.0f;
    b.lightDebug[2] = (uint64_t)age * kProbesPerFrame < kProbeTotal ? 1.0f : 0.0f;
    b.lightDebug[3] = l.shadowParams[3];
}

// A window pane the probe rays can leave a shaded room through: world-space centre and the rectangle's two half-extent vectors.
struct WindowPortal {
    float center[3], u[3], v[3];
};
constexpr uint32_t kMaxPortals = 24;
// The inside of a building, as a box the probe rays inside it trace exactly: world-space centre, half height, and the two
// horizontal half-extent vectors (window_portals.h finds it from the building's own floor, ceiling and inner wall faces).
struct InteriorRoom {
    float center[3], halfHeight, x[3], z[3];
};
constexpr uint32_t kMaxRooms = 6;
// Tail of PostParams in post.hlsl: x of count is the number of panes, y the number of rooms; three float4 per pane (centre, u, v),
// then three per room (centre and half height, x, z).
struct PortalBlock {
    float count[4];
    float data[kMaxPortals * 3][4];
    float rooms[kMaxRooms * 3][4];
};
static_assert(sizeof(PortalBlock) == 16 + kMaxPortals * 48 + kMaxRooms * 48, "post.hlsl hard-codes this layout");
constexpr size_t kLightBufferSize = sizeof(Lighting) + sizeof(PortalBlock); // the mesh passes' constants: Lighting, then the panes (LightingData in lighting.hlsli)
// The last panes' slots carry the ground's look instead: data[68] the farmland style (SetGroundStyle), data[69..71] the imagery's
// look and mapping (SetGroundImagery). So at most kMaxPortals - 2 panes.
// data[67] the world atlas's origin offset (SetWorldTexOrigin).
constexpr uint32_t kGroundStyleSlot = kMaxPortals * 3 - 4, kImagerySlot = kMaxPortals * 3 - 3, kWorldTexSlot = kMaxPortals * 3 - 5;
inline void FillPortals(const WindowPortal* p, uint32_t n, PortalBlock& b) {
    n = std::min(n, kMaxPortals - 2);
    std::fill(&b.data[0][0], &b.data[kWorldTexSlot][0], 0.0f);
    b.count[0] = (float)n;
    for (uint32_t k = 0; k < n; ++k) {
        for (int c = 0; c < 3; ++c) { b.data[k * 3][c] = p[k].center[c]; b.data[k * 3 + 1][c] = p[k].u[c]; b.data[k * 3 + 2][c] = p[k].v[c]; }
    }
}
inline void FillRooms(const InteriorRoom* r, uint32_t n, PortalBlock& b) {
    n = std::min(n, kMaxRooms);
    std::fill(&b.rooms[0][0], &b.rooms[0][0] + sizeof(b.rooms) / sizeof(float), 0.0f);
    b.count[1] = (float)n;
    for (uint32_t k = 0; k < n; ++k) {
        for (int c = 0; c < 3; ++c) { b.rooms[k * 3][c] = r[k].center[c]; b.rooms[k * 3 + 1][c] = r[k].x[c]; b.rooms[k * 3 + 2][c] = r[k].z[c]; }
        b.rooms[k * 3][3] = r[k].halfHeight;
    }
}

// What the post pipeline needs to know about the camera.
struct FrameView {
    Mat4 viewProj;     // unjittered
    Mat4 invJittered;  // inverse of the matrix the scene is actually drawn with
    Vec3 eye;
    float time = 0.0f; // seconds, moves the clouds
    bool sky = true;   // physical sky, clouds and haze
    bool cityAir = false; // with sky off: the city camera's cheap analytic sky, height fog and grade (post.hlsl CityComposite, CityGrade)
    int postTier = 4;  // graphics preset for the post passes: 0 Low (no AO, no bloom), 1 Low/Med (no bloom), 2 Medium, 3 High, 4 Ultra / PC
    float dt = 0.0f;   // seconds since last frame, for what animates in a vertex shader
    bool motionVectors = true; // TAA follows moving things by their own motion
    // The air (post.hlsl CompositePS), when air[3] > 0: x aerial-perspective distance scale (1.2 = clear desert air), y height-fog
    // extinction per metre at sea level (y = 0), z the fog's scale height in metres. air[3] == 0 keeps the built-in defaults.
    float air[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    // The air's colour, when air2[3] > 0: rgb (linear) the hue the sky and the haze lean to, [3] how far (an overcast or hazy day).
    float air2[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    // Weather over the ground (the cloud shadow pass): shade[0] how many sunGap are in use, [1] cloud cover 0..1 (everything shaded that
    // much), [2] gap edge softness 0..1. [3] how much of the sky clears of cloud (0 the usual sky, 1 none). sunGap: x, z centre in scene (floating origin) coordinates, radius in metres.
    float shade[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float sunGap[8][4] = {};
    // The far distance (post pass): [0] haze brightness (0 = unchanged), [1] sky brightness at the horizon (0 = unchanged), [2] how high
    // above the horizon that reaches (direction y), [3] extra haze tint toward air2's colour 0..1.
    float horizon[4] = {0.0f, 0.0f, 0.0f, 0.0f};
};

using MeshId = uint32_t;
constexpr MeshId kInvalidMesh = 0xFFFFFFFFu;

using TextureId = uint32_t;
constexpr TextureId kNoTexture = 0xFFFFFFFFu; // "use the default": white albedo, flat normal

using InstanceBufferId = uint32_t;
constexpr InstanceBufferId kNoInstances = 0xFFFFFFFFu;

using GpuBufferId = uint32_t;
constexpr GpuBufferId kNoGpuBuffer = 0xFFFFFFFFu;
using ComputeShaderId = uint32_t;
constexpr ComputeShaderId kNoComputeShader = 0xFFFFFFFFu;
enum class GpuBufferUse { Read, Instances, Args, ShaderData }; // ShaderData: written by compute, read by vertex and pixel shaders

// What ResolveScene needs to draw water, right after the sky and haze are composed; see water.h and shaders/water.hlsl.
struct WaterDraw {
    const MeshId* meshes = nullptr;
    uint32_t meshCount = 0;
    uint32_t nearCount = 0; // the first nearCount meshes are the dense one(s) around the camera
    GpuBufferId waves = kNoGpuBuffer, ripple = kNoGpuBuffer, sky = kNoGpuBuffer;
    Mat4 viewProj;   // jittered, as the scene was drawn
    float ripplePlace[4] = {}, nearPlace[4] = {}, tiles[4] = {}, wind[4] = {}; // wind: the frame's jitter in clip space x, y; displacement scale; debug
    // 1: shade the water once per 2 x 2 pixels where the GPU can (variable rate shading: D3D12 tier 1, Vulkan's fragment shading rate);
    // depth, coverage and edges stay full resolution. Backends without it draw at full rate.
    uint32_t coarse = 0; // 2: once per 4 x 4 where the GPU has the extra rates (else 2 x 2)
    // 1: the copy-free pipeline (water.hlsl WaterVSFlight / WaterPSBlend): the water blends over the scene (scene x transmittance +
    // light, dual-source blending), no copy of the picture or the depth, no velocity. Backends without it draw the usual way.
    uint32_t blend = 0; // 2: the same with the Quest look's shaders (WaterVSQuest / WaterPSQuest)
    // 1: mirror the compute buffers into textures for hardware-filtered sampling (water.hlsl t8-t12): the waves buffer laid out planar
    // (per cascade three planes of waveSize^2 float4), the sky buffer skySize[0] x skySize[1] float4; foam: a tiling RGBA8 texture.
    uint32_t textures = 0;
    uint32_t waveSize[3] = {};
    uint32_t skySize[2] = {};
    TextureId foam = kNoTexture;
    // rivers and lakes: flat water meshes at their own level (water.hlsl WaterVSRiver / WaterPSRiver, the copy-free blend), each
    // drawn at riverOffsets[3 i .. 3 i + 2] (render space). Drawn after the sea; with no sea meshes the pass copies nothing.
    const MeshId* rivers = nullptr;
    const float* riverOffsets = nullptr;
    uint32_t riverCount = 0;
};

// What the scene drew this frame (draws flagged `counted` only; editor overlays are not counted).
struct FrameCounts {
    uint32_t draws = 0;
    uint64_t triangles = 0;
    uint32_t shadowDraws = 0;     // every shadow map's casters, instanced ones counted once per draw
    uint64_t shadowTriangles = 0;
    uint32_t materialBinds = 0;   // SetMaterial calls that reached the GPU
};

// Render scale: the scene and post chain draw at window * scale and the tone map upscales to the window. One clamp and one
// rounding rule, shared so both backends and main.cpp's TAA jitter agree on the internal size.
inline float ClampRenderScale(float scale) { return scale != scale ? 1.0f : std::min(1.0f, std::max(0.5f, scale)); }
inline int ScaledSize(int window, float scale) { return std::max(1, (int)std::lround((double)window * scale)); }

// One static mesh placed in the hardware ray tracing scene: a float3x4 model matrix row by row, as instance buffers hold it.
struct RayInstance {
    MeshId mesh;
    float rows[12];
};
constexpr uint32_t kMaxRayInstances = 16384;

class Renderer {
public:
    virtual ~Renderer() = default;

    virtual const char* Name() const = 0;
    // `window` becomes the render target; it must not be used by another graphics API.
    virtual bool Init(void* nativeWindow, int width, int height, bool validation) = 0; // Windows: HWND. Apple: CAMetalLayer*
    virtual void Resize(int width, int height) = 0;
    // Internal resolution as a fraction of the window (clamped 0.5..1.0, see ScaledSize). Call outside BeginFrame..EndFrame;
    // before Init it is only remembered. Targets are made again when it changes.
    virtual void SetRenderScale(float scale) = 0;

    // ---- XR (OpenXR) support. Vulkan backend only so far (PCVR now, Quest later); other backends ignore these.
    // Hooks that let the XR runtime create the Vulkan objects (XR_KHR_vulkan_enable2). Set before Init. Handles are passed as
    // void* so this header stays free of Vulkan types; the int results are VkResult values.
    struct XrVulkanBridge {
        std::function<int(const void* vkInstanceCreateInfo, void** outInstance)> createInstance;
        std::function<void*(void* instance)> physicalDevice; // the GPU the headset is attached to
        std::function<int(void* physicalDevice, const void* vkDeviceCreateInfo, void** outDevice)> createDevice;
    };
    virtual void SetXrBridge(const XrVulkanBridge* bridge) { (void)bridge; }
    // Init with nativeWindow == nullptr renders offscreen into the renderer's own images at width x height (no window, no
    // present). Resize changes that size.
    // The image the next EndFrame copies the finished frame into (a VkImage of the same size, size-compatible format, e.g.
    // an XR swapchain image), left in COLOR_ATTACHMENT_OPTIMAL as OpenXR expects. 0 = none. Offscreen mode only.
    virtual void SetEndFrameCopyTarget(uint64_t nativeImage) { (void)nativeImage; }
    // Temporal reuse of last frame (TAA history). Off for stereo: alternating eyes would blend into each other.
    virtual void SetTemporalHistory(bool on) { (void)on; }
    // The native objects an XR graphics binding needs (Vulkan: VkInstance, VkPhysicalDevice, VkDevice, queue family, queue index).
    virtual bool NativeHandles(void** instance, void** physicalDevice, void** device, uint32_t* queueFamily, uint32_t* queueIndex) const {
        (void)instance; (void)physicalDevice; (void)device; (void)queueFamily; (void)queueIndex;
        return false;
    }

    // Uploads vertices and returns a handle. The mesh lives until the renderer is destroyed.
    // Meshes live welded and indexed in GPU memory. `dynamic` is for geometry replaced often (a live face rig): it
    // skips welding and the wait for the copy, and stays in memory the CPU can write.
    virtual MeshId CreateMesh(const Vertex* vertices, uint32_t count, Topology topology, bool dynamic = false) = 0;
    // A mesh the CPU rewrites often (skinned characters): indexed, drawn from device-local memory, starting as `vertices`.
    // It holds its newest vertices and the ones before them, which motion vectors read. A frame that doesn't ask for its
    // vertices keeps both and costs nothing; ask once more after the last change, so the two agree and motion ends.
    virtual MeshId CreateDeformableMesh(const Vertex* vertices, uint32_t vertexCount, const uint32_t* indices, uint32_t indexCount) = 0;
    // A draw of part of a deformable mesh's index range, sharing its vertices. Destroy it before its parent.
    virtual MeshId CreateMeshPart(MeshId parent, uint32_t firstIndex, uint32_t indexCount) = 0;
    // Staging memory for this frame's new vertices of a deformable mesh, write-only (uncached), to be filled whole before
    // EndFrame; nullptr when no frame is open or the mesh is not deformable. Call on the render thread, between BeginFrame
    // and CommitDeformables; any thread may write through the pointer.
    virtual Vertex* DeformableVertices(MeshId mesh) = 0;
    // Records the copies of what DeformableVertices handed out this frame. Before the frame's first draw.
    virtual void CommitDeformables() = 0;
    // Frees a mesh (streamed terrain chunks come and go). Safe between frames; the id may be reused.
    virtual void DestroyMesh(MeshId mesh) = 0;

    // Per-instance model matrices in device-local memory: `count` rows of 12 floats, a float3x4 row by row (x axis, y axis,
    // z axis and translation are the columns). Waits for the copy; call outside BeginFrame..EndFrame.
    virtual InstanceBufferId CreateInstanceBuffer(const float* rows, uint32_t count) = 0;
    virtual void DestroyInstanceBuffer(InstanceBufferId buffer) = 0;
    // Like DestroyInstanceBuffer, without waiting for the GPU: the memory goes once no frame in flight can read it (the id may be
    // reused at once). Backends without a retire queue wait instead.
    virtual void RetireInstanceBuffer(InstanceBufferId buffer) { DestroyInstanceBuffer(buffer); }

    // Material textures. `rgba` is
    // width*height*4 bytes, top row first; mip levels are generated. Call outside BeginFrame..EndFrame.
    virtual TextureId CreateTexture(const uint8_t* rgba, int width, int height) = 0;
    virtual void DestroyTexture(TextureId texture) = 0;
    // The albedo (multiplies the vertex color; alpha below 0.4 is cut out) and tangent-space normal map used by the
    // lit draws that follow, until changed. kNoTexture = default. Reset to (none, none) after drawing a textured mesh.
    // `surface` packs per-pixel skin data: R roughness (0.5 = neutral), G cavity (1 = open skin), B scatter amount.
    virtual void SetMaterial(TextureId albedo, TextureId normal, TextureId surface = kNoTexture) = 0;
    // Links a surface texture to its extra material map, bound with it by SetMaterial: R baked occlusion (1 = open), G reserved (1),
    // B metalness scale (1 = the material's own factor), A emissive strength (0 = none). kNoTexture unlinks; the default texel is
    // (255, 255, 255, 0). The link ends when either texture is destroyed. Call outside BeginFrame..EndFrame.
    virtual void SetSurfaceExtra(TextureId surface, TextureId extra) { (void)surface; (void)extra; }
    // Skin's pore normal (the normal map of a skin material, mesh.hlsl's skin branch) repeats this many times across the UV
    // atlas. Per material, keyed by its albedo texture: SetMaterial with that albedo uses it. Default kDefaultSkinDetailScale
    // (48, the old fixed value); 1 samples the normal map untiled. A scale <= 0 resets to the default. Forgotten when the
    // texture is destroyed. Any time, both backends.
    static constexpr float kDefaultSkinDetailScale = 48.0f;
    void SetSkinDetailScale(TextureId albedo, float scale) {
        if (albedo == kNoTexture) return;
        if (scale > 0.0f && scale != kDefaultSkinDetailScale) skinDetailScale_[albedo] = scale;
        else skinDetailScale_.erase(albedo);
    }
    float SkinDetailScale(TextureId albedo) const {
        const auto it = skinDetailScale_.find(albedo);
        return it == skinDetailScale_.end() ? kDefaultSkinDetailScale : it->second;
    }
    // How deep skin's pores read (the pit darkening and the normal tilt of mesh.hlsl's skin branch), per material keyed by its
    // albedo like SetSkinDetailScale: 1 = the default, 0.5 young and fine, 2 old and coarse; clamped 0..3. A depth < 0 resets.
    // Sent as materialConstants_.y (0 there means the default, so materials never set read as 1). Any time, both backends.
    void SetSkinPoreDepth(TextureId albedo, float depth) {
        if (albedo == kNoTexture) return;
        if (depth >= 0.0f && depth != 1.0f) skinPoreDepth_[albedo] = depth > 3.0f ? 3.0f : depth;
        else skinPoreDepth_.erase(albedo);
    }
    // An aircraft's skin (a painted cowl, a polished inlet lip), per material keyed by its albedo: its reflections see the real
    // environment, sky and clouds above a hazy horizon and the cloud-shadowed ground below (mesh.hlsl AirframeSky), and `coat`
    // 0..1 lays a clear lacquer over a painted base (a second, sharp specular lobe, as airliner paint is finished). Shares
    // materialConstants_.y with the pore depth (skin never is airframe): sent negative, -(1 + coat). coat < 0 resets.
    void SetAirframe(TextureId albedo, float coat) {
        if (albedo == kNoTexture) return;
        if (coat >= 0.0f) airframe_[albedo] = coat > 1.0f ? 1.0f : coat;
        else airframe_.erase(albedo);
    }
    float SkinPoreDepthConstant(TextureId albedo) const { // the value for the shader: 0 = default
        const auto af = airframe_.find(albedo);
        if (af != airframe_.end()) return -(1.0f + af->second);
        const auto it = skinPoreDepth_.find(albedo);
        return it == skinPoreDepth_.end() ? 0.0f : (it->second > 0.0f ? it->second : 1e-4f);
    }

    // World streaming: uploads that never wait for the GPU, and frees that wait until no frame in flight can use the resource.
    // The copy is queued ahead of the next frame, so the result can be drawn at once. Call outside BeginFrame..EndFrame.
    virtual MeshId CreateMeshNoWait(const Vertex* vertices, uint32_t vertexCount, const uint32_t* indices, uint32_t indexCount) = 0;
    virtual InstanceBufferId CreateInstanceBufferNoWait(const float* rows, uint32_t count) = 0;
    // A full mip chain made ahead of time; `dims[i]` is level i's size, each level width*height*4 bytes.
    virtual TextureId CreateTextureNoWait(const std::vector<std::vector<uint8_t>>& levels, const std::vector<std::pair<int, int>>& dims) = 0;
    virtual void RetireMesh(MeshId mesh) = 0;
    virtual void RetireTexture(TextureId texture) = 0;
    // The GPU's own memory in bytes; 0 when unknown.
    virtual uint64_t LocalMemoryBytes() const = 0;

    // ------------------------------------------------------------------ compute (GPU-driven placement)
    // A device-local buffer. Instances may also be a vertex stream; Args holds indirect draws (five uints per draw: index count,
    // instance count, first index, base vertex, first instance); both are writable by compute. Read is only ever read by it.
    virtual GpuBufferId CreateGpuBuffer(GpuBufferUse use, uint64_t bytes, const void* initial) = 0;
    virtual void DestroyGpuBuffer(GpuBufferId buffer) = 0;
    // An entry point of shaders/vegetation.hlsl; on Vulkan its SPIR-V is shaders/vegetation_<entry>.spv.
    virtual ComputeShaderId CreateComputeShader(const char* entry) = 0;
    // An entry point of shaders/<file>.hlsl (D3D12: compiled with WATER_COMPUTE defined for "water"); on Vulkan shaders/<file>_<entry>.spv.
    virtual ComputeShaderId CreateComputeShaderIn(const char* file, const char* entry) = 0;
    // Before ResolveScene: draws this water in it, this frame only. The arrays must live until then. False if its pipeline failed.
    virtual bool SetWaterDraw(const WaterDraw& draw) = 0;
    // Right after BeginFrame and before any shadow pass: every writable buffer stays writable until EndCompute.
    virtual void BeginCompute() = 0;
    // Four read buffers (t0-t3) and three written ones (u0-u2); constants are at most 128 bytes. Dispatches run in order.
    virtual void Dispatch(ComputeShaderId shader, const void* constants, uint32_t constantBytes, const GpuBufferId reads[4],
                          const GpuBufferId writes[3], uint32_t x, uint32_t y, uint32_t z) = 0;
    virtual void EndCompute() = 0;
    // A welded mesh's index count, for indirect arguments; 0 when it has no indices.
    virtual uint32_t MeshIndexCount(MeshId mesh) const = 0;
    // One indirect draw of a mesh over instances, with the vegetation vertex shader; `params` reach it as pc.model.
    virtual void DrawVegetationIndirect(MeshId mesh, GpuBufferId instances, GpuBufferId args, uint32_t argsOffset, const Mat4& viewProj, const Mat4& params) = 0;
    virtual void DrawVegetationShadowIndirect(MeshId mesh, GpuBufferId instances, GpuBufferId args, uint32_t argsOffset, const Mat4& params) = 0;
    // Which pipeline DrawVegetationIndirect uses: 0 opaque with alpha test, 1 alpha test into depth only, 2 shade against that depth.
    virtual void SetVegetationPass(int pass) = 0;
    // Debug: copies a buffer's start to the CPU, waiting for the GPU. Outside BeginFrame..EndFrame.
    virtual bool ReadGpuBuffer(GpuBufferId buffer, void* out, uint64_t bytes) = 0;

    virtual void BeginFrame(const float clearColor[4]) = 0;
    // Renders depth from the sun into a shadow map, using the shadowVP (Sun) or tightVP (Tight) of the lighting set before
    // BeginFrame. Call right after BeginFrame, before any other draw; casters are drawn in between. Casters are not counted.
    // Passes don't nest. When both maps are drawn, draw Tight first: the Sun pass's end closes the "shadow" GPU timer.
    // restoreSunCache: the Sun map starts as the depth last stored by StoreSunCache instead of cleared.
    virtual void BeginShadowPass(ShadowMap map, bool restoreSunCache = false) = 0;
    // Inside the Sun pass: keeps the depth drawn so far (the casters that don't move) under `key`, for later frames.
    virtual void StoreSunCache(uint64_t key) = 0;
    // Whether the stored depth is for `key`. Destroying a mesh forgets it, since a reused id may be other geometry.
    bool SunCacheHolds(uint64_t key) const { return key != 0 && sunCacheKey_ == key; }
    virtual void DrawShadowCaster(MeshId mesh, const Mat4& model) = 0;
    // A caster cut out by its albedo's alpha (hair cards): depth only where the texture has coverage, hashed so partial coverage
    // casts a partial shadow once the map's filter averages it. Binds `albedo` as the material.
    virtual void DrawShadowCasterCutout(MeshId mesh, const Mat4& model, TextureId albedo) = 0;
    // `count` instances of the mesh, from instance `first` of the buffer, into the current shadow pass.
    virtual void DrawShadowInstanced(MeshId mesh, InstanceBufferId buffer, uint32_t first, uint32_t count) = 0;
    virtual void EndShadowPass() = 0;
    // City shadows baked on the GPU (fly/city_shadow.h, shaders/city_shadow.hlsl): a ring buffer of kCityShadowSize texels round the
    // camera holding the town's height seen from straight above, and baked from it each texel's sunlit times of day. All calls between
    // BeginFrame and the scene's first draw; each restores the scene's targets. Unsupported backends ignore them (no baked shadows).
    virtual bool CityShadowSupported() const { return false; }
    // Clears ring texels [x, x+w) x [z, z+h) of the height map (when `clear`) and draws casters into them (DrawShadowCaster) through
    // `viewProj`: scene space to that rectangle's clip square, depth 0 at kCityShadowTop, 1 at kCityShadowTop - kCityShadowFar.
    virtual void BeginCityHeights(uint32_t x, uint32_t z, uint32_t w, uint32_t h, const Mat4& viewProj, bool clear) { (void)x; (void)z; (void)w; (void)h; (void)viewProj; (void)clear; }
    virtual void EndCityHeights() {}
    // Bakes the sunlit times into ring texels [x, x+w) x [z, z+h) from the height map; `c` is city_shadow.hlsl's BakeConstants.
    virtual void BakeCityShadow(uint32_t x, uint32_t z, uint32_t w, uint32_t h, const float c[8]) { (void)x; (void)z; (void)w; (void)h; (void)c; }
    // Forgets the baked times there (they read as no baked shadow until baked again).
    virtual void ClearCityShadow(uint32_t x, uint32_t z, uint32_t w, uint32_t h) { (void)x; (void)z; (void)w; (void)h; }
    // Where the ring lies and the time of day, for the lit draws (city_shadow.hlsli's constants). Before BeginFrame.
    virtual void SetCityShadowView(const float a[4], const float b[4]) { (void)a; (void)b; }
    // A small lit picture of the scene, drawn with Draw and matrices that reverse Z like the scene's, into a texture the probe rays read
    // (Dachsbacher and Stamminger's reflective shadow map, holding radiance). After the shadow passes, before the scene's draws.
    virtual void BeginCapture() = 0;
    virtual void EndCapture() = 0;
    uint32_t CaptureEpoch() const { return captureEpoch_; } // changes when the capture's texture was made anew and holds nothing
    // mvp = projection * view * model; model is used for lighting normals.
    virtual void Draw(MeshId mesh, const Mat4& mvp, const Mat4& model, bool depthTest, bool counted) = 0;
    // `count` instances of the mesh, from instance `first`, into the HDR scene with the current material. Always counted.
    virtual void DrawInstanced(MeshId mesh, InstanceBufferId buffer, uint32_t first, uint32_t count, const Mat4& viewProj) = 0;
    // Like DrawInstanced, with instance rows (kInstanceFloats each, as CreateInstanceBuffer takes them) valid for this
    // call only: copied into the per-frame transient buffer, so instances that move every frame (cosmetic traffic) need
    // no buffer of their own. Always counted.
    virtual void DrawInstancedTransient(MeshId mesh, const float* rows, uint32_t count, const Mat4& viewProj) = 0;
    // DrawInstancedTransient's twin for the current shadow pass (between BeginShadowPass and EndShadowPass): per-frame rows, no
    // buffer of their own. Casters are not counted as scene draws.
    virtual void DrawShadowInstancedTransient(MeshId mesh, const float* rows, uint32_t count) = 0;
    // ---- city chunk bakes (Track R, src/city_buildings.inl): a 128 m city cell drawn into five orthographic faces (top and four
    // sides; the table is shaders/chunk_faces.hlsli, render/chunk_faces.h) that ChunkVolPS ray marches from far away. Built on the
    // impostor store: a face is drawn into a store layer (with the store's tile depth buffer), then copied into the atlas, where a
    // cell owns a strip of five tile x tile texels. Three atlases, bound as the albedo, normal and surface textures of a material:
    //   channel 0  albedo + coverage  RGBA8: unlit, display space as the textures hold it; a = 1 where something was drawn
    //   channel 1  normal + AO        RGBA8: world normal * 0.5 + 0.5; a = occlusion (1 = open)
    //   channel 2  relief             RG16:  r = depth where the solid starts, g = 1 - depth where the last of it ends (0..1 along the
    //                                        face's view; empty texels hold r = 1, g = 1, so end < start)
    // CreateChunkAtlas: outside BeginFrame..EndFrame; again with another size frees the old one. False when unsupported.
    virtual bool CreateChunkAtlas(uint32_t tile, uint32_t width, uint32_t height) { (void)tile; (void)width; (void)height; return false; }
    virtual TextureId ChunkAtlasTexture(int channel) const { (void)channel; return kNoTexture; }
    // One mesh (a material part) and its instances, for ChunkBakeFace. Rows as DrawInstancedTransient takes them.
    struct ChunkBakeItem {
        MeshId mesh = kInvalidMesh;
        TextureId albedo = kNoTexture;
        const float* rows = nullptr;
        uint32_t count = 0;
    };
    // Bakes: inside a frame, after the shadow passes (and BeginCapture..EndCapture), before the scene's first draw. Between Begin and
    // End only ChunkBakeFace; End restores the scene's targets, cleared (as EndCapture does). Not counted as scene draws.
    virtual bool BeginChunkBake() { return false; }
    // One face: clears the store's tile, draws the items through `faceViewProj` (ChunkFaceMatrix), copies the tile to the atlas at
    // texel (atlasX, atlasY) in all three channels.
    virtual void ChunkBakeFace(const ChunkBakeItem* items, uint32_t count, const Mat4& faceViewProj, uint32_t atlasX, uint32_t atlasY) {
        (void)items; (void)count; (void)faceViewProj; (void)atlasX; (void)atlasY;
    }
    virtual void EndChunkBake() {}
    // Debug and the disk cache: a w x h block of one channel, 4 bytes a texel (RG16 as two little-endian uint16). Outside frames;
    // Read waits for the GPU, Write is queued ahead of the next frame.
    virtual bool ReadChunkAtlas(int channel, uint32_t x, uint32_t y, uint32_t w, uint32_t h, std::vector<uint8_t>& out) {
        (void)channel; (void)x; (void)y; (void)w; (void)h; (void)out; return false;
    }
    virtual bool WriteChunkAtlas(int channel, uint32_t x, uint32_t y, uint32_t w, uint32_t h, const uint8_t* texels) {
        (void)channel; (void)x; (void)y; (void)w; (void)h; (void)texels; return false;
    }
    // The chunk volumes (ChunkVolPS) into the HDR scene: `mesh` holds one box per cell, its vertices encoded as ChunkVolVS reads
    // them (position world; normal the box's low corner; rgb the box's size, a 0.852; uv the cell's strip in the atlas, texels).
    // Binds the atlas itself; afterwards the material is the default one. Counted.
    virtual void DrawChunkVolumes(MeshId mesh, const Mat4& viewProj) { (void)mesh; (void)viewProj; }
    // ---- "nacho" impostor cards (render/nacho.h bakes the atlas and packs the instances; shaders/nacho.hlsli draws them): one
    // camera-facing quad per instance that traces three slice planes into a six-view atlas and writes the surface's real depth.
    // `quad` is nacho.h's unit quad mesh; `buffer` holds NachoInstance rows (12 floats, as CreateInstanceBuffer takes them);
    // `albedo` / `normal` the atlas pair. All assets share one atlas, so a forest is one draw. params: fade near, fade far (metres;
    // far <= near = no fade: the card dithers in over that range, the mesh side uses SetDrawFade(-NachoFade(...))), gain on the
    // lighting, debug (0 off, 1 face colours, 2 unlit albedo). Counted. The shadow twin draws into the Sun map only (Tight is skipped).
    // Supported on Direct3D 12 and Vulkan; Metal draws nothing yet (NachoSupported false).
    virtual bool NachoSupported() const { return false; }
    // `surface`: what the material's third texture holds while the cards draw (the aerial imagery for buildings' roofs; city_nacho.h).
    virtual void DrawNachoCards(MeshId quad, InstanceBufferId buffer, uint32_t first, uint32_t count, const Mat4& viewProj, TextureId albedo,
                                TextureId normal, const float params[4], TextureId surface = kNoTexture) {
        (void)quad; (void)buffer; (void)first; (void)count; (void)viewProj; (void)albedo; (void)normal; (void)params; (void)surface;
    }
    virtual void DrawNachoShadow(MeshId quad, InstanceBufferId buffer, uint32_t first, uint32_t count, TextureId albedo, TextureId normal) {
        (void)quad; (void)buffer; (void)first; (void)count; (void)albedo; (void)normal;
    }
    // A screen-door fade for the lit scene draws that follow, until changed (Track R's dithered LOD transitions): keep in (0, 1)
    // shows the pixels whose dither value lies below keep, keep in (-1, 0) the ones at or above -keep (the complement: an old and a
    // new representation with keep t and -t never overlap and leave no holes), 0 or 1 no fade. Sent with the material's constants (w).
    virtual void SetDrawFade(float keep) { materialConstants_[3] = keep; }
    // Vertices valid for this call only (copied into a per-frame buffer).
    virtual void DrawTransient(const Vertex* vertices, uint32_t count, Topology topology, const Mat4& mvp,
                               const Mat4& model, bool depthTest) = 0;
    // The sky comes from ResolveScene; draw the scene with a sub-pixel jittered matrix for temporal anti-aliasing.
    // ResolveScene ends the HDR scene: draws after it are display-space overlays.
    // ---- smooth UI (src/gui, shaders/ui.hlsl, render/ui_vertex.h): its own pipeline with real alpha blending and no depth
    // test, for the in-game GUI. Rounded boxes, shadows and capsules by signed distance, text from an SDF atlas.
    // UiSupported() is false when the pipeline did not build (shader missing or failed): callers then use DrawTransient.
    virtual bool UiSupported() const { return false; }
    // The SDF font atlas (a texture from CreateTexture) the text mode samples. Set once after Init; kNoTexture = none.
    virtual void SetUiAtlas(TextureId atlas) { (void)atlas; }
    // Overlay phase only (after ResolveScene), like DrawTransient. ortho = OrthoPixels(width, height).
    virtual void DrawUi(const UiVertex* vertices, uint32_t count, const Mat4& ortho) { (void)vertices; (void)count; (void)ortho; }
    // The RGBA sprite atlas UiMode::Image samples (icons, tile previews; a texture from CreateTexture). kNoTexture = none.
    virtual void SetUiImages(TextureId atlas) { (void)atlas; }
    // Overlay phase, before the DrawUi calls that use UiMode::Glass: copies what is on screen now into the shared backdrop,
    // box-filtered down to 1/16 of the viewport, which Glass quads blur and tint (one copy and a tiny texture per frame,
    // whatever the number of glass panels). Without a call this frame, glass samples last frame's (or a black) backdrop.
    virtual void CaptureUiBackdrop() {}

    virtual void SetFrameView(const FrameView& view) = 0;
    virtual void ResolveScene() = 0;
    virtual void EndFrame() = 0; // submits, presents, waits
    // Debug: while on, EndFrame keeps a copy of the frame it presented (BGRA bytes).
    virtual void SetFrameCapture(bool on) = 0;
    // Rebuilds the shaders from source without a restart; false when unsupported or a shader failed to compile (the old ones stay).
    virtual bool ReloadShaders() { return false; }
    virtual bool CapturedFrame(std::vector<uint8_t>& rgba, int& width, int& height) = 0;

    // Call between frames when the world origin moves. Re-express the previous
    // unjittered projection in the new coordinates for TAA and motion vectors.
    void ShiftPreviousView(Vec3 offset) { prevViewProj_ = ShiftPreviousViewProjection(prevViewProj_, offset); }

    // Takes effect at the next BeginFrame and stays until changed.
    void SetLighting(const Lighting& lighting) { lighting_ = lighting; }
    // The panes the irradiance probes treat as openings this frame (at most kMaxPortals; none = every probe behaves as before).
    void SetWindowPortals(const WindowPortal* panes, uint32_t count) { FillPortals(panes, count, portals_); }
    // Satellite imagery under the ground (mesh.hlsl GroundImagery): bind its texture as the surface map of the ground draws, and say
    // here, before BeginFrame, how scene x, z map to its uv: u = a[0] + a[2] x + a[3] z, v = a[1] + b[0] x + b[1] z; b[2] strength
    // (0 = none), b[3] gain on its linear colour; look: [0] how much of its hue grass takes (the rest keeps the procedural hue at the
    // imagery's brightness), [1] how much pavement follows its brightness, [2] [3] metres from the eye over which it fades in.
    // Rides in the mesh passes' constants (the last pane's slots).
    // Farmland (mesh.hlsl FarmField): x, z the floating origin modulo fly/farm.h kPeriod, the layout style (farm::Style), the day of
    // the year. Before BeginFrame.
    // The world material atlas (mesh.hlsl WorldTexPos): the floating origin modulo 240 m (x, y, z), so the photographs tiled from the
    // scene position stay fixed to the world. Before BeginFrame.
    void SetWorldTexOrigin(const float s[4]) { for (int c = 0; c < 4; ++c) portals_.data[kWorldTexSlot][c] = s[c]; }
    void SetGroundStyle(const float s[4]) { for (int c = 0; c < 4; ++c) portals_.data[kGroundStyleSlot][c] = s[c]; }
    void SetGroundImagery(const float a[4], const float b[4], const float look[4]) {
        for (int c = 0; c < 4; ++c) {
            portals_.data[kImagerySlot][c] = look[c]; portals_.data[kImagerySlot + 1][c] = a[c]; portals_.data[kImagerySlot + 2][c] = b[c];
        }
    }
    // The building interiors whose probes trace their walls, floor and ceiling (at most kMaxRooms).
    void SetInteriorRooms(const InteriorRoom* rooms, uint32_t count) { FillRooms(rooms, count, portals_); }
    // Two local lamps in the last room's slots (so at most kMaxRooms - 1 rooms with them): dome xyz+intensity, flood xyz+intensity,
    // flood aim. Call after SetInteriorRooms, which clears them. See CockpitLights in lighting.hlsli.
    void SetCockpitLights(const float dome[4], const float flood[4], const float aim[3]) {
        for (int c = 0; c < 4; ++c) { portals_.rooms[15][c] = dome[c]; portals_.rooms[16][c] = flood[c]; }
        for (int c = 0; c < 3; ++c) portals_.rooms[17][c] = aim[c];
    }

    // Hardware ray tracing (DXR 1.1 ray queries on Direct3D 12, VK_KHR_ray_query on Vulkan): the ambient occlusion pass traces
    // real rays against the static meshes given here, so objects shade the light around them (a barrel the floor it stands on).
    virtual bool RayTracingSupported() const { return false; }
    // What the rays can hit next frame (at most kMaxRayInstances; triangle meshes made by CreateMesh / CreateMeshNoWait). Each
    // mesh gets its own acceleration structure the first time it is listed, a few a frame; until then it is left out.
    void SetRayScene(const RayInstance* instances, uint32_t count) { rayScene_.assign(instances, instances + std::min(count, kMaxRayInstances)); }

    // The scene's depth pre-pass, for Draw and DrawInstanced of lit triangles: 1 = depth only (no pixel shader, nothing counted),
    // 2 = shade what the pre-pass drew (depth equal, no depth write), 0 = as normal. Only parts that never cut out or discard
    // may be drawn in mode 1, with the same matrices as in mode 2. Skinned meshes ignore it (mode 1 skips them).
    virtual void SetScenePrepass(int mode) { (void)mode; }
    // A GPU timestamp inside the scene pass, ending the stretch named here (the first one starts at the shadow passes' end).
    // The D3D12 renderer logs a 60-frame mean of each name after its GPU frame line ("scene parts:"); others ignore it.
    virtual void GpuSubMark(const char* name) { (void)name; }

    FrameCounts Counts() const { return counts_; }
    // Diagnostics (TBD_FRAMELOG): the newest finished frame's GPU passes in ms (shadow, scene, ao, clouds, composite, taa,
    // tonemap, overlays; that frame is kFramesInFlight old), and CPU ms of this frame's fence wait and Present.
    const double* GpuPasses() const { return gpuPasses_; }
    double FenceWaitMs() const { return fenceWaitMs_; }
    double PresentMs() const { return presentMs_; }

protected:
    Mat4 prevViewProj_ = Mat4::Identity();
    FrameCounts counts_, lastCounts_; // lastCounts_: the previous frame's, for the timer log
    std::unordered_map<TextureId, float> skinDetailScale_; // SetSkinDetailScale; only the ones off the default
    std::unordered_map<TextureId, float> skinPoreDepth_;   // SetSkinPoreDepth; only the ones off the default
    std::unordered_map<TextureId, float> airframe_;        // SetAirframe: coat 0..1
    // The bound material's constants for the pixel shader (D3D12 root constants b3, Vulkan push constants at offset 128):
    // x skin detail scale, y skin pore depth (0 = default 1), z the chunk tile (DrawChunkVolumes), w the fade (SetDrawFade). Set by SetMaterial, sent with every lit draw.
    float materialConstants_[4] = {kDefaultSkinDetailScale, 0.0f, 0.0f, 0.0f};
    void ForgetMaterialConstants(TextureId id) { skinDetailScale_.erase(id); skinPoreDepth_.erase(id); airframe_.erase(id); }
    uint64_t sunCacheKey_ = 0;
    // The line after each 60-frame GPU timer line: what the last frame drew.
    void LogCounts(const char* api) const; // log.cpp
    double gpuPasses_[10] = {}; // the timer marks less one (kTimerMarks in both renderers); 8 overran into the members below
    double fenceWaitMs_ = 0.0, presentMs_ = 0.0;
    static double TraceNowMs() {
        int64_t now, freq;
        now = tbd::platform::Ticks();
        freq = tbd::platform::TickFrequency();
        return (double)now * 1000.0 / (double)freq;
    }
    // Scoped step timer for the streaming upload calls: Mark() after each step; Done() logs the steps when the call was slow.
    struct UploadSteps {
        const char* name[10] = {};
        double at[10] = {};
        int n = 0;
        double start = TraceNowMs();
        void Mark(const char* step) { if (n < 10) { name[n] = step; at[n++] = TraceNowMs(); } }
        void Done(const char* api, const char* what, uint64_t bytes, double slowMs = 8.0);
    };
    Lighting lighting_;
    PortalBlock portals_ = {};
    uint32_t captureEpoch_ = 0;
    std::vector<RayInstance> rayScene_;
};

std::unique_ptr<Renderer> CreateD3D12Renderer();
// Fills the D3D12 shader cache on all cores; blocks. Run it off the main thread so the window stays alive.
void PrecompileD3D12Shaders(std::atomic<int>& done, std::atomic<int>& total, const std::atomic<bool>& stop, std::atomic<int>* failed = nullptr);
std::unique_ptr<Renderer> CreateVulkanRenderer();
// Apple only (renderer_metal.mm): Metal via metal-cpp. Init's window is a CAMetalLayer*.
std::unique_ptr<Renderer> CreateMetalRenderer();

// Appends to engine.log next to the executable (and the debugger output).
void Log(const char* format, ...);
// Crash reporting: crash_<time>.log (stack, registers, modules, last log lines) and crash_<time>.dmp beside the executable,
// for unhandled exceptions, std::terminate, abort, stack overflow and CRT errors. MarkCleanExit() tells the log the exit was normal.
void InstallCrashHandlers();
void MarkCleanExit();

// Folder holding the running executable (shaders live in <it>\shaders).
std::wstring ExeDirectory();

} // namespace tbd
