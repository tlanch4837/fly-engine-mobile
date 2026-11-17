# Fly-engine mobile

The lean "basic" build of Fly-engine, for the iPhone first and the Meta Quest 3 later. It keeps only what holds a phone or
headset frame budget:

- generated terrain, with streamed satellite imagery draped over all of it
- baked airports: runways from OpenStreetMap as flat strips with markings, on flattened ground (`data/airports/*.json`)
- the trainer aircraft, with chase, cockpit and orbit cameras, and a small HUD

There are no towns, buildings, trees, ground vehicles, cockpit model or water system. The plan is to add features back one at a
time, only while the frame budget holds. First measurement (laptop, D3D12, 1600x900): about 3 ms GPU, 67-80 draws, 0.3M triangles.

## Build the iPhone app (Codemagic)

`codemagic.yaml` has two workflows. In Codemagic, add this repository with the project type **Other**.

- **ios-check**: an unsigned Release build, run on every push. It proves the code and the Metal shaders compile. No Apple
  account needed.
- **ios-testflight**: a signed build uploaded to TestFlight, to install on the phone. It needs a paid Apple Developer account
  and these, set up once:
  1. In App Store Connect: register the bundle id `com.ovoa.flybasic` and create an app record for it.
  2. In Codemagic, Teams > Integrations > Developer Portal: add an App Store Connect API key named `FLY_ASC_KEY`.
  3. In Codemagic, an environment group `apple` with `APPLE_TEAM_ID` (your 10-character team id).

On a Mac instead:

```
cmake -S . -B build-ios -G Xcode -DCMAKE_SYSTEM_NAME=iOS -DCMAKE_OSX_SYSROOT=iphoneos -DCMAKE_OSX_DEPLOYMENT_TARGET=16.0 -DFLY_TEAM_ID=XXXXXXXXXX
open build-ios/FlyEngine.xcodeproj
```

## Build and run on Windows

`build_basic.bat` (Visual Studio 2022) makes `bin\fly_basic.exe`. The first run compiles the shaders, which takes a few minutes;
after that they are cached.

Env switches: `FLY_AIRPORT=KDTW`, `FLY_RUNWAY=22R`, `FLY_SPAWN=approach|runway`, `FLY_FOG=0|1|2`, `FLY_POST_TIER=0..4`,
`FLY_SAT=0` (no imagery), `FLY_HUD=0`.

Keys: arrows pitch and roll, A / D rudder, W / S throttle, C camera, R reset, F fog. An Xbox pad works too.

## Shaders

The HLSL in `src/shaders` is the source. Direct3D 12 compiles it at startup. The iPhone uses the committed Metal versions in
`src/shaders/msl`: after any HLSL change, run `msl.bat` on Windows (needs the Vulkan SDK: HLSL to SPIR-V with dxc, then Metal
with spirv-cross) and commit the result.

## Adding an airport

```
python tools/bake_airport.py KJFK
```

This fetches the runways once from the Overpass API into `data/airports/kjfk.json`, which the app bundles. Pick it with
`FLY_AIRPORT=KJFK`.

## Data and licences

- Runways: OpenStreetMap contributors (ODbL).
- Places: GeoNames (CC BY 4.0). Land mask and coastlines: Natural Earth (public domain).
- Satellite imagery is streamed at run time from USGS The National Map (public domain, US) and EOX Sentinel-2 cloudless
  (CC BY 4.0). A shipping build must point `FLY_SAT_URL` at our own copies of the tiles, not at these public servers.
- UI font: DejaVu (see `data/ui/LICENSE_DejaVu.txt`).
