# Fly-engine mobile: the lean basic game (terrain, satellite imagery, baked airports; no 3D) for iPhone first, then Quest 3.
#
#   iOS (Metal), on a Mac with Xcode, or on Codemagic (codemagic.yaml):
#     cmake -S . -B build-ios -G Xcode -DCMAKE_SYSTEM_NAME=iOS -DCMAKE_OSX_SYSROOT=iphoneos -DCMAKE_OSX_DEPLOYMENT_TARGET=16.0
#     cmake --build build-ios --config Release -- -sdk iphoneos CODE_SIGNING_ALLOWED=NO
#   -DFLY_TEAM_ID=XXXXXXXXXX signs it. The Metal shaders are the committed src/shaders/msl/*.metal, regenerated on Windows by
#   msl.bat whenever src/shaders changes.
#
#   Windows (MSVC, Direct3D 12 + Vulkan), for quick testing: build_basic.bat, or
#     cmake -S . -B build-win -G "Visual Studio 17 2022" -A x64 && cmake --build build-win --config Release
cmake_minimum_required(VERSION 3.28)

if(CMAKE_SYSTEM_NAME STREQUAL "iOS")
    set(CMAKE_OSX_SYSROOT iphoneos CACHE STRING "iOS SDK")
    set(CMAKE_OSX_DEPLOYMENT_TARGET 16.0 CACHE STRING "Minimum iOS version")
    project(FlyEngine VERSION 0.1.0 LANGUAGES C CXX OBJC OBJCXX)
else()
    project(FlyEngine VERSION 0.1.0 LANGUAGES C CXX)
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- portable code: the game, the Optima generators (headers), and the shared renderer helpers
set(FLY_CORE_SOURCES
    src/fly/game.cpp
)

if(WIN32)
    add_executable(fly_engine WIN32
        ${FLY_CORE_SOURCES}
        src/platform/win/main_win.cpp
        src/platform/platform_win.cpp
        src/render/log.cpp
        src/render/renderer_d3d12.cpp
        src/render/renderer_vulkan.cpp)
    target_include_directories(fly_engine PRIVATE src)
    target_compile_definitions(fly_engine PRIVATE _CRT_SECURE_NO_WARNINGS)
    target_compile_options(fly_engine PRIVATE /bigobj /W4)
    target_link_libraries(fly_engine PRIVATE d3d12 dxgi d3dcompiler dbghelp xinput windowscodecs ole32 user32 gdi32 shell32)
    find_package(Vulkan QUIET)
    if(Vulkan_FOUND)
        target_compile_definitions(fly_engine PRIVATE TBD_HAVE_VULKAN)
        target_link_libraries(fly_engine PRIVATE Vulkan::Vulkan)
    endif()
    # Shaders next to the exe (Direct3D 12 compiles HLSL at startup and caches it in bin/shadercache)
    set_target_properties(fly_engine PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
        RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/bin)
    add_custom_command(TARGET fly_engine POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_SOURCE_DIR}/bin/shaders
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/src/shaders ${CMAKE_SOURCE_DIR}/bin/shaders)

elseif(CMAKE_SYSTEM_NAME STREQUAL "iOS")
    if(NOT CMAKE_GENERATOR STREQUAL "Xcode")
        message(FATAL_ERROR "Configure iOS with -G Xcode")
    endif()
    set(CMAKE_XCODE_GENERATE_SCHEME ON)
    string(TOLOWER "${CMAKE_OSX_SYSROOT}" FLY_SYSROOT)
    if(FLY_SYSROOT MATCHES "iphonesimulator")
        set(FLY_METAL_SDK iphonesimulator)
        set(FLY_METAL_TARGET "air64-apple-ios${CMAKE_OSX_DEPLOYMENT_TARGET}-simulator")
    else()
        set(FLY_METAL_SDK iphoneos)
        set(FLY_METAL_TARGET "air64-apple-ios${CMAKE_OSX_DEPLOYMENT_TARGET}")
    endif()
    set(CMAKE_OBJCXX_STANDARD 20)
    set(FLY_TEAM_ID "" CACHE STRING "Apple development team for code signing (empty: unsigned)")
    set(FLY_BUNDLE_ID "com.ovoa.flybasic" CACHE STRING "Bundle identifier")
    set(FLY_BUILD_NUMBER "1" CACHE STRING "CFBundleVersion: must rise with every TestFlight upload (Codemagic passes BUILD_NUMBER)")

    # Metal shaders: one .air per .metal, linked into default.metallib (as in cityengine-ios)
    file(GLOB FLY_MSL "${CMAKE_CURRENT_SOURCE_DIR}/src/shaders/msl/*.metal")
    set(FLY_SHADER_DIR "${CMAKE_CURRENT_BINARY_DIR}/shaders/${FLY_METAL_SDK}-${CMAKE_OSX_DEPLOYMENT_TARGET}")
    set(FLY_AIR)
    foreach(msl IN LISTS FLY_MSL)
        get_filename_component(stem "${msl}" NAME_WE)
        set(air "${FLY_SHADER_DIR}/${stem}.air")
        add_custom_command(OUTPUT "${air}"
            COMMAND "${CMAKE_COMMAND}" -E make_directory "${FLY_SHADER_DIR}"
            COMMAND xcrun --sdk "${FLY_METAL_SDK}" metal -std=metal3.0 -target "${FLY_METAL_TARGET}" -c "${msl}" -o "${air}"
            DEPENDS "${msl}" VERBATIM)
        list(APPEND FLY_AIR "${air}")
    endforeach()
    set(FLY_METALLIB "${FLY_SHADER_DIR}/default.metallib")
    add_custom_command(OUTPUT "${FLY_METALLIB}" COMMAND xcrun --sdk "${FLY_METAL_SDK}" metallib ${FLY_AIR} -o "${FLY_METALLIB}"
        DEPENDS ${FLY_AIR} VERBATIM)
    set_source_files_properties("${FLY_METALLIB}" PROPERTIES GENERATED TRUE MACOSX_PACKAGE_LOCATION Resources)
    set_source_files_properties(${FLY_MSL} PROPERTIES MACOSX_PACKAGE_LOCATION Resources/msl XCODE_EXPLICIT_FILE_TYPE text)

    # game data into the bundle's data/ folder: the baked airports, the Earth (places, land mask, coast) and the UI font
    file(GLOB FLY_AIRPORTS "${CMAKE_CURRENT_SOURCE_DIR}/data/airports/*.json")
    file(GLOB FLY_WORLD "${CMAKE_CURRENT_SOURCE_DIR}/data/world/*.bin")
    file(GLOB FLY_UI "${CMAKE_CURRENT_SOURCE_DIR}/data/ui/*")
    set_source_files_properties(${FLY_AIRPORTS} PROPERTIES MACOSX_PACKAGE_LOCATION Resources/data/airports XCODE_EXPLICIT_FILE_TYPE file)
    set_source_files_properties(${FLY_WORLD} PROPERTIES MACOSX_PACKAGE_LOCATION Resources/data/world XCODE_EXPLICIT_FILE_TYPE file)
    set_source_files_properties(${FLY_UI} PROPERTIES MACOSX_PACKAGE_LOCATION Resources/data/ui XCODE_EXPLICIT_FILE_TYPE file)
    set(FLY_DATA ${FLY_AIRPORTS} ${FLY_WORLD} ${FLY_UI})

    add_executable(fly_engine MACOSX_BUNDLE
        ${FLY_CORE_SOURCES}
        src/platform/ios/main_ios.mm
        src/platform/platform_ios.mm
        src/platform/apple/log_apple.cpp
        src/render/renderer_metal.mm
        "${FLY_METALLIB}" ${FLY_MSL} ${FLY_DATA})
    target_include_directories(fly_engine PRIVATE src third_party/metal-cpp)
    target_compile_definitions(fly_engine PRIVATE TBD_HAVE_METAL=1)
    set_source_files_properties(src/render/renderer_metal.mm PROPERTIES COMPILE_OPTIONS "-fno-objc-arc") # metal-cpp: manual refs
    set_source_files_properties(src/platform/ios/main_ios.mm src/platform/platform_ios.mm PROPERTIES COMPILE_OPTIONS "-fobjc-arc")
    target_link_libraries(fly_engine PRIVATE "-framework Foundation" "-framework CoreGraphics" "-framework UIKit"
        "-framework QuartzCore" "-framework Metal" "-framework ImageIO" "-framework GameController")
    set_target_properties(fly_engine PROPERTIES
        MACOSX_BUNDLE_INFO_PLIST ${CMAKE_SOURCE_DIR}/src/platform/ios/Info.plist.in
        MACOSX_BUNDLE_GUI_IDENTIFIER ${FLY_BUNDLE_ID}
        MACOSX_BUNDLE_BUNDLE_NAME "Fly"
        MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION}
        MACOSX_BUNDLE_BUNDLE_VERSION "${FLY_BUILD_NUMBER}"
        XCODE_ATTRIBUTE_CURRENT_PROJECT_VERSION "${FLY_BUILD_NUMBER}"
        XCODE_ATTRIBUTE_PRODUCT_BUNDLE_IDENTIFIER ${FLY_BUNDLE_ID}
        XCODE_ATTRIBUTE_IPHONEOS_DEPLOYMENT_TARGET "${CMAKE_OSX_DEPLOYMENT_TARGET}"
        XCODE_ATTRIBUTE_TARGETED_DEVICE_FAMILY "1,2"
        XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++20"
        XCODE_ATTRIBUTE_DEVELOPMENT_TEAM "${FLY_TEAM_ID}")
    if(FLY_TEAM_ID STREQUAL "")
        set_target_properties(fly_engine PROPERTIES XCODE_ATTRIBUTE_CODE_SIGNING_ALLOWED NO XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY "")
    endif()

else()
    message(FATAL_ERROR "Fly-engine mobile builds for iOS and Windows so far (Android / Quest 3 is planned)")
endif()
