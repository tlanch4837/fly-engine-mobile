@echo off
rem Windows test build of the basic game (one cl call) into bin\fly_basic.exe. Needs Visual Studio 2022; VULKAN_SDK adds the Vulkan backend. Optional arg: exe name.
setlocal
set VCVARS="C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
call %VCVARS% >nul 2>&1 || exit /b 1
set S=%~dp0src
set B=%~dp0bin
set EXE=fly_basic
if not "%~1"=="" set EXE=%~1
if not exist "%B%\shaders" mkdir "%B%\shaders"
if not exist "%~dp0obj" mkdir "%~dp0obj"
rem the fork's own shader set
copy /y "%S%\shaders\*.hlsl" "%B%\shaders\" >nul
copy /y "%S%\shaders\*.hlsli" "%B%\shaders\" >nul
set CFLAGS=/nologo /bigobj /std:c++20 /EHsc /W4 /O2 /Zi /MP /Fd"%~dp0obj\\" /Fo"%~dp0obj\\"
set VKINC=
set VKLIB=
if defined VULKAN_SDK set VKINC=/I"%VULKAN_SDK%\Include" /DTBD_HAVE_VULKAN
if defined VULKAN_SDK set VKLIB=/LIBPATH:"%VULKAN_SDK%\Lib" vulkan-1.lib
cl %CFLAGS% %VKINC% /I"%S%" /I"%~dp0third_party\openxr\include" /D_CRT_SECURE_NO_WARNINGS /Fe"%B%\%EXE%.exe" ^
  "%S%\platform\win\main_win.cpp" "%S%\fly\game.cpp" "%S%\xr\openxr_session.cpp" "%S%\platform\platform_win.cpp" "%S%\render\log.cpp" "%S%\render\renderer_d3d12.cpp" "%S%\render\renderer_vulkan.cpp" ^
  /link /DEBUG /OPT:REF /OPT:ICF /SUBSYSTEM:WINDOWS user32.lib gdi32.lib shell32.lib d3d12.lib dxgi.lib d3dcompiler.lib dbghelp.lib xinput.lib "%~dp0third_party\openxr\lib\openxr_loader.lib" windowscodecs.lib ole32.lib %VKLIB% || exit /b 1
copy /y "%~dp0third_party\openxr\bin\openxr_loader.dll" "%B%\" >nul
echo Built %B%\%EXE%.exe
