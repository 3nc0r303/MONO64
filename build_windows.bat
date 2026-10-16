@echo off
setlocal
cd /d "%~dp0"
echo === MONO64 VST3 / standalone Windows x64 ===
echo Needs Visual Studio 2022 with Desktop Development with C++, Git, CMake
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
if errorlevel 1 goto failed
cmake --build build --config Release --parallel 4
if errorlevel 1 goto failed
echo.
echo Success! Plugin: build\MONO64_artefacts\Release\VST3\MONO64.vst3
echo Standalone: build\MONO64_artefacts\Release\Standalone\MONO64.exe
echo Copy the VST3 folder to C:\Program Files\Common Files\VST3\
pause
exit /b 0
:failed
echo.
echo BUILD FAILED. Copy the entire error log when asking for help.
pause
exit /b 1
