@echo off
rem Builds the Release engine and runs the unit tests from a clean checkout.
rem The no-mistakes gate runs this as its test command (see .no-mistakes.yaml),
rem and it works the same when run by hand from the repository root.
setlocal

call :FindCMake || exit /b 1

echo == Configuring
"%CMAKE%" --preset vs2026 || exit /b 1

echo == Building Release
"%CMAKE%" --build --preset release || exit /b 1

echo == Running tests
rem ctest lives next to cmake. It runs the unit_tests target from CMakeLists.txt.
"%CMAKE_DIR%\ctest.exe" --test-dir build -C Release --output-on-failure || exit /b 1

echo == Gate check passed
exit /b 0

:FindCMake
rem Prefer a cmake on PATH, otherwise use the copy bundled with Visual Studio,
rem since a plain Visual Studio install does not put cmake on PATH.
for /f "delims=" %%c in ('where cmake 2^>nul') do (
    set "CMAKE=%%c"
    goto :FoundCMake
)
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" goto :NoCMake
rem The "call" prefix keeps cmd from stripping the quotes around a path that
rem contains parentheses, such as "Program Files (x86)".
for /f "delims=" %%c in ('call "%VSWHERE%" -latest -products * -find Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe') do (
    set "CMAKE=%%c"
    goto :FoundCMake
)
:NoCMake
echo error: cmake not found. Install CMake or the Visual Studio C++ CMake tools.
exit /b 1

:FoundCMake
for %%d in ("%CMAKE%") do set "CMAKE_DIR=%%~dpd"
set "CMAKE_DIR=%CMAKE_DIR:~0,-1%"
echo Using cmake: %CMAKE%
exit /b 0
