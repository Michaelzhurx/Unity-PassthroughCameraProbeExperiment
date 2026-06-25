param(
    [string]$UnityRoot = "D:\Unity\6000.4.1f1",
    [string]$AprilTagVersion = "v3.4.5"
)

$ErrorActionPreference = "Stop"

$repoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$androidPlayer = Join-Path $UnityRoot "Editor\Data\PlaybackEngines\AndroidPlayer"
$ndkRoot = Join-Path $androidPlayer "NDK"
$cmakeExe = Join-Path $androidPlayer "SDK\cmake\3.22.1\bin\cmake.exe"
$ninjaExe = Join-Path $androidPlayer "SDK\cmake\3.22.1\bin\ninja.exe"
$toolchain = Join-Path $ndkRoot "build\cmake\android.toolchain.cmake"

if (!(Test-Path $cmakeExe)) { throw "CMake not found: $cmakeExe" }
if (!(Test-Path $ninjaExe)) { throw "Ninja not found: $ninjaExe" }
if (!(Test-Path $toolchain)) { throw "Android NDK toolchain not found: $toolchain" }

$tempRoot = Join-Path $repoRoot "Temp\AprilTagUnityBuild"
$sourceRoot = Join-Path $tempRoot "apriltag"
$buildRoot = Join-Path $tempRoot "build-arm64-v8a"
$wrapperRoot = Join-Path $repoRoot "Native\AprilTagUnity"
$outputRoot = Join-Path $repoRoot "Assets\Plugins\Android\arm64-v8a"
$outputLib = Join-Path $outputRoot "libapriltag_unity.so"

New-Item -ItemType Directory -Force -Path $tempRoot, $outputRoot | Out-Null

if (!(Test-Path $sourceRoot)) {
    git clone --depth 1 --branch $AprilTagVersion https://github.com/AprilRobotics/apriltag.git $sourceRoot
}

if (Test-Path $buildRoot) {
    Remove-Item -LiteralPath $buildRoot -Recurse -Force
}

& $cmakeExe -S $wrapperRoot -B $buildRoot -G Ninja `
    "-DCMAKE_MAKE_PROGRAM=$ninjaExe" `
    "-DCMAKE_TOOLCHAIN_FILE=$toolchain" `
    "-DANDROID_ABI=arm64-v8a" `
    "-DANDROID_PLATFORM=android-29" `
    "-DAPRILTAG_SOURCE_DIR=$sourceRoot" `
    "-DBUILD_PYTHON_WRAPPER=OFF" `
    "-DCMAKE_BUILD_TYPE=Release"

& $cmakeExe --build $buildRoot --config Release

$builtLib = Join-Path $buildRoot "libapriltag_unity.so"
if (!(Test-Path $builtLib)) {
    throw "Expected output library was not built: $builtLib"
}

Copy-Item -LiteralPath $builtLib -Destination $outputLib -Force
Write-Host "Built $outputLib"
