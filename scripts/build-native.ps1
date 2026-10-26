# Builds box3d.dll from the native/box3d submodule and copies it into the jar resources.
$ErrorActionPreference = 'Stop'
Set-Location (Join-Path $PSScriptRoot '..')

if ($IsWindows -or $env:OS -eq 'Windows_NT') {
    $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
    if (-not (Test-Path $vswhere)) { throw "vswhere.exe not found at $vswhere; install Visual Studio or Build Tools with the C++ workload." }
    $vsPath = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    if (-not $vsPath) { throw 'No Visual Studio installation with the x64 C++ tools was found.' }
    $vcvars = Join-Path $vsPath 'VC\Auxiliary\Build\vcvars64.bat'
    if (-not (Test-Path $vcvars)) { throw "vcvars64.bat not found at $vcvars" }

    $platform = 'windows-x86_64'
    $buildDir = "native-build/$platform"
    $cmakeArgs = @(
        '-S native/box3d', "-B $buildDir", '-G Ninja', '-DCMAKE_BUILD_TYPE=Release', '-DBUILD_SHARED_LIBS=ON',
        '-DBOX3D_SAMPLES=OFF', '-DBOX3D_UNIT_TESTS=OFF', '-DBOX3D_BENCHMARKS=OFF', '-DBOX3D_DOCS=OFF', '-DBOX3D_VALIDATE=OFF'
    ) -join ' '
    cmd /c "`"$vcvars`" >nul && cmake $cmakeArgs && cmake --build $buildDir --config Release"
    if ($LASTEXITCODE -ne 0) { throw "Native build failed with exit code $LASTEXITCODE" }

    $dll = "$buildDir/bin/box3d.dll"
    $outDir = "src/main/resources/natives/$platform"
} else {
    throw 'Only Windows x64 natives are supported in v0.'
}

if (-not (Test-Path $dll)) { throw "Expected $dll after the build" }
New-Item -ItemType Directory -Force $outDir | Out-Null
Copy-Item $dll $outDir -Force
(git -C native/box3d rev-parse HEAD).Trim() | Set-Content -NoNewline -Encoding ascii 'src/main/resources/natives/box3d.version'
Write-Host "Copied $dll to $outDir"
