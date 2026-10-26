# Step 02: Build the Box3D native library

**Goal:** a reproducible Windows x64 `box3d.dll` built from a pinned Box3D commit, checked in as a resource of the
library jar.

**Depends on:** step 01.

## Facts

- Box3D: https://github.com/erincatto/box3d, MIT license, by Erin Catto. The author calls it alpha. The only tag is
  `v0.1.0` (2026-06-30), and the public headers changed substantially after it.
- **Pin commit `f36d061ba26998c87cb7dc593a4065aaa03fb7eb`** (2026-10-20, "CMake improvements (#201)"). The API
  notes in steps 03 and 04 were read from this commit. Do not use the `v0.1.0` tag.
- `BUILD_SHARED_LIBS=ON` builds a DLL. On Windows, CMake defines `box3d_EXPORTS` for the shared target, so `B3_API`
  becomes `__declspec(dllexport)`.
- The top-level `CMakeLists.txt` declares `LANGUAGES C CXX`, so a C++ compiler must exist even for a library-only build.
- Single precision is the default. **Do not** set `BOX3D_DOUBLE_PRECISION`: it changes the ABI of `b3Pos` and
  `b3WorldTransform`, and steps 03 and 04 assume floats.
- With `BOX3D_DISABLE_SIMD` left at its default (OFF), an AVX2 path is chosen at runtime, with an SSE2 fallback. No
  flags are needed.
- The DLL lands in `<build dir>/bin/box3d.dll`.

This machine has Visual Studio 2019 and 2022, Ninja (from the Python Scripts dir), CMake 4.2 and LLVM clang 19.

## Files

### Submodule

```
git submodule add https://github.com/erincatto/box3d.git native/box3d
cd native/box3d
git checkout f36d061ba26998c87cb7dc593a4065aaa03fb7eb
cd ../..
git add .gitmodules native/box3d
```

### `scripts/build-native.ps1`

A PowerShell 7 script that:

1. Finds Visual Studio with
   `& "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`,
   then sets `$vcvars = "$vsPath\VC\Auxiliary\Build\vcvars64.bat"`. It fails with a clear message if either is
   missing.
2. Runs CMake inside the vcvars environment through `cmd /c`, so MSVC `cl` is the compiler:

   ```
   cmd /c "`"$vcvars`" && cmake -S native/box3d -B native-build/windows-x86_64 -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DBOX3D_SAMPLES=OFF -DBOX3D_UNIT_TESTS=OFF -DBOX3D_BENCHMARKS=OFF -DBOX3D_DOCS=OFF -DBOX3D_VALIDATE=OFF && cmake --build native-build/windows-x86_64 --config Release"
   ```

   Check `$LASTEXITCODE` and throw if it is non-zero.
3. Copies `native-build/windows-x86_64/bin/box3d.dll` to
   `src/main/resources/natives/windows-x86_64/box3d.dll`, creating the directory if needed.
4. Writes `src/main/resources/natives/box3d.version`. It holds one line: the output of
   `git -C native/box3d rev-parse HEAD`.

It runs from the repository root (`Set-Location $PSScriptRoot/..` first).

**Fallback when vcvars or MSVC is broken:** replace the compiler arguments with
`-DCMAKE_C_COMPILER=clang-cl -DCMAKE_CXX_COMPILER=clang-cl -DCMAKE_LINKER=lld-link`. Keep the vcvars wrapper, because
clang-cl still needs the MSVC CRT and Windows SDK. Only use this fallback if the MSVC build fails.

### Checked-in binary

Commit `src/main/resources/natives/windows-x86_64/box3d.dll` and `src/main/resources/natives/box3d.version`. The robot
build then doesn't need a native toolchain. `.gitattributes` from step 01 already marks `*.dll` as binary.

Linux and macOS natives are out of scope for v0. Leave the script structured so another platform block can be added
later (for example a `switch` on `$IsWindows`).

## Acceptance

```
pwsh scripts/build-native.ps1
```

- It ends with `src/main/resources/natives/windows-x86_64/box3d.dll` present (roughly 0.5–3 MB).
- The DLL exports the API. Check with `llvm-objdump -p src/main/resources/natives/windows-x86_64/box3d.dll`
  (LLVM is on PATH) and confirm the export table lists `b3CreateWorld`, `b3World_Step`, `b3CreateSphereShape`,
  `b3MakeBoxHull` and `b3GetVersion`.
- `.\gradlew.bat build` still passes, and the jar contains `natives/windows-x86_64/box3d.dll` (`jar tf build/libs/box3d-sim-0.1.0-SNAPSHOT.jar`).

Commit: `Build Box3D f36d061 as a Windows DLL`.
