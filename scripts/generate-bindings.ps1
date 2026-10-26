# Regenerates src/main/java/box3dsim/ffm/gen from the native/box3d headers with jextract.
$ErrorActionPreference = 'Stop'
Set-Location (Join-Path $PSScriptRoot '..')

$jextract = Get-ChildItem tools -Recurse -Filter jextract.bat -ErrorAction SilentlyContinue | Select-Object -First 1
if (-not $jextract) { throw 'jextract not found; run scripts/get-jextract.ps1 first' }

$include = 'native/box3d/include'
$header = "$include/box3d/box3d.h"
New-Item -ItemType Directory -Force native-build | Out-Null

& $jextract.FullName --dump-includes native-build/jextract-all.txt -I $include $header
if ($LASTEXITCODE -ne 0) { throw "jextract --dump-includes failed with exit code $LASTEXITCODE" }

# Only functions marked B3_API exist in the DLL; the rest are static inline helpers.
$exported = [System.Collections.Generic.HashSet[string]]::new()
foreach ($file in Get-ChildItem "$include/box3d" -Filter *.h) {
    foreach ($line in Get-Content $file.FullName) {
        $m = [regex]::Match($line, 'B3_API\s+[\w\s\*]+?\b(b3\w+)\s*\(')
        if ($m.Success) { [void]$exported.Add($m.Groups[1].Value) }
    }
}

$kept = foreach ($line in Get-Content native-build/jextract-all.txt) {
    $m = [regex]::Match($line, '^(--include-\w+)\s+(\S+)\s+#\s*header:\s*(.+)$')
    if (-not $m.Success) { continue }
    $kind = $m.Groups[1].Value
    $name = $m.Groups[2].Value
    $path = $m.Groups[3].Value.Replace('\', '/')
    if ($kind -eq '--include-function') {
        if ($exported.Contains($name)) { "$kind $name" }
    } elseif ($path.Contains('native/box3d/include')) {
        "$kind $name"
    }
}
$kept | Set-Content -Encoding ascii native-build/jextract-includes.txt

$out = 'src/main/java/box3dsim/ffm/gen'
if (Test-Path $out) { Remove-Item -Recurse -Force $out }
& $jextract.FullName --output src/main/java --target-package box3dsim.ffm.gen --header-class-name Box3D `
    -I $include '@native-build/jextract-includes.txt' $header
if ($LASTEXITCODE -ne 0) { throw "jextract failed with exit code $LASTEXITCODE" }

Write-Host "Exported functions: $($exported.Count); generated files: $((Get-ChildItem $out -Filter *.java).Count)"
