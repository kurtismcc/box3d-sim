# Downloads and verifies jextract (JDK 25 early-access build) into tools/.
$ErrorActionPreference = 'Stop'
Set-Location (Join-Path $PSScriptRoot '..')

$url = 'https://download.java.net/java/early_access/jextract/25/2/openjdk-25-jextract+2-4_windows-x64_bin.tar.gz'
$archive = 'tools/jextract.tar.gz'
New-Item -ItemType Directory -Force tools | Out-Null

Invoke-WebRequest -Uri $url -OutFile $archive
$expected = ((Invoke-WebRequest -Uri "$url.sha256").Content | Out-String).Trim().Split()[0].ToLower()
$actual = (Get-FileHash -Algorithm SHA256 $archive).Hash.ToLower()
if ($expected -ne $actual) { throw "jextract SHA-256 mismatch: expected $expected, got $actual" }

tar -xzf $archive -C tools
if ($LASTEXITCODE -ne 0) { throw "tar failed with exit code $LASTEXITCODE" }
Remove-Item $archive

$bat = Get-ChildItem tools -Recurse -Filter jextract.bat | Select-Object -First 1
if (-not $bat) { throw 'jextract.bat not found after extraction' }
Write-Host $bat.FullName
