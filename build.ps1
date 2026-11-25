param([string]$Keil = 'E:\keil5\32\UV4\UV4.exe')
$ErrorActionPreference = 'Stop'
$carRoot = $PSScriptRoot
$project = Join-Path $carRoot 'car.uvprojx'
$target = 'car'
$image = 'car.hex'
$objects = 'Objects'
$null = New-Item -ItemType Directory -Force (Join-Path $carRoot $objects)
$log = Join-Path $carRoot "$objects/keil_build.log"
if (-not (Test-Path -LiteralPath $Keil)) { throw "Keil not found: $Keil. Supply -Keil with the UV4.exe path." }
$process = Start-Process -FilePath $Keil -ArgumentList @('-r', ('"' + $project + '"'), '-t', $target, '-o', ('"' + $log + '"')) -WindowStyle Hidden -Wait -PassThru
$result = Get-Content -LiteralPath $log -Raw
Write-Output $result
if ($process.ExitCode -gt 1 -or $result -notmatch '0 Error\(s\)') { throw 'Keil build failed; see Objects/keil_build.log.' }
Copy-Item -LiteralPath (Join-Path $carRoot "$objects/$target.hex") -Destination (Join-Path $carRoot $image) -Force
Write-Output "Updated $image. This script does not flash the board."
