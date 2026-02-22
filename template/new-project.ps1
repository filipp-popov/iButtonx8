param(
    [Parameter(Mandatory = $true)] [string]$ProjectName,
    [Parameter(Mandatory = $true)] [string]$Chip,
    [Parameter(Mandatory = $true)] [string]$Target,
    [Parameter(Mandatory = $true)] [string]$HalCrate,
    [string]$HalMod,
    [Parameter(Mandatory = $true)] [string]$HalFeature,
    [Parameter(Mandatory = $true)] [int]$FlashKb,
    [Parameter(Mandatory = $true)] [int]$RamKb,
    [string]$HalVersion = "0.10",
    [string]$ClockSetup = ".sysclk(8.MHz())",
    [string]$GpioPort = "GPIOC",
    [string]$LedPin = "pc13",
    [string]$LedMode = "into_push_pull_output(&mut gpio.crh)",
    [string]$LedOn = "led.set_low();",
    [string]$LedOff = "led.set_high();",
    [int]$BlinkDivisor = 8,
    [string]$OutDir = "."
)

$ErrorActionPreference = "Stop"

$templateRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$skeleton = Join-Path $templateRoot "skeleton"
$projectDir = Join-Path $OutDir $ProjectName

if ([string]::IsNullOrWhiteSpace($HalMod)) {
    $HalMod = $HalCrate.Replace("-", "_")
}

if (Test-Path $projectDir) {
    throw "Destination already exists: $projectDir"
}

Copy-Item -Recurse -Force $skeleton $projectDir

$replacements = @{
    "__PROJECT_NAME__" = $ProjectName
    "__CHIP__" = $Chip
    "__TARGET__" = $Target
    "__HAL_CRATE__" = $HalCrate
    "__HAL_MOD__" = $HalMod
    "__HAL_VERSION__" = $HalVersion
    "__HAL_FEATURE__" = $HalFeature
    "__FLASH_KB__" = [string]$FlashKb
    "__RAM_KB__" = [string]$RamKb
    "__CLOCK_SETUP__" = $ClockSetup
    "__GPIO_PORT__" = $GpioPort
    "__LED_PIN__" = $LedPin
    "__LED_MODE__" = $LedMode
    "__LED_ON__" = $LedOn
    "__LED_OFF__" = $LedOff
    "__BLINK_DIVISOR__" = [string]$BlinkDivisor
}

Get-ChildItem -Path $projectDir -Recurse -File | ForEach-Object {
    $content = Get-Content -Raw $_.FullName
    foreach ($k in $replacements.Keys) {
        $content = $content.Replace($k, $replacements[$k])
    }
    Set-Content -Path $_.FullName -Value $content
}

Write-Host "Created project: $projectDir"
Write-Host "Next:"
Write-Host "  1) cd $projectDir"
Write-Host "  2) rustup target add $Target"
Write-Host "  3) cargo build"
