# STM32 Rust Starter Template

This folder contains a reusable embedded Rust starter you can clone for different STM32 MCUs.

## Quick Start

Run from `stm32f103-bluepill-blink/template`:

```powershell
.\new-project.ps1 \
  -ProjectName stm32f103-blink \
  -Chip STM32F103C8 \
  -Target thumbv7m-none-eabi \
  -HalCrate stm32f1xx-hal \
  -HalFeature stm32f103 \
  -FlashKb 64 \
  -RamKb 20 \
  -ClockSetup ".use_hse(8.MHz()).sysclk(72.MHz()).pclk1(36.MHz())" \
  -GpioPort GPIOC \
  -LedPin pc13 \
  -LedMode "into_push_pull_output(&mut gpio.crh)" \
  -LedOn "led.set_low();" \
  -LedOff "led.set_high();"
```

Example for STM32F030:

```powershell
.\new-project.ps1 \
  -ProjectName stm32f030-blink \
  -Chip STM32F030F4Px \
  -Target thumbv6m-none-eabi \
  -HalCrate stm32f0xx-hal \
  -HalFeature stm32f030 \
  -FlashKb 16 \
  -RamKb 4 \
  -ClockSetup ".sysclk(8.MHz())" \
  -GpioPort GPIOA \
  -LedPin pa5 \
  -LedMode "into_push_pull_output(&mut gpio.moder, &mut gpio.otyper)" \
  -LedOn "led.set_high().ok();" \
  -LedOff "led.set_low().ok();"
```

## Notes

- HAL APIs differ across STM32 families. You may need to adjust pin init and RCC setup for your board.
- `-HalMod` is optional; if omitted, it is auto-derived from `-HalCrate` by replacing `-` with `_`.
- `.cargo/config.toml` and `.vscode/launch.json` are auto-filled for the chip/target you pass.
- Generated project includes `memory.x`; adjust Flash/RAM values for your exact device.
