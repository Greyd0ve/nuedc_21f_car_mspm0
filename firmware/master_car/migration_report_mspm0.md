# MSPM0G3507 First-Round Port Report

## Paths

- New project: `C:\Users\zch20\Desktop\测试文件夹\nuedc_e_car_mspm0`
- Keil project: `C:\Users\zch20\Desktop\测试文件夹\nuedc_e_car_mspm0\keil\empty_LP_MSPM0G3507_nortos_keil.uvprojx`
- Build log: `C:\Users\zch20\Desktop\测试文件夹\nuedc_e_car_mspm0\build_mspm0_port.log`

The Keil target name is still `empty_LP_MSPM0G3507_nortos_keil` for command-line build compatibility. Source groups and output still point to the migrated MSPM0 car firmware.

## Migrated App/Control Files

- `App/app_e_car.c`, `App/app_e_car.h`
- `App/app_e_serial.c`, `App/app_e_serial.h`
- `App/app_control.c`, `App/app_control.h`
- `App/app_line.c`, `App/app_line.h`
- `Control/pid.c`, `Control/pid.h`

Changes:

- Removed STM32 header usage from App files.
- `App/app_line.c` now calls `Grayscale_ReadAll(raw)` instead of direct GPIO access.
- `App/app_e_serial.c` adds `ECAR_ENABLE_REMOTE_START`, default `0`, so remote start commands return `start-disabled`.

## Rewritten Hardware/System/User Files

- `Hardware/Board_Config.h`
- `Hardware/PWM.c`, `Hardware/PWM.h`
- `Hardware/Motor.c`, `Hardware/Motor.h`
- `Hardware/Encoder.c`, `Hardware/Encoder.h`
- `Hardware/Grayscale.c`, `Hardware/Grayscale.h`
- `Hardware/Key.c`, `Hardware/Key.h`
- `Hardware/Serial.c`, `Hardware/Serial.h`
- `Hardware/OLED.c`, `Hardware/OLED.h`
- `Hardware/BeepLed.c`, `Hardware/BeepLed.h`
- `System/Timer.c`, `System/Timer.h`
- `User/main.c`

## Board_Config.h IO Summary

- Motor: PA12/PA13 PWM on TIMG0, PB6/PB7 left direction, PB8/PB9 right direction, PB23 STBY.
- Encoders: PB10/PB11 left A/B, PB0/PB1 right A/B, A phase GPIO interrupt, signs `LEFT=-1`, `RIGHT=+1`.
- Grayscale CD4051: PA14 OUT, PA15/PA16/PA17 AD0/AD1/AD2.
- UART: PA8 TX, PA9 RX, UART1, 9600 baud.
- Keys: PB15/PB16/PB2/PB3, internal pull-up, active low.
- System tick: TIMG8 1ms periodic interrupt.

## SysConfig

Configured and generated:

- GPIO: motor direction/STBY, encoder inputs and interrupts, grayscale address/output, keys.
- PWM: TIMG0 CC0/CC1 on PA12/PA13, 1600 timer counts for about 20kHz at 32MHz, app-level duty range 0..1000.
- Timer: TIMG8 1ms periodic interrupt.
- UART: UART1 on PA8/PA9 at 9600 baud with RX interrupt.

I2C/OLED is not configured in this first round because OLED is a safe stub.

## Real Drivers vs Stubs

Real DriverLib implementations:

- PWM
- Motor
- Encoder GPIO interrupt counting
- Grayscale CD4051 polling
- Key debounce
- Serial UART TX/RX ring buffer
- 1ms Timer scheduler

Temporary safe stubs:

- OLED
- BeepLed

## Checks

- `stm32f10x.h`: not present in migrated source.
- STM32 SPL calls: not present in migrated source.
- Keil Rebuild: `0 Error(s), 0 Warning(s)`.
- Flash/download: not executed.

## Safety State

- `g_carEnable = 0` at init.
- Target speeds are zero at init.
- `Motor_Init()` calls `Motor_StopAll()`.
- PWM compare is zeroed.
- Direction pins are forced low in `Motor_StopAll()`.
- Remote serial start is disabled by default.
- `main()` does not call `ECar_Start()`.

## Why Not Run On Car Yet

This is only a compile-valid first-round port. Pin electrical behavior, PWM polarity, encoder direction, grayscale black/white level, UART wiring, and key wiring have not been bench-verified on the MSPM0 car board.

## Next Round

- UART echo test.
- Key input test.
- PWM test with motors lifted/off-load.
- Encoder lifted-wheel test.
- Print grayscale `raw[8]`.
- Add and test real OLED I2C display.
- Enable `ECar_Start()` only after the above checks pass.
