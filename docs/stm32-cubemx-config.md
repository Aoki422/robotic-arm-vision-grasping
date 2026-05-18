# STM32 HAL 适配说明（非当前运行前提）

当前项目主目标是**无实机 PC/仿真闭环**。本文件只说明未来如何把已经通过 PC 测试的核心逻辑接回 CubeMX HAL；它不是当前运行、测试或验证的必要步骤。

## 当前 PC 模式

PC 构建时启用：

```bash
cmake -S . -B build -DPC_SIM=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

`PC_SIM` 会启用：

- `Core/Inc/platform_hal.h`
- `Core/Src/platform_hal_pc.c`

这些文件提供 `HAL_GetTick`、PWM compare、IRQ 开关等模拟桩函数，不依赖真实 STM32、定时器或串口。

## 未来实机适配接口

若未来要接入真实 STM32 工程：

1. 取消 `PC_SIM` 编译宏。
2. 让 `platform_hal.h` 包含 CubeMX 生成的 `stm32f1xx_hal.h`。
3. 在 CubeMX 工程中提供 `htim2`、`htim3`、USART 中断入口。
4. 在 USART 接收中断中调用 `UART_FeedByte(byte)`。
5. 通过 `DebugLog_SetWriter()` 注入串口日志发送函数。

## 与当前项目的边界

以上内容只描述接口适配点，不要求真实硬件、实物标定或烧录才能运行本仓库。当前验证路径以 `tests/run_tests.py` 和 `coppeliasim/visual_grasping.py --mode pc` 为准。
