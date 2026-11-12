# dmmc02-hexmotor

DM-MC02（STM32H723VGT6）上的 hexmeow Meow 协议关节电机驱动。  
控制通路使用压缩 MIT / 共享 RPDO，支持 PV、PP、力矩模式和多电机。

## 1. 工程结构

```text
dmmc02-hexmotor/
├── CMakeLists.txt                  # 顶层构建入口
├── README.md
├── LICENSE
├── .gitignore
│
├── cmake/
│   └── arm-none-eabi.cmake         # ARM GNU 工具链
│
├── firmware/
│   ├── CMakeLists.txt
│   ├── app/
│   │   ├── main.c                  # 三总线例程：CAN1 + CAN2 + CAN3
│   │   ├── demo.c                  # 可选自检（默认关闭）
│   │   └── demo.h
│   ├── bsp/
│   │   ├── board.c/h               # 板级初始化、电机电源控制
│   │   ├── board_clock.c/h         # 时钟树
│   │   ├── board_fdcan.c/h         # FDCAN1/2/3、5M BRS、TDC
│   │   ├── board_uart.c/h          # USART1 日志
│   │   └── board_led.c/h           # WS2812 状态灯
│   ├── drivers/
│   │   ├── canfd/                  # CAN/CAN-FD 传输层
│   │   ├── canopen/                # NMT / 心跳 / SDO
│   │   └── meow/                   # Meow 设备层与控制层
│   └── platform/                   # startup、链接脚本、HAL 配置
│
├── tests/
│   └── host/                       # 主机侧编解码单元测试
│
└── third_party/
    └── stm32h7/                    # STM32H7 HAL + CMSIS
```

`build/`、`build-host/` 为本地构建目录，已被 `.gitignore` 忽略。

## 2. 驱动结构

```text
应用层
  meow_motor_mit / set_velocity / set_position / set_torque
  meow_motor_get_state / read_status
        │
        ▼
设备层 meow_motor
  身份识别、初始化、模式切换
  MIT 范围、力矩限幅、规划参数
  错误/在线/状态管理
        │
        ▼
总线层 meow_bus
  多电机调度、共享 RPDO、发现、心跳、bus-off 恢复
        │
        ▼
协议层 canopen
  NMT、心跳、SDO expedited
        │
        ▼
传输层 canfd
  CAN / CAN-FD、BRS、TDC、RX 环形缓冲
        │
        ▼
BSP / STM32H723 FDCAN
```

### 模块职责

| 模块 | 职责 |
| --- | --- |
| `meow_types` | 型号、模式、错误码、范围、状态等公共类型 |
| `meow_codec` | 压缩 MIT 打包、TPDO 解码、速度估计 |
| `meow_motor` | 单台电机初始化、参数读写、MIT/PV/PP/力矩控制 |
| `meow_bus` | 多电机共享帧、发现、心跳、在线监控 |
| `canopen` | NMT、SDO、心跳生产/消费 |
| `canfd` | CAN-FD 收发、BRS、TDC、bus-off 恢复 |
| `board_fdcan` | 1 Mbit/s 仲裁段 + 5 Mbit/s 数据段时序 |
| `board_led` | WS2812 渐变呼吸灯 |

### 控制数据流

- **MIT 控制下行**：`meow_motor_mit()` 只更新缓存 → `meow_bus_sync()` 把每台电机 8 字节目标打包为共享 RPDO1 `0x190` → FDCAN 发送。
- **模式控制**：共享 RPDO2 `0x290` 每台电机 1 字节模式；PP/PV 的 SDO 参数由 `meow_motor` 写入。
- **状态上行**：FDCAN 中断 → RX 环形缓冲 → `meow_bus_poll()` → 心跳/TPDO 解码 → `meow_motor` 状态缓存。

## 3. 使用方式

### 3.1 构建

```bash
export ARM_TOOLCHAIN_PATH=/path/to/arm-none-eabi/bin

cmake -S . -B build \
  -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug

cmake --build build -j
```

产物：

```text
build/dmmc02_hexmotor.elf
build/dmmc02_hexmotor.hex
build/dmmc02_hexmotor.bin
```

### 3.2 烧录

```bash
printf 'connect\nh\nloadfile build/dmmc02_hexmotor.hex\nr\ng\nq\n' > /tmp/flash.jlink
JLinkExe -device STM32H723VG -if SWD -speed 4000 \
  -autoconnect 1 -CommandFile /tmp/flash.jlink
```

### 3.3 三总线初始化

每条物理总线使用独立的 `meow_bus_t` 和电机数组；主机节点 ID 都是 `0x10`，互不冲突。

```c
#include "board.h"
#include "meow_bus.h"
#include "meow_motor.h"

meow_bus_t bus1, bus2, bus3;
meow_motor_t motors1[MEOW_MAX_MOTORS];
meow_motor_t motors2[MEOW_MAX_MOTORS];
meow_motor_t motors3[MEOW_MAX_MOTORS];

/* 上电默认打开两路可控电机电源，不做额外判断。 */
board_motor_power_set(BOARD_MOTOR_POWER_OUT1, true);  /* CAN1 / P7 */
board_motor_power_set(BOARD_MOTOR_POWER_OUT2, true);  /* CAN2 / P8 */

/* 等电机控制器启动完成后再发现。 */
board_delay_ms(500U);

meow_bus_init(&bus1, BOARD_FDCAN_1, 0x10U);  /* PD0/PD1 */
meow_bus_init(&bus2, BOARD_FDCAN_2, 0x10U);  /* PB5/PB6 */
meow_bus_init(&bus3, BOARD_FDCAN_3, 0x10U);  /* PD12/PD13 */

meow_bus_start(&bus1);
meow_bus_start(&bus2);
meow_bus_start(&bus3);

meow_bus_discover(&bus1, 1500U);
meow_bus_discover(&bus2, 1500U);
meow_bus_discover(&bus3, 1500U);

/* 每条总线上的节点 ID 必须唯一。 */
meow_bus_add_motor(&bus1, &motors1[0], 0x01U);
meow_bus_add_motor(&bus2, &motors2[0], 0x01U);
meow_bus_add_motor(&bus2, &motors2[1], 0x02U);

/* NULL 使用默认压缩 MIT 范围。 */
meow_bus_configure(&bus1, NULL);
meow_bus_configure(&bus2, NULL);
meow_bus_configure(&bus3, NULL);

while (1) {
    meow_bus_poll(&bus1);
    meow_bus_poll(&bus2);
    meow_bus_poll(&bus3);

    meow_bus_sync(&bus1);
    meow_bus_sync(&bus2);
    meow_bus_sync(&bus3);

    board_delay_ms(1U);
}
```

所有电机必须在对应的 `meow_bus_configure()` 之前加入。示例应用会自动发现在线的
电机；没有电机的总线扫描后保持静默，不发送主机心跳。

### 3.4 压缩 MIT

```c
/* p  : rev
 * v  : rev/s
 * kp : Nm/rev
 * kd : Nm*s/rev
 * tf : Nm
 */
meow_motor_enable_mit(&bus, &motor);
meow_motor_mit(&motor, p, v, kp, kd, tf);

while (1) {
    meow_bus_poll(&bus);
    meow_bus_sync(&bus);   /* 1 kHz */
    board_delay_ms(1);
}
```

`meow_motor_mit()` 非阻塞；`meow_bus_sync()` 负责实际发送共享帧。

### 3.5 PV / PP / 力矩

```c
meow_motor_set_velocity(&bus, &motor, 0.5f);   /* rev/s */
meow_motor_set_position(&bus, &motor, 0.25f);  /* rev */
meow_motor_set_torque(&bus, &motor, 1.0f);     /* Nm */
meow_motor_disable(&bus, &motor);
meow_motor_clear_error(&bus, &motor);
```

PP 在固件内部使用 Meow + CiA402 混合序列，应用层只需要 `meow_motor_set_position()`。

### 3.6 参数读写

```c
meow_mit_range_t range;
meow_range_default(&range);
meow_motor_set_mit_range(&bus, &motor, &range);
meow_motor_get_mit_range(&bus, &motor, &range);

meow_motor_set_torque_limit(&bus, &motor, 5.0f);   /* Nm */
float limit_nm;
meow_motor_get_torque_limit(&bus, &motor, &limit_nm);

meow_profile_limits_t limits = {
    .velocity_limit_rev_s = 3.0f,
    .acceleration_rev_s2  = 10.0f,
    .deceleration_rev_s2  = 10.0f,
};
meow_motor_set_profile_limits(&bus, &motor, &limits);
meow_motor_get_profile_limits(&bus, &motor, &limits);
```

`set_mit_range()` / `set_torque_limit()` 要求电机处于失能状态。

### 3.7 状态与错误

```c
meow_motor_state_t st;
meow_motor_get_state(&motor, &st);

if (st.fault || st.error_code != 0U) {
    /* error_code、NMT、在线状态均可从 st 读取 */
    meow_motor_clear_error(&bus, &motor);
}
```

### 3.8 多总线与多电机

- CAN1、CAN2、CAN3 可同时运行，各自有独立的 `meow_bus_t`。
- 每条总线的主机节点默认 `0x10`，因为三条物理总线互相隔离。
- 每条总线独立的共享 RPDO1 `0x190`（每台电机 8 字节压缩 MIT）和
  RPDO2 `0x290`（每台电机 1 字节模式）。
- 同一条总线上节点 ID 必须唯一；不同总线可以使用相同节点 ID。
- 1 台电机：8 字节经典帧；2–8 台：`8 × N` 字节 CAN-FD + BRS。
- 所有电机必须在对应总线的 `meow_bus_configure()` 之前加入。

### 3.9 电机电源

```c
board_motor_power_set(BOARD_MOTOR_POWER_OUT1, true);   /* CAN1 / P7 */
board_motor_power_set(BOARD_MOTOR_POWER_OUT2, true);   /* CAN2 / P8 */
board_motor_power_set(BOARD_MOTOR_POWER_OUT2, false);  /* 关断 */
```

示例固件在上电时无条件打开 OUT1 和 OUT2，不做电压、电流或状态判断。

| 接口 | CAN | 电机 24V |
| --- | --- | --- |
| P7 / OUT1（PC14） | CAN1 | 可控 |
| P8 / OUT2（PC13） | CAN2 | 可控 |
| P12 | CAN3 | 无开关，需外部 24V |

`PC15 / Power_5V_EN` 只控制 5V 逻辑/CAN 收发器电源，不能用来切电机。

### 3.10 构建选项

| 选项 | 默认 | 说明 |
| --- | --- | --- |
| `MEOW_DEMO_MIT` | `OFF` | MIT 零力矩自检 |
| `MEOW_DEMO_PVPP` | `OFF` | PV/PP 自检 |
| `MEOW_DEBUG_OD` | `OFF` | 对象字典探测 |
| `MEOW_DEMO_CIA_PP` | `OFF` | CiA402 PP 专项诊断 |
| `MEOW_TEST_NODE` | `2` | 自检目标节点（1=4310，2=4342） |

## 4. 字段含义

### 4.1 单位

| 字段 | 单位 |
| --- | --- |
| position | rev（输出轴圈数） |
| velocity | rev/s |
| torque | Nm |
| kp | Nm/rev |
| kd | Nm·s/rev |
| temperature | °C |
| timestamp | µs |

### 4.2 `meow_mit_range_t`

| 字段 | 含义 |
| --- | --- |
| `position_min_rev` / `position_max_rev` | 压缩 MIT 位置范围 |
| `velocity_min_rev_s` / `velocity_max_rev_s` | 速度范围 |
| `torque_min_nm` / `torque_max_nm` | 力矩范围 |
| `kp_min_nm_rev` / `kp_max_nm_rev` | kp 范围 |
| `kd_min_nm_s_rev` / `kd_max_nm_s_rev` | kd 范围 |

范围必须有限、`min < max`，且包含 0。

### 4.3 `meow_profile_limits_t`

| 字段 | 含义 |
| --- | --- |
| `velocity_limit_rev_s` | 规划速度上限 |
| `acceleration_rev_s2` | 规划加速度 |
| `deceleration_rev_s2` | 规划减速度 |

### 4.4 `meow_motor_state_t`

| 字段 | 含义 |
| --- | --- |
| `position_rev` | 输出轴位置 |
| `velocity_rev_s` | 位置差分 + 滤波后的速度 |
| `torque_nm` | 实际力矩 |
| `driver_temp_c` / `motor_temp_c` | 驱动器 / 电机温度 |
| `timestamp_us` | 电机 TPDO 时间戳 |
| `last_update_ms` | 最近一次状态更新时间 |
| `error_code` | 详细错误码 `0x453F` |
| `error_register` | 错误寄存器 `0x1001` |
| `mode_display` | 当前模式显示 |
| `nmt_state` | NMT 状态（Boot-up/Stopped/Operational/Pre-Op） |
| `online` | 是否在线 |
| `enabled` | 是否使能 |
| `fault` | 是否故障 |

### 4.5 模式值

| 值 | 模式 |
| --- | --- |
| 0 | Disabled |
| 1 | Profile Position |
| 2 | Profile Velocity |
| 3 | Torque |
| 4 | MIT |

### 4.6 关键对象字段

| 对象 | 含义 |
| --- | --- |
| `0x4103:01` | 压缩 MIT 选择器 |
| `0x4103:02/03` | 压缩 MIT 目标（lower/upper） |
| `0x4103:04..0D` | 压缩 MIT 映射范围 |
| `0x4103:0C/0D` | 力矩映射范围 |
| `0x4401` | 模式命令 |
| `0x4402` | 模式显示 |
| `0x453F` | 详细错误 |
| `0x4564` | 实际位置（Q8.24） |
| `0x4571` | 目标力矩（‰ peak） |
| `0x4576` | 峰值力矩（Nm） |
| `0x4577` | 实际力矩（‰ peak） |
| `0x457A` | PP 目标位置（Q8.24） |
| `0x4581/83/84` | 规划速度/加速度/减速度 |
| `0x45FF` | PV 目标速度（f32 rev/s） |
| `0x6040/6041` | CiA402 控制字/状态字 |
| `0x6060/6061` | CiA402 模式/模式显示 |
| `0x6064` | CiA402 实际位置（f32 rev） |
| `0x607A` | CiA402 PP 目标（f32 rev） |
| `0x6081/83/84` | CiA402 规划参数 |

### 4.7 共享 RPDO 字段

| 帧 | 内容 |
| --- | --- |
| RPDO1 `0x190` | 每台电机 8 字节：position 16 bit、velocity/torque/kp/kd 各 12 bit |
| RPDO2 `0x290` | 每台电机 1 字节模式命令 |

### 4.8 TPDO 字段

| 帧 | 内容 |
| --- | --- |
| TPDO1 | `0x4564` 位置 Q8.24、`0x4713` 时间戳、`0x4577` 力矩 ‰ |
| TPDO2 | `0x4402` 模式、`0x453F` 错误、`0x4104:01/02` 驱动器/电机温度 |

## 5. 资料链接

- DM-MC02 开发板资料：<https://gitee.com/kit-miao/dm-mc02>
- hexmeow 官方协议/驱动参考：<https://github.com/hex-meow/hex-motor>
- STM32H723 产品页：<https://www.st.com/en/microcontrollers-microprocessors/stm32h723vg.html>
