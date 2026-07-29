# ING916 DAP 固件仓库

本仓库由三个相互独立的固件工程和一个统一发布目录组成。首次使用时必须先下载 bootloader，再下载其中一套 APP；后续可以进入 U 盘模式升级 APP。

## 仓库结构

| 目录 | 内容 | 特点 |
| --- | --- | --- |
| `DAPLink/` | 基于 Arm DAPLink 主线的 ING916 适配 | 完整 DAPLink 功能 |
| `CherryDAP_ing/` | 基于 CherryDAP、CherryUSB 和 FreeRTOS 的 ING916 适配 | USB 初始化和设备枚举更快 |
| `bl/` | ING916 bootloader | 支持通过 U 盘拖拽 HEX/BIN 文件升级 APP |
| `firmware/` | 可直接下载的固件 | 按 bootloader、DAPLink APP、CherryDAP APP 分类 |

三个工程的源码、依赖和 Keil 工程文件均位于各自目录内，互不混杂。

## 固件文件

- Bootloader：`firmware/bl/ing916_bl.bin`
- DAPLink APP：`firmware/daplink/ingchips_ing916_if_crc.bin` 或 `firmware/daplink/ingchips_ing916_if_crc.hex`
- CherryDAP APP：`firmware/cherrydap/CherryDAP_ing.bin` 或 `firmware/cherrydap/CherryDAP_ing.hex`

两套 APP 二选一。APP 的链接起始地址均为 `0x0200F000`，不要把 APP 下载到 bootloader 区域。

## 首次下载

1. 连接调试器的 SWD/J-Link 接口。
2. 先下载 `firmware/bl/ing916_bl.bin`。使用裸 BIN 下载时，起始地址为 `0x02000000`；也可以打开 `bl/ing916__bl.uvprojx` 在 Keil 中下载。
3. 再从 DAPLink 或 CherryDAP 中选择一套 APP 下载：
   - HEX 文件自带地址信息，可直接下载。
   - 裸 BIN 文件的下载起始地址为 `0x0200F000`。
4. 复位或重新上电，确认调试器正常枚举。

必须保持“先 bootloader，后 APP”的顺序。只下载 APP 时，设备不能使用 bootloader 的 U 盘升级功能。

## U 盘升级 APP

1. 断开设备电源或 USB。
2. 将调试口的 `TX` 短接到 `VCC`。
3. 保持短接并重新连接 USB 或复位设备，电脑会出现 bootloader U 盘。
4. 将新的 APP `.hex` 或 `.bin` 文件拖入该 U 盘。一次只复制一个固件文件。
5. 等待写入完成和设备复位，不要在复制过程中断电或拔出 USB。
6. 移除 `TX` 与 `VCC` 的短接，再次复位或重新上电，运行新的 APP。

U 盘升级只更新从 `0x0200F000` 开始的 APP 区域，不需要重复下载 bootloader。

## 编译入口

### 主线 DAPLink 适配

```text
DAPLink/projectfiles/uvision5/ingchips_ing916_if/ingchips_ing916_if.uvprojx
```

工程目标为 `ingchips_ing916_if`。用于 bootloader 升级时，应使用带 CRC 的输出文件。上游项目说明和开发资料分别见 `DAPLink/README.md` 与 `DAPLink/docs/`。

### CherryDAP 适配

```text
CherryDAP_ing/projectfiles/CherryDAP_ing.uvprojx
```

链接脚本将 APP 放在 `0x0200F000`。该版本适合需要更快 USB 初始化和枚举速度的场景。

### Bootloader

```text
bl/ing916__bl.uvprojx
```

重新编译后，应先通过 SWD/J-Link 下载 bootloader，再下载其中一套 APP。
