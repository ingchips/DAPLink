# ING916 DAPLink 适配

本目录是基于 Arm DAPLink 主线的 ING916 适配工程，目录结构和上游源码均完整保留。

- Keil 工程：`projectfiles/uvision5/ingchips_ing916_if/ingchips_ing916_if.uvprojx`
- 工程目标：`ingchips_ing916_if`
- APP 起始地址：`0x0200F000`
- 发布固件：`../firmware/daplink/`

用于 bootloader U 盘升级时，应选择带 CRC 的输出文件。首次下载和 U 盘升级步骤见仓库根目录 `README.md`。
