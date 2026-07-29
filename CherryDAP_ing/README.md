# CherryDAP ING916 适配

本目录是基于 CherryDAP、CherryUSB 和 FreeRTOS 的 ING916 调试器 APP，特点是 USB 初始化和设备枚举速度更快。

- Keil 工程：`projectfiles/CherryDAP_ing.uvprojx`
- 链接脚本：`projectfiles/CherryDAP_ing.sct`
- APP 起始地址：`0x0200F000`
- 发布固件：`../firmware/cherrydap/`

首次使用时先下载 `../firmware/bl/ing916_bl.bin`，再下载本 APP。完整下载和 U 盘升级步骤见仓库根目录 `README.md`。
