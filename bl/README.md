# ING916 Bootloader

本目录包含 ING916 DAPLink bootloader 源码和 Keil 工程。

- Keil 工程：`ing916__bl.uvprojx`
- Bootloader 起始地址：`0x02000000`
- APP 起始地址：`0x0200F000`
- 发布固件：`../firmware/bl/ing916_bl.bin`

设备上电时，将调试口 `TX` 短接到 `VCC` 可进入 U 盘升级模式。把链接到 `0x0200F000` 的 APP HEX 或 BIN 文件拖入 U 盘即可更新 APP。完整步骤见仓库根目录 `README.md`。
