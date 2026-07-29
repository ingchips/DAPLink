# 发布固件

本目录集中存放可直接下载的固件：

- `bl/`：bootloader，首次下载到 `0x02000000`。
- `daplink/`：基于主线 DAPLink 的 APP，下载到 `0x0200F000`。
- `cherrydap/`：基于 CherryDAP 的快速初始化 APP，下载到 `0x0200F000`。

先下载 bootloader，再从两套 APP 中选择一套下载。详细步骤见仓库根目录 `README.md`。
