# 项目记忆

## 项目概况

- 项目根目录：`C:\Espressif\frameworks\esp-idf-v5.1.2\video-wifi`
- Git 仓库，基于 ESP-IDF v5.1.2。
- 当前主要目录：`components/`、`esp32Board_wifi/`。
- 实际 ESP-IDF 应用目录：`esp32Board_wifi/`，公共组件位于根目录的 `components/`。
- 当前硬件使用 GC2145 摄像头，主要网页源码为 `components/modules/web/www/index_gc2145.html`。
- 网页修改后必须同步重新生成同目录的 `index_gc2145.html.gz`，固件通过 `components/modules/CMakeLists.txt` 嵌入该压缩资源。

## 已完成修改

- 2026-08-21：GC2145 网页的 ROI Y 和 ROI Height 均新增 `- / +` 单步按钮，每次调整 1，并复用现有 `/control` 下发逻辑及 `Y + Height <= 240` 联动边界。
- 2026-08-21：从 GC2145 网页移除 Face Detection、Face Recognition、Color Bar、BPC、WPC、Raw GMA，同时移除无入口的 Enroll Face 按钮和相关前端脚本。
- 2026-08-21：GC2145 网页 Exposure 控件范围改为驱动实际支持的 `1~4095`，增加 `- / +` 单步按钮和当前数值显示；按钮及滑块均通过 `/control?var=aec_value&val=<值>` 实时下发。
- 已同步生成 `components/modules/web/www/index_gc2145.html.gz`，并验证解压内容与 HTML 完全一致。
- 已用 Edge 无头渲染验证：曝光状态 `1024` 正确同步，按钮 `1024 -> 1025`、滑块下发 `768`，ROI Y `70 -> 71`，移动端控件无重叠，JavaScript 异常数为 0。

## 维护说明

- 代码修改后由用户执行 ESP-IDF 编译、烧录和实机验证，除非用户明确要求 Codex 操作。

## GC2145 曝光控制

- 启动时在 `esp32Board_wifi/main/src/yahboom_camera.c` 中关闭自动曝光；当前工作区已将 `GC2145_MANUAL_EXPOSURE` 从 `0x04e2` 调低为 `0x0400`，数值越小，曝光越低。
- 驱动实现在 `components/yahboom_esp32-camera/sensors/gc2145.c`：page 0 寄存器 `0xb6` bit0 控制 AEC，`0x03/0x04` 保存 12 位手动曝光值，有效范围为 `1~0x0fff`。
- 网页 `Exposure` 滑块通过 `/control?var=aec_value&val=<值>` 调用同一驱动接口；修改手动曝光前应保持 `AEC SENSOR` 关闭。
