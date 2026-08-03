# 项目记忆

## 项目概况

- 项目目录：`C:/Espressif/frameworks/esp-idf-v5.1.2/video-wifi/esp32Board_wifi`。
- 这是独立的 ESP-IDF v5.1.2 ESP32-S3 摄像头视频图传工程，公共组件位于上一级 `../components/`。
- 摄像头网页源码位于 `../components/modules/web/www/`，HTML 经 gzip 压缩后嵌入固件。
- 当前硬件使用 GC2145 摄像头，主要网页为 `../components/modules/web/www/index_gc2145.html`。

## 工作约定

- 默认使用中文交流，新增代码注释和项目文档使用中文。
- 修改网页后必须同步重新生成对应的 `.html.gz`，再编译和烧录固件。
- 未经明确要求不提交、推送或发布代码。
- 临时产物统一放入 `codex-work/`，任务结束前清理无用文件。

## 当前任务

- 2026-08-03 已将 GC2145 网页的单帧图片保存功能改为电脑浏览器端 MP4 视频录制。
- 页面按钮为 `Record MP4`，第一次点击开始录制并显示计时，再次点击停止并下载带时间戳文件名的 `.mp4` 文件。
- 录制以 20 FPS 将 MJPEG 画面绘制到 Canvas，再由浏览器 `MediaRecorder` 输出 MP4/H.264。
- 实现只选择 `video/mp4` MIME，不回退 WebM；下载前还会检查文件头的 `ftyp` MP4 标识，避免仅修改扩展名。
- 浏览器不支持 MP4 `MediaRecorder` 时会明确提示升级或更换浏览器，不会生成伪 MP4 文件。
- `main/src/app_myhttpd.cpp` 已为根页面增加禁止缓存响应头，避免烧录后继续显示旧的 `Save` 页面。
- 已重新生成 `../components/modules/web/www/index_gc2145.html.gz`，并校验解压内容与 HTML 完全一致。
- 已通过 Node.js JavaScript 语法检查和 ESP-IDF 完整构建；固件为 `build/Camera_Display.bin`，大小 `0xf45a0` 字节，最小应用分区剩余 74%。
- 当前桌面会话没有可控制的浏览器实例，因此尚未完成真实浏览器 MP4 编码兼容性实测；页面已包含严格运行时能力检查。
- 本次未烧录固件，避免覆盖开发板上现有的颜色识别固件。
- 2026-08-03 实机复查时，电脑已连接 `Yahboom_ESP32_WIFI`，`192.168.4.1` 返回的新页面含 8 个 MP4 标识且不含任何 WebM 标识，响应同时带有 `Cache-Control: no-store`。
- 烧录新固件后，烧录前已经打开的旧网页标签页仍会继续运行内存中的旧 WebM JavaScript。必须关闭旧标签页后重新打开，或使用 `Ctrl+F5`/带版本查询参数的新 URL 强制载入新页面。
