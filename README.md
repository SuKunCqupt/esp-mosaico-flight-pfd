# 飞行表 · ESP32-S31 自制飞机姿态仪（PFD）

在 ESP-Mosaico 开发板（ESP32-S31，480×480 AMOLED 圆角屏）上实现的自制「主飞行显示器」（PFD）：
人工地平仪、俯仰梯、横滚刻度、航向与罗盘、G 值表，并可通过 Wi-Fi 让手机把 GPS 位置、高度、速度送到板子上，
显示在速度带 / 高度带上。

> **English summary.** A homemade primary flight display (artificial horizon, pitch ladder, roll arc,
> heading/compass from the on-board BMM150 magnetometers, G meter) for the ESP-Mosaico board
> (ESP32-S31, 480×480 AMOLED, BMI270 IMU). A phone can stream GPS position/altitude/speed to the board
> over the board's own Wi-Fi hotspot through an HTTPS web page (no app needed). Built on the
> ESP-Mosaico SDK with ESP-IDF 6.2. MIT licensed. **Not for real navigation.**

⚠️ 这是玩具 / 学习项目，**不能用于真实飞行或导航**。

## 功能

- **姿态**：BMI270 六轴 + BMM150 磁力计，200 Hz Mahony 姿态融合（独立核心运行），屏幕刷新约 37 帧/秒。
- **人工地平仪**：天空 / 地面、俯仰梯（每 5°）、横滚刻度弧与指针、飞机符号。
- **航向**：顶部航向框 + 底部罗盘（磁航向，需做一次磁力计校准）。
- **G 值表**：当前 G 值与峰值 G（菜单里可清零）。
- **手机定位（GPS）**：板子开一个 WPA2 热点，手机连上后打开板子自带的 HTTPS 网页，
  浏览器把定位（经纬度、海拔、速度、航迹）每秒发给板子，显示在速度带、高度带和航迹菱形上；断开 3 秒提示「信号丢失」。
- **菜单**（长按屏幕打开）：水平校准、显示方向（竖直 / 平放）、磁力计校准、手机定位、峰值 G 清零、亮度、模拟数据。
- **校准**：开机自动陀螺零偏；「水平校准」（背面朝下平放 2 秒，修正 IMU 安装误差）；「磁力计校准」（转动板子约 20 秒）。结果都保存在 NVS 里。
- 全中文界面（Noto Sans CJK 位图字体，构建时生成）。

## 硬件

- ESP-Mosaico 开发板：ESP32-S31、16 MB Flash、PSRAM、480×480 CO5300 QSPI AMOLED、CST9220 触摸、
  BMI270 IMU、两颗 BMM150 磁力计。
- 一部带 GPS 的手机（iPhone Safari 或 Android Chrome），用于手机定位功能（可选）。

## 目录

```
main/                      Engine 入口
components/app_fusion/     姿态融合（四元数、Mahony）、磁力计校准、水平校准数学
components/app_sensors/    BMI270 / BMM150 驱动与 200 Hz 传感器任务（sensors_host.c 为电脑模拟）
components/app_pfd/        绘图：地平仪、刻度带、罗盘、菜单、二维码（qrcodegen）、字体
components/app_link/       手机 GPS：Wi-Fi 热点 + HTTPS 网页（page/index.html）
components/app_game_runtime/  输入、菜单、提示、日志，把各模块组合起来
tests/host/                电脑上运行的单元测试与截图渲染测试
tools/gen_cert.sh          生成 HTTPS 证书（构建时自动调用）
tools/gen_font.py          重新生成中文字体（改了界面文字后运行）
```

## 构建与烧录

本项目是 **ESP-Mosaico SDK 工作区里的一个项目**，依赖 SDK 里的 BSP、Raylib Lite 引擎和 ESP-Iris
系统更新工具（这些不在本仓库里，请从 SDK 获取）。

1. 按 ESP-Mosaico SDK（<https://github.com/esp-mosaico/esp-mosaico-vibe>）的说明克隆工作区（含子模块），
   安装它指定版本的 ESP-IDF（6.2）和 Python 环境。
2. 把本仓库克隆到工作区的 `projects/flight_pfd`（目录层级必须是 `projects/<名字>`，CMake 里用相对路径找 SDK）：
   ```sh
   cd <工作区>
   git clone https://github.com/SuKunCqupt/esp-mosaico-flight-pfd.git projects/flight_pfd
   ```
3. 第一次构建会下载依赖（`dependencies.lock` 已固定版本），然后因缺少 `gen_bmgr_codes` 停下，
   这是正常的；生成板级配置后再构建一次：
   ```sh
   source <esp-idf>/export.sh
   python mosaico.py game build --project projects/flight_pfd      # 第一次：拉依赖，会报 gen_bmgr_codes 缺失
   P="$PWD/projects/flight_pfd"
   IDF_EXTRA_ACTIONS_PATH="$P/managed_components/espressif__esp_board_manager" \
     idf.py -C "$P" bmgr -c "$PWD/submodule/raylib-lite-engine/examples/boards/esp-mosaico/bmgr" -b esp_mosaico
   python mosaico.py game build --project projects/flight_pfd      # 第二次：正式构建
   ```
   第一次配置时会自动运行 `tools/gen_cert.sh`（需要 `openssl`），在 `components/app_link/certs/`
   生成你自己的 HTTPS 证书（该目录已在 `.gitignore` 中，**不要提交**）。
4. 打包并用 ESP-Iris 系统更新方式写入（只更新分区表和应用分区，不动 bootloader / 恢复分区）：
   ```sh
   idf.py -C projects/flight_pfd system-update-bundle
   python mosaico.py iris system-update --project projects/flight_pfd
   ```
   全新或状态不明的板子，请先按 SDK 文档运行一次 `python mosaico.py recover`。
5. 看日志：`python mosaico.py iris logs --project projects/flight_pfd`，每秒一行 `att roll=… pitch=… hdg=…`。

电脑上模拟运行（不需要板子）：
```sh
python mosaico.py game sim --project projects/flight_pfd
```

单元测试：
```sh
cd projects/flight_pfd/tests/host
cc -O2 -Wall -I../../components/app_fusion/include -o /tmp/test_fusion test_fusion.c \
   ../../components/app_fusion/fusion.c ../../components/app_fusion/magcal.c -lm && /tmp/test_fusion
cc -O2 -Wall -I../../components/app_link/include -o /tmp/test_link test_link.c \
   ../../components/app_link/link_core.c -lm && /tmp/test_link
```

## 使用

- **拿法**：默认「竖直」——屏幕朝向自己，屏幕法线方向就是机头方向。菜单「显示方向」可切换成「平放」（背面朝下，屏幕上边是机头）。
- **长按屏幕**打开菜单，点按选择。

### 校准

1. **陀螺零偏**：每次开机自动进行，开机时请让板子静止约 1 秒。
2. **水平校准**（第一次使用时做一次，结果永久保存）：
   1. 把板子**背面朝下平放在水平桌面上**；
   2. 菜单 →「水平校准（背面朝下平放）」；
   3. 保持不动约 2 秒（晃动会提示「检测到晃动，请放稳」并重新计时）；
   4. 提示「校准完成，请竖直拿起」后竖直拿起即可。偏离水平超过 15° 会提示「未平放，请重试」。
3. **磁力计校准**（要用航向时做一次）：菜单 →「磁力计校准」，拿起板子慢慢朝各个方向转动（翻过来、竖起来、侧过来），
   约 20 秒覆盖够了会自动完成。远离磁铁、手机、电脑和金属桌面。

### 手机定位（GPS）

1. 菜单 →「手机定位（GPS）」。屏幕显示第一个二维码：板子的热点 `飞行表-XXXX`（密码首次开机随机生成并保存）。
   用手机相机扫码连接，或在 Wi-Fi 列表里手动连接。
2. 点「下一步」，扫第二个二维码打开 `https://192.168.4.1`。
3. 浏览器会提示证书不受信任（证书是你自己构建时生成的自签名证书）：
   - **iPhone（Safari）**：点「显示详细信息 → 访问此网站」。如果点「开始定位」后没有弹出权限窗口，
     按网页里「iPhone 安装证书」的步骤装一次证书：下载证书 → 设置 → 通用 → VPN与设备管理 → 安装 →
     设置 → 通用 → 关于本机 → 证书信任设置 → 打开「FlightPFD Local CA」。
     另外确认 设置 → 隐私与安全性 → 定位服务 已打开，「Safari 浏览器网站」为「使用 App 期间」，并打开「精确位置」。
   - **Android（Chrome）**：点「高级 → 继续前往」。手机设置里打开「位置信息」，Chrome 询问位置权限时选「允许」。
4. 在网页上点「开始定位」并保持网页在前台（锁屏或切到后台后浏览器会停止发送定位）。
   板子提示「手机定位已连接」，速度带和高度带开始显示数据。

> 为什么用 HTTPS：浏览器只在安全上下文（HTTPS）里提供定位接口。证书的 CA 带名称约束，只能用于
> 192.168.4.1、iPhone 热点网段 172.20.10.x 和 `.local`，CA 私钥签完即销毁。
> 网页里还可以选填一个 Wi-Fi，让板子同时连到该网络（可选）。

## 已知限制

- **没有气压计**：高度、速度完全来自手机 GPS，精度一般（高度常有 ±10 m 以上误差），更新约 1 次/秒。
- 手机网页必须保持在前台；浏览器后台不会发送定位。
- 水平校准只能修正「前后 / 左右倾斜」方向的安装误差；绕屏幕法线的转动在平放姿态下观测不到，若存在会表现为固定的横滚偏差。桌面本身必须水平。
- 磁航向受附近磁铁、金属、电脑影响；磁力计饱和时会自动丢弃该样本。
- 竖直姿态俯仰接近 ±90° 时欧拉角会有奇异，航向显示会跳。
- 刷新率受屏幕 QSPI 带宽限制，约 37 帧/秒。

## 路线图

- 外接气压计（如 BMP390 / MS5611）：气压高度、升降速度（VSI）。
- 外接 UART GPS 模块（NMEA / UBX），不再依赖手机。
- 更完整的水平校准（第二个姿态，修正绕屏幕法线的误差）。
- 记录飞行数据到 Flash，电脑上回放。

## 许可证

MIT，见 [LICENSE](LICENSE)。第三方部分（qrcodegen：MIT；Noto Sans CJK 字体位图：SIL OFL 1.1；
来自 ESP-Mosaico 空白模板的入口文件：Apache-2.0）保留各自的许可证。
