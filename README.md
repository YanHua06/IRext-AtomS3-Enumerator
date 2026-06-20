# IRext AtomS3 Lite — 独立枚举器 / Standalone Enumerator

# IRext AtomS3 Lite — 独立枚举器 (IRext-AtomS3-Enumerator)

<p align="center">
  <img src="https://img.shields.io/badge/platform-ESP32S3-blue?logo=espressif" />
<a href="https://www.arduino.cc/">
    <img src="https://img.shields.io/badge/framework-Arduino-teal?logo=arduino" />
  </a>
<a href="https://github.com/irext/irext">
      <img src="https://img.shields.io/badge/IR-IRext-orange" />
  </a>
  <img src="https://img.shields.io/badge/license-MIT-green" />
</p>

---

不依赖电脑，AtomS3 Lite 单独开 AP，连上后弹出黑底绿字 Web 终端，
配合板载按键完成红外码枚举测试。也支持电脑联机推送（模式B）。

## 目录结构

```
irext_standalone/
├── platformio.ini
├── src/
│   ├── main.cpp           主固件
│   └── ir_decode/         IRext 解码引擎（C，无需改动）
├── data/                  LittleFS 分区内容（模式A用，烧录: pio run -t uploadfs）
│   └── README.txt
└── upload_bins.py         电脑端配套工具（准备 data/，或模式B联机推送）
```

## 两种模式（开机自动判断）

**模式A · 独立模式** — `data/` 里有 `.bin` 文件时启用
AtomS3 完全离线工作，断电重启自动继续枚举进度（进度不持久化，重启回到第1条）。

**模式B · 联机模式** — `data/` 为空时启用
电脑用 `upload_bins.py remote` 通过 TCP 实时推送 bin 文件，AtomS3 只负责发射和显示。

## 使用流程

### 1）准备 bin 文件（模式A，推荐外出测试用）

```bash
python3 upload_bins.py prepare \
    --db irext_db_20260519_sqlite3.db \
    --bin-dir irext-binaries_20260519/irext-binaries_20260519 \
    --brand 格力 --cat 1 \
    --out data
```

`cat <填入通过db数据库查到的类别id，例如空调是1、TV是2>`

需要将数据库文件放在py脚本同目录下

会把匹配的 bin 复制到 `data/`，重命名为 `<catId>_<subCat>_<序号>.bin`（如 `2_1_001.bin`）。
LittleFS 对文件名长度有限制（约28字节），原始 protocol/remote 名称不再放进文件名，
完整对照表写在 `_manifest.txt`（在项目根目录，不在 `data/` 里，不会占用 flash 空间）。

**换品牌或重新生成前，先清空 `data/` 目录**，避免新旧文件混在一起：

```bash
# Windows
del data\*.bin

# 或直接删文件夹重建
rmdir /s /q data
mkdir data
```

### 2）烧录

```bash
pio run -t upload      # 烧主程序
pio run -t uploadfs    # 烧 data/ 到 LittleFS（重要，容易漏）
```

### 3）外出测试

1. 给 AtomS3 供电（充电宝即可）
2. 手机连 Wi-Fi：`IRext-Enumerator` / 密码 `12345678`
3. 大多数手机会自动弹出 Web 终端；没弹出就手动打开 `http://192.168.4.1`
4. 对着空调，按板载按键：
   - **短按** = 发射当前条目（第一次按会自动 start）
   - **长按**（>0.6s）= 标记命中并跳下一条
5. 或者在 Web 终端 / 串口输入命令操作：

```
start              开始枚举
next / n           发射当前并跳下一条
hit / h            标记当前命中并跳下一条
resend / r         重发当前（没反应时再试一次）
skip / s           跳过不发射，直接下一条
status             查看进度
list               列出所有bin及命中标记
goto <n>           跳到第n条
setac <温度> <模式> <风速>   切到空调指令（模式: 0冷1热2自动）
settv <keycode>    切到TV/按键指令
setcat <cat> <sub> 手动覆盖 category/sub_category
```

6. 枚举结束后 Web 终端会打印所有命中的文件名，记下来即可。

### 4）（可选）不烧录，电脑联机测试

不想每次重新 `uploadfs`，可以先用模式B在电脑上快速试：

```bash
python3 upload_bins.py remote \
    --db irext_db_20260519_sqlite3.db \
    --bin-dir irext-binaries_20260519/irext-binaries_20260519 \
    --brand TCL --cat 0 \
    --host 192.168.4.1 --type tv --keycode 0
```

电脑需连上 AtomS3 的 AP（或同一局域网），运行后逐条推送，手动确认。

## 注意事项

- **LittleFS 容量限制**：AtomS3 Lite 默认分区约 1.4MB，一次性塞太多品牌的 bin 可能放不下，建议按品牌分批 `prepare`
- **文件管理**：`prepare` 会清空重名文件，不会清空 `data/` 里其他已有文件；换品牌测试前建议先手动清空 `data/` 目录再 `prepare`
- **Wi-Fi 模式**：AP 模式下 AtomS3 不连接外部 Wi-Fi，断网测试期间无法用原来的 `irext_client.py`（那个脚本依赖 AtomS3 主动连接你的路由器）

## 常见编译错误

### avr/pgmspace.h 找不到

**错误信息：**

```bash
In file included from path/to/file.ino:2:
path/to/MPU6050.h:45:10: fatal error: avr/pgmspace.h: No such file or directory
45 | #include <avr/pgmspace.h>
|          ^~~~~~~~~~~~~~~~
compilation terminated.
```

**原因：**

- `avr/pgmspace.h` 是 AVR 平台（如 Arduino Uno）专用的头文件，用于访问程序存储器（Flash）
- ESP32 平台使用不同的工具链，`pgmspace.h` 文件位于 `esp32/cores/esp32/` 目录下
- 库代码直接包含 `<avr/pgmspace.h>` 会导致 ESP32 编译失败

**解决方案：**

1. 找到报错的文件（如 `MPU6050.h`）
2. 将以下行：
   ```cpp
   #include <avr/pgmspace.h>
   ```
   修改为：
   ```cpp
   #include <pgmspace.h>
   ```
3. 保存文件并重新编译