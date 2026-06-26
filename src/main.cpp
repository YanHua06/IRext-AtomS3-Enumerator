/**
 * IRext AtomS3 Lite — 独立红外枚举器
 *
 * 两种工作模式（开机自动检测）：
 *
 * 【模式A · 独立模式】LittleFS 中有 .bin 文件时启用
 *   - 读取 /data/*.bin，按顺序枚举发射
 *   - AtomS3 按键：单击=发射当前并下一条，长按=标记命中
 *   - Web 终端 / 串口：查看状态、输入命令控制枚举
 *
 * 【模式B · 终端模式】LittleFS 为空时启用
 *   - 等待电脑端 upload_bins.py 通过 TCP 推送 bin + 指令
 *   - AtomS3 收到后发射，结果回显到 Web 终端和串口
 *
 * 【Web 终端】连接 AP "IRext-Enumerator"（密码 12345678）
 *   后自动弹出，黑底绿字 Linux 风格
 *   支持命令：next / hit / skip / status / restart / loglevel <0-3>
 *
 * 【串口命令】同 Web 终端命令，115200 baud
 *
 * 【AP 信息】
 *   SSID: IRext-Enumerator
 *   密码: 12345678
 *   IP:   192.168.4.1
 */

#include <Arduino.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <IRremote.hpp>
#include <ArduinoJson.h>
#include <Base64.h>
#include <vector>
#include <algorithm>
#include <cstdarg>

extern "C" {
  #include "ir_decode/include/ir_decode.h"
  #include "ir_decode/include/ir_ac_control.h"
  #include "ir_decode/include/ir_defs.h"
}

// ════════════════════════════════════════════════════════
// 配置
// ════════════════════════════════════════════════════════
#define IR_PIN_INTERNAL  4       // G4  - 内部红外 LED
#define IR_PIN_EXTERNAL  6       // G6  - 外置红外发射模块
#define IR_RECV_PIN      39      // G39 - 外置红外接收头

#define BTN_PIN         41        // AtomS3 Lite 内置按键
#define AP_SSID         "IRext-Enumerator"
#define AP_PASS         "12345678"
#define AP_IP           192,168,4,1
#define TCP_PORT        8080
#define WEB_PORT        80
#define DNS_PORT        53
#define LOG_BUF_MAX     128       // 终端历史行数
#define LOG_LINE_LEN    160
#define LEARN_TIMEOUT_MS 30000    // 学习模式超时 30 秒
#define LEARNED_DIR      "/learned"
#define LEARNED_INDEX    "/learned/index.json"
#define MAX_LEARNED      50

// 默认测试指令（枚举时发射）
// 空调：开机26℃制冷；电视：电源键keycode=0
static int  g_testKeycode  = 0;
static bool g_testIsAc     = true;   // true=空调模式 false=TV/通用按键模式
static int  g_testTemp     = 26;
static int  g_testMode     = 0;
static int  g_testFan      = 0;
static int  g_testPower    = 0;      // 0=开机 1=关机
static int  g_testSwing    = 0;      // 0=扫风开 1=扫风关
static int  g_acFuncCode   = 0;      // ir_decode 第1参数: 0=电源/完整帧 9=风速 10=扫风 1=模式
static int  g_testCat      = 1;      // category_id
static int  g_testSubcat   = 1;      // sub_category

// ════════════════════════════════════════════════════════
// 全局对象
// ════════════════════════════════════════════════════════
AsyncWebServer  webServer(WEB_PORT);
AsyncWebSocket  ws("/ws");
DNSServer       dnsServer;
WiFiServer      tcpServer(TCP_PORT);

// IR 输出选择 & 学习状态
static int            g_irSendPin    = IR_PIN_INTERNAL;
static bool           g_learnMode    = false;
static bool           g_learnCaptured = false;
static uint16_t       g_learnFreq    = 38000;
static std::vector<uint16_t> g_learnRawBuf;  // 捕获的微秒时序
static unsigned long  g_learnStartMs = 0;     // 学习超时计时

// ════════════════════════════════════════════════════════
// 日志 / 终端输出（串口 + WebSocket 广播）
// ════════════════════════════════════════════════════════
static std::vector<String> g_termLog;  // 滚动历史

void termPrint(const char* fmt, ...) {
  char buf[LOG_LINE_LEN];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  Serial.println(buf);

  String line = String(buf);
  g_termLog.push_back(line);
  if ((int)g_termLog.size() > LOG_BUF_MAX) g_termLog.erase(g_termLog.begin());

  // 广播给所有 WebSocket 客户端
  ws.textAll(line + "\n");
}

// ════════════════════════════════════════════════════════
// IR 发射核心
// ════════════════════════════════════════════════════════
static unsigned char* g_binData    = nullptr;
static int            g_binLen     = 0;
static uint16_t       g_rawBuf[2048];

bool irLoadBin(const uint8_t* data, int len, int cat, int subcat) {
  if (g_binData) { free(g_binData); g_binData = nullptr; }
  g_binData = (unsigned char*)malloc(len);
  if (!g_binData) { termPrint("[E] malloc 失败"); return false; }
  memcpy(g_binData, data, len);
  g_binLen = len;
  if (IR_DECODE_FAILED == ir_binary_open(cat, subcat, g_binData, g_binLen)) {
    termPrint("[E] ir_binary_open 失败 cat=%d sub=%d", cat, subcat);
    free(g_binData); g_binData = nullptr;
    return false;
  }
  return true;
}

bool irSend() {
  if (!g_binData) { termPrint("[E] 未加载 bin"); return false; }

  uint16_t rawLen = 0;
  if (g_testIsAc) {
    t_remote_ac_status ac = {};
    ac.ac_power      = (t_ac_power)g_testPower;
    ac.ac_temp       = (t_ac_temperature)(g_testTemp - 16);
    ac.ac_mode       = (t_ac_mode)g_testMode;
    ac.ac_wind_speed = (t_ac_wind_speed)g_testFan;
    ac.ac_wind_dir   = (t_ac_swing)g_testSwing;
    rawLen = ir_decode(g_acFuncCode, g_rawBuf, &ac);
  } else {
    rawLen = ir_decode(g_testKeycode, g_rawBuf, nullptr);
  }

  if (rawLen == 0) { termPrint("[E] ir_decode 返回空"); return false; }

  // 切换到指定输出引脚
  if (IrSender.sendPin != g_irSendPin) {
    IrSender.setSendPin(g_irSendPin);
  }

  // 学习模式下暂停接收，避免捕获自身发射的红外
  if (g_learnMode) IrReceiver.stop();

  IrSender.sendRaw(g_rawBuf, rawLen, 38);

  // 恢复接收
  if (g_learnMode) {
    delay(50);
    IrReceiver.start();
  }
  return true;
}

// ════════════════════════════════════════════════════════
// 枚举状态机
// ════════════════════════════════════════════════════════
enum EnumState { IDLE, RUNNING, PAUSED, DONE };

static EnumState        g_enumState  = IDLE;
static std::vector<String> g_binList;   // LittleFS bin 文件路径列表
static int              g_enumIdx    = 0;
static std::vector<int> g_hits;         // 命中的索引

void enumLoadBinList() {
  g_binList.clear();
  File root = LittleFS.open("/");
  File f = root.openNextFile();
  while (f) {
    String name = f.name();
    if (name.endsWith(".bin")) {
      g_binList.push_back("/" + name);
    }
    f = root.openNextFile();
  }
  // 排序
  std::sort(g_binList.begin(), g_binList.end());
  termPrint("[*] LittleFS 中找到 %d 个 .bin 文件", (int)g_binList.size());
}

bool enumLoadCurrent() {
  if (g_enumIdx >= (int)g_binList.size()) return false;
  const String& path = g_binList[g_enumIdx];
  File f = LittleFS.open(path, "r");
  if (!f) { termPrint("[E] 无法打开 %s", path.c_str()); return false; }
  int sz = f.size();
  uint8_t* buf = (uint8_t*)malloc(sz);
  if (!buf) { termPrint("[E] malloc 失败"); f.close(); return false; }
  f.read(buf, sz);
  f.close();

  // 文件名格式（upload_bins.py prepare 生成）: /<catId>_<subCat>_<protocol>_<remote>.bin
  // 解析失败则退回全局 g_testCat/g_testSubcat
  int cat = g_testCat, sub = g_testSubcat;
  int u1 = path.indexOf('_');
  int u2 = (u1 >= 0) ? path.indexOf('_', u1 + 1) : -1;
  if (u1 > 0 && u2 > u1) {
    // path 以 '/' 开头，substring(1,u1) 跳过前导斜杠
    String catStr = path.substring(1, u1);
    String subStr = path.substring(u1 + 1, u2);
    if (catStr.length() && subStr.length()) {
      cat = catStr.toInt();
      sub = subStr.toInt();
    }
  }

  bool ok = irLoadBin(buf, sz, cat, sub);
  free(buf);
  if (ok) termPrint("[%d/%d] 加载: %s (cat=%d sub=%d)", g_enumIdx+1, (int)g_binList.size(), path.c_str(), cat, sub);
  return ok;
}

void enumStart() {
  if (g_binList.empty()) { termPrint("[E] 无 bin 文件，请先上传到 LittleFS"); return; }
  g_enumIdx  = 0;
  g_hits.clear();
  g_enumState = RUNNING;
  termPrint("[*] 开始枚举，共 %d 个文件", (int)g_binList.size());
  termPrint("[*] 按键单击=发射并下一条  长按=标记命中  串口/Web输入命令");
  if (enumLoadCurrent()) irSend();
}

void enumNext() {
  if (g_enumState != RUNNING && g_enumState != PAUSED) return;
  g_enumIdx++;
  if (g_enumIdx >= (int)g_binList.size()) {
    g_enumState = DONE;
    termPrint("[*] 枚举完成！命中 %d 个：", (int)g_hits.size());
    for (int h : g_hits) termPrint("    ✓ #%d %s", h+1, g_binList[h].c_str());
    return;
  }
  g_enumState = RUNNING;
  if (enumLoadCurrent()) irSend();
}

void enumMarkHit() {
  if (g_enumState != RUNNING && g_enumState != PAUSED) {
    termPrint("[W] 当前不在枚举中"); return;
  }
  g_hits.push_back(g_enumIdx);
  termPrint("🎯 命中！#%d %s", g_enumIdx+1, g_binList[g_enumIdx].c_str());
}

void enumResend() {
  if (g_enumState != RUNNING && g_enumState != PAUSED) return;
  termPrint("[*] 重发 #%d", g_enumIdx+1);
  irSend();
}

// ════════════════════════════════════════════════════════
// IR 学习 — 接收 & 存储已学码
// ════════════════════════════════════════════════════════

struct LearnedEntry {
  int id;
  String name;
  String file;
  uint16_t frequency;
};

static std::vector<LearnedEntry> g_learnedList;
static int g_nextLearnId = 1;

void learnLoadIndex() {
  g_learnedList.clear();
  g_nextLearnId = 1;

  if (!LittleFS.exists(LEARNED_INDEX)) return;

  File f = LittleFS.open(LEARNED_INDEX, "r");
  if (!f) return;

  String content = f.readString();
  f.close();

  JsonDocument doc;
  if (deserializeJson(doc, content) != DeserializationError::Ok) return;

  for (JsonVariant entry : doc.as<JsonArray>()) {
    LearnedEntry e;
    e.id        = entry["id"] | 0;
    e.name      = entry["name"] | "";
    e.file      = entry["file"] | "";
    e.frequency = entry["frequency"] | 38000;
    g_learnedList.push_back(e);
    if (e.id >= g_nextLearnId) g_nextLearnId = e.id + 1;
  }
}

void learnSaveIndex() {
  JsonDocument doc;
  for (auto& e : g_learnedList) {
    JsonObject obj = doc.add<JsonObject>();
    obj["id"]        = e.id;
    obj["name"]      = e.name;
    obj["file"]      = e.file;
    obj["frequency"] = e.frequency;
  }

  File f = LittleFS.open(LEARNED_INDEX, "w");
  if (f) {
    serializeJson(doc, f);
    f.close();
  }
}

bool learnSaveRaw(const String& filePath, uint16_t freqHz) {
  if (g_learnRawBuf.empty()) return false;

  File f = LittleFS.open(filePath, "w");
  if (!f) { termPrint("[E] 无法创建 %s", filePath.c_str()); return false; }

  f.write((uint8_t*)&freqHz, 2);
  uint16_t rawlen = g_learnRawBuf.size();
  f.write((uint8_t*)&rawlen, 2);
  f.write((uint8_t*)g_learnRawBuf.data(), rawlen * 2);
  f.close();
  return true;
}

void learnPollReceiver() {
  if (!g_learnMode) return;

  // 30s 超时检查
  if (millis() - g_learnStartMs > LEARN_TIMEOUT_MS) {
    g_learnMode = false;
    g_learnCaptured = false;
    g_learnRawBuf.clear();
    termPrint("[LEARN] 30秒无信号，已自动退出学习模式");
    return;
  }

  if (IrReceiver.decode()) {
    g_learnCaptured = true;
    g_learnRawBuf.clear();

    // 从 irparams.rawbuf[] 提取 raw tick 值并转换为微秒
    // rawbuf[] 是 uint8_t 的 50µs tick 值 (MICROS_PER_TICK=50)
    uint16_t rawlen = IrReceiver.decodedIRData.rawlen;
    if (rawlen > 0) {
      // rawbuf[0] 是 initial gap, 从 rawbuf[1] 开始是实际的 mark/space 时序
      for (IRRawlenType i = 1; i < rawlen; i++) {
        uint32_t us = (uint32_t)IrReceiver.irparams.rawbuf[i] * MICROS_PER_TICK;
        // 在 uint16_t 范围内 (最大 255*50=12750 µs, 对于大多数遥控器足够)
        g_learnRawBuf.push_back((uint16_t)us);
      }
      g_learnFreq = 38000;  // 默认 38kHz
    }

    termPrint("[LEARN] 捕获！原始长度: %d 个脉冲 (freq=%d Hz)",
              (int)g_learnRawBuf.size(), g_learnFreq);
    termPrint("[LEARN] 输入 learn save <名称> 保存, learn cancel 丢弃");

    IrReceiver.resume();
  }
}

void learnSave(const String& name) {
  if (!g_learnCaptured) {
    termPrint("[E] 没有捕获到的红外码，请先对准接收器按遥控器");
    return;
  }
  if (g_learnRawBuf.empty()) {
    termPrint("[E] 捕获数据为空，请重新学习");
    g_learnCaptured = false;
    return;
  }

  char fileName[32];
  snprintf(fileName, sizeof(fileName), "/learned/ir_%04d.raw", g_nextLearnId);

  if (!learnSaveRaw(fileName, g_learnFreq)) {
    termPrint("[E] 保存文件失败");
    return;
  }

  LearnedEntry entry;
  entry.id        = g_nextLearnId++;
  entry.name      = name;
  entry.file      = String(fileName);
  entry.frequency = g_learnFreq;
  g_learnedList.push_back(entry);
  learnSaveIndex();

  g_learnCaptured = false;
  g_learnRawBuf.clear();
  termPrint("[LEARN] 已保存: #%d \"%s\"", entry.id, name.c_str());
}

void learnDelete(int id) {
  for (auto it = g_learnedList.begin(); it != g_learnedList.end(); ++it) {
    if (it->id == id) {
      LittleFS.remove(it->file);
      g_learnedList.erase(it);
      learnSaveIndex();
      termPrint("[LEARN] 已删除 #%d", id);
      return;
    }
  }
  termPrint("[E] 未找到学习码 #%d", id);
}

bool sendLearnedCode(const String& filePath) {
  File f = LittleFS.open(filePath, "r");
  if (!f) { termPrint("[E] 无法打开 %s", filePath.c_str()); return false; }

  uint16_t freqHz = 0, rawlen = 0;
  if (f.read((uint8_t*)&freqHz, 2) != 2 ||
      f.read((uint8_t*)&rawlen, 2) != 2) {
    termPrint("[E] 读取头失败");
    f.close();
    return false;
  }

  if (rawlen == 0) {
    termPrint("[E] raw 数据为空");
    f.close();
    return false;
  }

  uint16_t* rawBuf = (uint16_t*)malloc(rawlen * 2);
  if (!rawBuf) { termPrint("[E] malloc 失败"); f.close(); return false; }
  if (f.read((uint8_t*)rawBuf, rawlen * 2) != (int)(rawlen * 2)) {
    termPrint("[E] 读取 raw 数据失败");
    free(rawBuf);
    f.close();
    return false;
  }
  f.close();

  // 切换输出引脚
  if (IrSender.sendPin != g_irSendPin) {
    IrSender.setSendPin(g_irSendPin);
  }

  uint16_t freqKHz = freqHz / 1000;
  if (freqKHz == 0) freqKHz = 38;

  if (g_learnMode) IrReceiver.stop();
  IrSender.sendRaw(rawBuf, rawlen, freqKHz);
  if (g_learnMode) { delay(50); IrReceiver.start(); }

  free(rawBuf);
  termPrint("[*] 发射已学码 (freq=%dHz len=%d)", freqHz, rawlen);
  return true;
}

// ════════════════════════════════════════════════════════
// 命令解析（串口 / WebSocket 共用）
// ════════════════════════════════════════════════════════
void printStatus() {
  termPrint("─── 状态 ─────────────────────────────");
  termPrint("模式    : %s", g_binList.empty() ? "B(终端)" : "A(独立)");
  termPrint("枚举    : %s  [%d/%d]",
    g_enumState==IDLE?"IDLE":g_enumState==RUNNING?"RUNNING":g_enumState==PAUSED?"PAUSED":"DONE",
    g_enumIdx+1, (int)g_binList.size());
  termPrint("当前文件: %s", g_binList.empty() ? "-" : (g_enumIdx < (int)g_binList.size() ? g_binList[g_enumIdx].c_str() : "-"));
  termPrint("命中数  : %d", (int)g_hits.size());
  termPrint("指令    : %s cat=%d sub=%d",
    g_testIsAc ? "AC(空调)" : "TV(按键)", g_testCat, g_testSubcat);
  if (g_testIsAc) termPrint("  空调参数: temp=%d mode=%d fan=%d", g_testTemp, g_testMode, g_testFan);
  else            termPrint("  按键码  : %d", g_testKeycode);
  termPrint("AP SSID : %s  IP: 192.168.4.1", AP_SSID);
  termPrint("IR 输出 : %s (GPIO%d)",
    g_irSendPin == IR_PIN_INTERNAL ? "内部" : "外部", g_irSendPin);
  termPrint("学习模式: %s%s",
    g_learnMode ? "开启" : "关闭",
    g_learnCaptured ? " (已捕获，等待保存)" : "");
  termPrint("已学码  : %d 条", (int)g_learnedList.size());
  termPrint("──────────────────────────────────────");
}

void printHelp() {
  termPrint("╔══════════════════════════════════════╗");
  termPrint("║   IRext AtomS3 枚举器 — 命令列表     ║");
  termPrint("╠══════════════════════════════════════╣");
  termPrint("║ 枚举控制                              ║");
  termPrint("║  start        开始枚举                ║");
  termPrint("║  next/n       发射并进入下一条         ║");
  termPrint("║  hit/h        标记命中并跳下一条       ║");
  termPrint("║  resend/r     重发当前（不跳）         ║");
  termPrint("║  skip/s       跳过不发射               ║");
  termPrint("║  goto <n>     跳到第n条               ║");
  termPrint("║  status       显示当前状态             ║");
  termPrint("║  list         列出所有bin及命中标记    ║");
  termPrint("╠══════════════════════════════════════╣");
  termPrint("║ 空调快捷控制（需先枚举命中某bin）      ║");
  termPrint("║  temp <16-30> / temp+ / temp-  调温   ║");
  termPrint("║  fan <0-3>  / fan+  / fan-    调风速  ║");
  termPrint("║    0=自动 1=低 2=中 3=高               ║");
  termPrint("║  swing on/off/（不带参数=切换）调扫风  ║");
  termPrint("║  mode <0-4>/cool/heat/auto/fan/dry    ║");
  termPrint("║  off          发关机指令               ║");
  termPrint("║  setac <t><m><f>[sw][pw] 一次性设全部  ║");
  termPrint("╠══════════════════════════════════════╣");
  termPrint("║ 其他                                  ║");
  termPrint("║  settv <keycode>  切换为TV按键模式     ║");
  termPrint("║  setcat <cat><sub> 手动设category      ║");
  termPrint("║  help         显示此帮助               ║");
  termPrint("╠══════════════════════════════════════╣");
  termPrint("║ IR 输出选择                           ║");
  termPrint("║  ir internal/int  使用内部 IR (GPIO4) ║");
  termPrint("║  ir external/ext  使用外部 IR (GPIO6) ║");
  termPrint("║  ir status        查看当前输出选择    ║");
  termPrint("╠══════════════════════════════════════╣");
  termPrint("║ IR 学习                               ║");
  termPrint("║  learn            进入学习模式        ║");
  termPrint("║  learn save <名称> 保存捕获的码       ║");
  termPrint("║  learn cancel     放弃当前捕获        ║");
  termPrint("║  learn list       列出已学码          ║");
  termPrint("║  learn play <id>  发射已学码          ║");
  termPrint("║  learn delete <id> 删除已学码         ║");
  termPrint("║  learn stop       退出学习模式        ║");
  termPrint("╚══════════════════════════════════════╝");
}

void handleCommand(const String& rawCmd) {
  String cmd = rawCmd;
  cmd.trim();
  if (cmd.length() == 0) return;

  termPrint("> %s", cmd.c_str());

  if (cmd == "start") {
    enumStart();
  } else if (cmd == "next" || cmd == "n") {
    enumNext();
  } else if (cmd == "hit" || cmd == "h") {
    enumMarkHit();
    enumNext();
  } else if (cmd == "resend" || cmd == "r") {
    enumResend();
  } else if (cmd == "skip" || cmd == "s") {
    enumNext();
  } else if (cmd == "status") {
    printStatus();
  } else if (cmd == "list") {
    termPrint("bin 文件列表 (%d):", (int)g_binList.size());
    for (int i = 0; i < (int)g_binList.size(); i++) {
      termPrint("  %d: %s%s", i+1, g_binList[i].c_str(), i==g_enumIdx?" ◀":
        (std::find(g_hits.begin(),g_hits.end(),i)!=g_hits.end())?" ✓":"");
    }
  } else if (cmd == "help") {
    printHelp();
  } else if (cmd.startsWith("goto ")) {
    int n = cmd.substring(5).toInt() - 1;
    if (n >= 0 && n < (int)g_binList.size()) {
      g_enumIdx = n;
      g_enumState = RUNNING;
      if (enumLoadCurrent()) irSend();
    } else {
      termPrint("[E] 超出范围 1~%d", (int)g_binList.size());
    }
  } else if (cmd.startsWith("setac")) {
    // setac <temp> <mode> <fan> [swing=0/1] [power=0/1]
    // 未填的参数保持上次值
    int t=g_testTemp, m=g_testMode, f=g_testFan, sw=g_testSwing, pw=g_testPower;
    sscanf(cmd.c_str(), "setac %d %d %d %d %d", &t, &m, &f, &sw, &pw);
    // 推算本次主要变化的 function code（ir_decode 第1参数）
    // 只要发生变化就用对应的专用 function，都没变则用电源/完整帧(0)
    if      (f  != g_testFan)   g_acFuncCode = 9;   // AC_FUNCTION_WIND_SPEED
    else if (sw != g_testSwing) g_acFuncCode = 10;  // AC_FUNCTION_WIND_SWING
    else if (m  != g_testMode)  g_acFuncCode = 1;   // AC_FUNCTION_MODE
    else if (pw != g_testPower) g_acFuncCode = 0;   // AC_FUNCTION_POWER
    else                        g_acFuncCode = 0;   // 温度或其他，用完整帧
    g_testIsAc = true;
    g_testTemp = t; g_testMode = m; g_testFan = f;
    g_testSwing = sw; g_testPower = pw;
    const char* modeStr[] = {"制冷","制热","自动","送风","除湿"};
    const char* fanStr[]  = {"自动","低","中","高"};
    termPrint("[*] AC: %s %d℃ %s 风速:%s 扫风:%s (func=%d)",
              pw==0?"开机":"关机", t,
              m>=0&&m<=4 ? modeStr[m] : "?",
              f>=0&&f<=3 ? fanStr[f]  : "?",
              sw==0?"开":"关", g_acFuncCode);

  } else if (cmd.startsWith("fan")) {
    // fan+ / fan- / fan <0-3>
    g_testIsAc   = true;
    g_acFuncCode = 9;  // AC_FUNCTION_WIND_SPEED
    if (cmd == "fan+") {
      g_testFan = min(g_testFan + 1, 3);
    } else if (cmd == "fan-") {
      g_testFan = max(g_testFan - 1, 0);
    } else {
      int f = g_testFan;
      sscanf(cmd.c_str(), "fan %d", &f);
      g_testFan = constrain(f, 0, 3);
    }
    const char* fanStr[] = {"自动","低","中","高"};
    termPrint("[*] 风速 → %s，正在发射...", fanStr[g_testFan]);
    if (!irSend()) termPrint("[E] 发射失败");

  } else if (cmd.startsWith("swing")) {
    // swing on  → keycode 10（摆风开）
    // swing off → keycode 11（固定风向，停止摆动）
    // swing     → 切换
    g_testIsAc = true;
    if (cmd == "swing on")        g_testSwing = 0;
    else if (cmd == "swing off")  g_testSwing = 1;
    else                          g_testSwing = g_testSwing ? 0 : 1;
    g_acFuncCode = (g_testSwing == 0) ? 10 : 11; // 10=SWING, 11=FIX
    termPrint("[*] 扫风 → %s (keycode=%d)，正在发射...",
              g_testSwing==0 ? "摆动开" : "固定(关)", g_acFuncCode);
    if (!irSend()) termPrint("[E] 发射失败");

  } else if (cmd.startsWith("mode")) {
    // mode <0-4>  或  mode cool/heat/auto/fan/dry
    g_testIsAc   = true;
    g_acFuncCode = 1;  // AC_FUNCTION_MODE
    int m = g_testMode;
    if      (cmd.indexOf("cool") >= 0 || cmd.indexOf("冷") >= 0) m = 0;
    else if (cmd.indexOf("heat") >= 0 || cmd.indexOf("热") >= 0) m = 1;
    else if (cmd.indexOf("auto") >= 0 || cmd.indexOf("自动") >= 0) m = 2;
    else if (cmd.indexOf("fan")  >= 0 || cmd.indexOf("送风") >= 0) m = 3;
    else if (cmd.indexOf("dry")  >= 0 || cmd.indexOf("除湿") >= 0) m = 4;
    else sscanf(cmd.c_str(), "mode %d", &m);
    g_testMode = constrain(m, 0, 4);
    const char* modeStr[] = {"制冷","制热","自动","送风","除湿"};
    termPrint("[*] 模式 → %s，正在发射...", modeStr[g_testMode]);
    if (!irSend()) termPrint("[E] 发射失败");

  } else if (cmd.startsWith("temp")) {
    // temp <16-30>  或  temp+ / temp-
    g_testIsAc   = true;
    g_acFuncCode = 0;  // 温度用完整帧
    if (cmd == "temp+")      g_testTemp = min(g_testTemp + 1, 30);
    else if (cmd == "temp-") g_testTemp = max(g_testTemp - 1, 16);
    else { int t=g_testTemp; sscanf(cmd.c_str(), "temp %d", &t); g_testTemp=constrain(t,16,30); }
    termPrint("[*] 温度 → %d℃，正在发射...", g_testTemp);
    if (!irSend()) termPrint("[E] 发射失败");
  } else if (cmd == "off") {
    g_testIsAc   = true;
    g_testPower  = 1;
    g_acFuncCode = 0;  // AC_FUNCTION_POWER
    if (irSend()) termPrint("[*] 关机指令已发射");
    else          termPrint("[E] 发射失败");
    g_testPower = 0;  // 发完自动复位，下次 resend 仍是开机
  } else if (cmd.startsWith("settv")) {
    int kc = 0;
    sscanf(cmd.c_str(), "settv %d", &kc);
    g_testIsAc = false;
    g_testKeycode = kc;
    termPrint("[*] 切换为TV按键模式 keycode=%d", kc);
  } else if (cmd.startsWith("setcat")) {
    int cat=1, sub=1;
    sscanf(cmd.c_str(), "setcat %d %d", &cat, &sub);
    g_testCat = cat; g_testSubcat = sub;
    termPrint("[*] category=%d sub_category=%d", cat, sub);
  } else if (cmd.startsWith("loglevel")) {
    termPrint("[*] loglevel 仅影响串口详细输出（暂未分级）");

  // ─── IR 输出选择 ───
  } else if (cmd == "ir internal" || cmd == "ir int") {
    g_irSendPin = IR_PIN_INTERNAL;
    termPrint("[*] IR 输出切换为: 内部 (GPIO%d)", IR_PIN_INTERNAL);
  } else if (cmd == "ir external" || cmd == "ir ext") {
    g_irSendPin = IR_PIN_EXTERNAL;
    termPrint("[*] IR 输出切换为: 外部 (GPIO%d)", IR_PIN_EXTERNAL);
  } else if (cmd == "ir status") {
    termPrint("[*] IR 输出: %s (GPIO%d)",
      g_irSendPin == IR_PIN_INTERNAL ? "内部" : "外部", g_irSendPin);

  // ─── IR 学习 ───
  } else if (cmd == "learn") {
    if (g_enumState == RUNNING) {
      termPrint("[E] 枚举进行中，无法进入学习模式");
    } else {
      g_learnMode      = true;
      g_learnCaptured  = false;
      g_learnRawBuf.clear();
      g_learnStartMs   = millis();
      IrReceiver.start();
      termPrint("[LEARN] 学习模式已开启 (30s超时)，对准外置接收器按遥控器...");
      termPrint("[LEARN] 捕获后输入 'learn save <名称>' 保存, 'learn cancel' 取消");
    }
  } else if (cmd == "learn cancel") {
    g_learnMode      = false;
    g_learnCaptured  = false;
    g_learnRawBuf.clear();
    termPrint("[LEARN] 学习已取消");
  } else if (cmd.startsWith("learn save ")) {
    String name = cmd.substring(11);
    name.trim();
    if (name.length() == 0) {
      termPrint("[E] 用法: learn save <名称>");
    } else {
      learnSave(name);
      g_learnMode = false;  // 保存后退出学习模式
    }
  } else if (cmd.startsWith("learn play ")) {
    int id = cmd.substring(11).toInt();
    bool found = false;
    for (auto& e : g_learnedList) {
      if (e.id == id) {
        if (sendLearnedCode(e.file)) {
          termPrint("[LEARN] 已发射 #%d \"%s\"", id, e.name.c_str());
        }
        found = true;
        break;
      }
    }
    if (!found) termPrint("[E] 未找到学习码 #%d", id);
  } else if (cmd == "learn list") {
    if (g_learnedList.empty()) {
      termPrint("[LEARN] 没有已学习的红外码");
    } else {
      termPrint("[LEARN] 已学习红外码 (%d):", (int)g_learnedList.size());
      for (auto& e : g_learnedList) {
        termPrint("  #%d: \"%s\" (freq=%dHz)", e.id, e.name.c_str(), e.frequency);
      }
    }
  } else if (cmd.startsWith("learn delete ")) {
    int id = cmd.substring(13).toInt();
    learnDelete(id);
  } else if (cmd == "learn stop") {
    g_learnMode      = false;
    g_learnCaptured  = false;
    g_learnRawBuf.clear();
    termPrint("[LEARN] 学习模式已关闭");

  } else {
    termPrint("[?] 未知命令，输入 help 查看列表");
  }
}

// ════════════════════════════════════════════════════════
// 模式B：TCP 接收 bin + 指令（兼容 upload_bins.py）
// ════════════════════════════════════════════════════════
static WiFiClient g_tcpClient;

void handleTcpLine(const String& line) {
  if (line.startsWith("a_hello")) {
    g_tcpClient.println("e_hello");

  } else if (line.startsWith("a_bin,")) {
    // a_bin,<cat>,<subcat>,<b64len>,<b64data>
    char proto[8], remote[32], b64[16384];
    int  cat=1, subcat=1, b64len=0;
    // parse: a_bin,cat,subcat,len,data
    String s = line.substring(6);
    int c1=s.indexOf(','), c2=s.indexOf(',',c1+1), c3=s.indexOf(',',c2+1);
    if (c1<0||c2<0||c3<0) { g_tcpClient.println("e_error"); return; }
    cat    = s.substring(0,c1).toInt();
    subcat = s.substring(c1+1,c2).toInt();
    b64len = s.substring(c2+1,c3).toInt();
    String b64data = s.substring(c3+1);
    if (b64len != (int)b64data.length()) { g_tcpClient.println("e_error"); return; }

    int decLen = base64_dec_len((char*)b64data.c_str(), b64len);
    uint8_t* buf = (uint8_t*)malloc(decLen);
    if (!buf) { g_tcpClient.println("e_error"); return; }
    base64_decode((char*)buf, (char*)b64data.c_str(), b64len);

    bool ok = irLoadBin(buf, decLen, cat, subcat);
    free(buf);
    if (ok) {
      termPrint("[TCP] bin 加载成功 cat=%d sub=%d size=%d", cat, subcat, decLen);
      g_tcpClient.println("e_bin");
    } else {
      g_tcpClient.println("e_error");
    }

  } else if (line.startsWith("a_control,")) {
    // a_control,<b64len>,<b64data>
    String s = line.substring(10);
    int c1 = s.indexOf(',');
    if (c1<0) { g_tcpClient.println("e_failed"); return; }
    int b64len = s.substring(0,c1).toInt();
    String b64data = s.substring(c1+1);

    int jsonLen = base64_dec_len((char*)b64data.c_str(), b64len);
    char* jsonStr = (char*)malloc(jsonLen+1);
    if (!jsonStr) { g_tcpClient.println("e_failed"); return; }
    base64_decode(jsonStr, (char*)b64data.c_str(), b64len);
    jsonStr[jsonLen] = 0;

    // 解析 JSON 更新测试参数
    JsonDocument doc;
    if (deserializeJson(doc, jsonStr) == DeserializationError::Ok) {
      int kc = doc["keyCode"] | 0;
      if (doc.containsKey("acStatus")) {
        g_testIsAc  = true;
        g_testTemp  = (doc["acStatus"]["acTemp"] | 10) + 16;
        g_testMode  = doc["acStatus"]["acMode"]  | 0;
        g_testFan   = doc["acStatus"]["acWindSpeed"] | 0;
        g_testKeycode = 0;
      } else {
        g_testIsAc    = false;
        g_testKeycode = kc;
      }
    }
    free(jsonStr);

    bool ok = irSend();
    termPrint("[TCP] 控制指令 → %s", ok ? "✅ 成功" : "❌ 失败");
    g_tcpClient.println(ok ? "e_success" : "e_failed");

  } else if (line.startsWith("a_error")) {
    g_tcpClient.stop();
  }
}

// ════════════════════════════════════════════════════════
// Web 终端 HTML
// ════════════════════════════════════════════════════════
static const char WEB_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>IRext Terminal</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{background:#0d0d0d;color:#00ff41;font-family:'Courier New',monospace;height:100vh;display:flex;flex-direction:column}
  #header{padding:6px 12px;border-bottom:1px solid #00ff4133;display:flex;align-items:center;gap:12px;flex-shrink:0}
  #header h1{font-size:14px;letter-spacing:2px;color:#00ff41}
  #dot{width:8px;height:8px;border-radius:50%;background:#ff3333;flex-shrink:0}
  #dot.on{background:#00ff41;box-shadow:0 0 6px #00ff41}
  #term{flex:1;overflow-y:auto;padding:10px 14px;line-height:1.55;font-size:13px;white-space:pre-wrap;word-break:break-all}
  #inputrow{display:flex;border-top:1px solid #00ff4133;flex-shrink:0}
  #prompt{padding:8px 10px;color:#00ff41;flex-shrink:0;font-size:13px;align-self:center}
  #inp{flex:1;background:transparent;border:none;color:#00ff41;font-family:inherit;font-size:13px;padding:8px 4px;outline:none;caret-color:#00ff41}
  #sendbtn{padding:8px 16px;background:#003300;border:none;color:#00ff41;cursor:pointer;font-family:inherit;font-size:13px;border-left:1px solid #00ff4133}
  #sendbtn:active{background:#005500}
  .sys{color:#888}
  .hit{color:#ffff00;font-weight:bold}
  .err{color:#ff5555}
  /* 快捷按钮 */
  #shortcuts{display:flex;gap:6px;padding:6px 12px;border-bottom:1px solid #00ff4133;flex-wrap:wrap;flex-shrink:0}
  .sc{padding:4px 10px;background:#001a00;border:1px solid #00ff4155;color:#00ff41;cursor:pointer;font-size:11px;border-radius:2px}
  .sc:active{background:#003300}
  /* 参数控制面板 */
  #ctrlpanel{padding:6px 12px;border-bottom:1px solid #00ff4133;flex-shrink:0;display:flex;flex-direction:column;gap:5px}
  .ctrlrow{display:flex;align-items:center;gap:5px;flex-wrap:wrap}
  .clabel{font-size:11px;color:#669966;width:36px;flex-shrink:0}
  .cb{padding:3px 9px;background:#001a00;border:1px solid #00ff4133;color:#00cc33;cursor:pointer;font-size:11px;border-radius:2px;white-space:nowrap}
  .cb:active,.cb.active{background:#00ff41;color:#000;border-color:#00ff41}
  .cb.danger{border-color:#ff555555;color:#ff8888}
  .cb.danger:active{background:#ff3333;color:#fff}
  /* 学习面板 */
  #learnpanel{padding:6px 12px;border-bottom:1px solid #00ff4133;flex-shrink:0;display:none;flex-direction:column;gap:5px}
  .lb{padding:3px 9px;background:#001a00;border:1px solid #ffaa0055;color:#ffaa00;cursor:pointer;font-size:11px;border-radius:2px;white-space:nowrap}
  .lb:active{background:#ffaa00;color:#000}
  .linput{background:#001a00;border:1px solid #ffaa0055;color:#ffaa00;font-family:inherit;font-size:11px;padding:3px 6px;width:90px;outline:none;caret-color:#ffaa00}
</style>
</head>
<body>
<div id="header">
  <div id="dot"></div>
  <h1>▶ IRext AtomS3 Terminal</h1>
</div>
<div id="shortcuts">
  <button class="sc" onclick="send('start')">▶ start</button>
  <button class="sc" onclick="send('next')">⏭ next</button>
  <button class="sc" onclick="send('hit')">🎯 hit</button>
  <button class="sc" onclick="send('resend')">↺ resend</button>
  <button class="sc" onclick="send('skip')">⏩ skip</button>
  <button class="sc" onclick="send('status')">ℹ status</button>
  <button class="sc" onclick="send('list')">📋 list</button>
  <button class="sc" onclick="send('help')">? help</button>
</div>
<div id="ctrlpanel">
  <div class="ctrlrow" style="margin-bottom:2px;border-bottom:1px solid #00ff4122;padding-bottom:3px">
    <span class="clabel">IR输出</span>
    <button class="cb active" id="btnIrInt" onclick="setIrOutput('int')">内部</button>
    <button class="cb" id="btnIrExt" onclick="setIrOutput('ext')">外部</button>
  </div>
  <div class="ctrlrow">
    <span class="clabel">电源</span>
    <button class="cb" onclick="send('resend')">开机</button>
    <button class="cb danger" onclick="send('off')">关机</button>
  </div>
  <div class="ctrlrow">
    <span class="clabel">温度</span>
    <button class="cb" onclick="send('temp-')">－</button>
    <button class="cb" id="btnTemp" onclick="">26℃</button>
    <button class="cb" onclick="send('temp+')">＋</button>
  </div>
  <div class="ctrlrow">
    <span class="clabel">模式</span>
    <button class="cb" onclick="setMode(0)">制冷</button>
    <button class="cb" onclick="setMode(1)">制热</button>
    <button class="cb" onclick="setMode(2)">自动</button>
    <button class="cb" onclick="setMode(3)">送风</button>
    <button class="cb" onclick="setMode(4)">除湿</button>
  </div>
  <div class="ctrlrow">
    <span class="clabel">风速</span>
    <button class="cb" onclick="setFan(0)">自动</button>
    <button class="cb" onclick="setFan(1)">低</button>
    <button class="cb" onclick="setFan(2)">中</button>
    <button class="cb" onclick="setFan(3)">高</button>
    <button class="cb" style="margin-left:8px" onclick="send('fan-')">－</button>
    <button class="cb" onclick="send('fan+')">＋</button>
  </div>
  <div class="ctrlrow">
    <span class="clabel">扫风</span>
    <button class="cb" id="btnSwingOn"  onclick="send('swing on')">摆动开</button>
    <button class="cb" id="btnSwingOff" onclick="send('swing off')">固定</button>
  </div>
</div>
<div id="learnpanel">
  <div class="ctrlrow">
    <span class="clabel" style="color:#ffaa00;width:36px">学习</span>
    <button class="lb" id="btnLearnStart" onclick="send('learn')">开始</button>
    <button class="lb" onclick="send('learn stop')">停止</button>
    <button class="lb" onclick="send('learn cancel')">取消</button>
    <input class="linput" id="learnName" type="text" placeholder="名称">
    <button class="lb" onclick="saveLearn()">保存</button>
    <button class="lb" onclick="send('learn list')">列表</button>
  </div>
  <div class="ctrlrow">
    <span class="clabel" style="color:#ffaa00;width:36px"></span>
    <span style="font-size:10px;color:#887744">ID:</span>
    <input class="linput" id="learnPlayId" type="text" placeholder="编号" style="width:50px">
    <button class="lb" onclick="playLearn()">发射</button>
    <button class="lb" onclick="deleteLearn()">删除</button>
  </div>
</div>
<div id="term"></div>
<div id="inputrow">
  <span id="prompt">irext $</span>
  <input id="inp" type="text" autocomplete="off" autocorrect="off" autocapitalize="off" spellcheck="false" placeholder="输入命令...">
  <button id="sendbtn" onclick="sendInput()">发送</button>
</div>
<script>
const term = document.getElementById('term');
const inp  = document.getElementById('inp');
const dot  = document.getElementById('dot');
let ws, hist=[], histIdx=-1;

function append(text, cls) {
  const lines = text.split('\n');
  lines.forEach(line => {
    if (!line.length) return;
    const d = document.createElement('div');
    if (cls) d.className = cls;
    else if (line.includes('🎯')||line.includes('✓')) d.className='hit';
    else if (line.includes('[E]')||line.includes('❌')) d.className='err';
    else if (line.startsWith('>')) d.style.color='#88ff88';
    d.textContent = line;
    term.appendChild(d);
  });
  term.scrollTop = term.scrollHeight;
}

function send(cmd) {
  if (ws && ws.readyState===1) ws.send(cmd);
  else append('[离线] 未连接', 'err');
}

// 面板辅助
let curTemp=26, curMode=0, curFan=0, curSwing=0;
const modeNames=['制冷','制热','自动','送风','除湿'];
const fanNames=['自动','低','中','高'];

function setMode(m){ send('mode '+m); }
function setFan(f){  send('fan '+f);  }

// IR 输出切换
function setIrOutput(mode) {
  if (mode === 'int') {
    send('ir internal');
    document.getElementById('btnIrInt').classList.add('active');
    document.getElementById('btnIrExt').classList.remove('active');
  } else {
    send('ir external');
    document.getElementById('btnIrExt').classList.add('active');
    document.getElementById('btnIrInt').classList.remove('active');
  }
}

// 学习面板
function saveLearn() {
  const name = document.getElementById('learnName').value.trim();
  if (!name) { append('[提示] 请先输入名称', 'sys'); return; }
  send('learn save ' + name);
}

function playLearn() {
  const id = document.getElementById('learnPlayId').value.trim();
  if (!id) { append('[提示] 请输入ID编号', 'sys'); return; }
  send('learn play ' + id);
}

function deleteLearn() {
  const id = document.getElementById('learnPlayId').value.trim();
  if (!id) { append('[提示] 请输入ID编号', 'sys'); return; }
  send('learn delete ' + id);
}

function syncPanel(text){
  // 从日志消息中同步面板状态
  let m;
  if((m=text.match(/温度\s*→\s*(\d+)℃/))){
    curTemp=parseInt(m[1]);
    document.getElementById('btnTemp').textContent=curTemp+'℃';
  }
  if((m=text.match(/风速\s*→\s*(自动|低|中|高)/))){
    const fi=fanNames.indexOf(m[1]);
    document.querySelectorAll('#ctrlpanel .ctrlrow:nth-child(5) .cb').forEach((b,i)=>{
      b.classList.toggle('active', i===fi);
    });
  }
  if((m=text.match(/模式\s*→\s*(制冷|制热|自动|送风|除湿)/))){
    const mi=modeNames.indexOf(m[1]);
    document.querySelectorAll('#ctrlpanel .ctrlrow:nth-child(4) .cb').forEach((b,i)=>{
      b.classList.toggle('active', i===mi);
    });
  }
  if(text.includes('摆动开')){
    document.getElementById('btnSwingOn').classList.add('active');
    document.getElementById('btnSwingOff').classList.remove('active');
  } else if(text.includes('固定(关)')){
    document.getElementById('btnSwingOff').classList.add('active');
    document.getElementById('btnSwingOn').classList.remove('active');
  }
  // IR 输出状态
  if(text.includes('IR 输出切换为: 内部')){
    document.getElementById('btnIrInt').classList.add('active');
    document.getElementById('btnIrExt').classList.remove('active');
  } else if(text.includes('IR 输出切换为: 外部')){
    document.getElementById('btnIrExt').classList.add('active');
    document.getElementById('btnIrInt').classList.remove('active');
  }
  // 学习模式状态
  if(text.includes('[LEARN] 学习模式已开启')){
    document.getElementById('learnpanel').style.display = 'flex';
    document.getElementById('learnName').value = '';
    document.getElementById('learnName').placeholder = '等待捕获...';
    document.getElementById('learnPlayId').value = '';
  } else if(text.includes('[LEARN] 30秒无信号') || text.includes('学习模式已关闭') || text.includes('已保存:')){
    document.getElementById('learnpanel').style.display = 'none';
  } else if(text.includes('[LEARN] 捕获')){
    document.getElementById('learnName').placeholder = '输入名称后保存';
    document.getElementById('learnName').style.borderColor = '#ffaa00';
  } else if(text.includes('[LEARN] 已保存')){
    document.getElementById('learnName').style.borderColor = '#ffaa0055';
    document.getElementById('learnName').value = '';
  }
}

function sendInput() {
  const v = inp.value.trim();
  if (!v) return;
  hist.unshift(v); histIdx=-1;
  inp.value='';
  send(v);
}

inp.addEventListener('keydown', e => {
  if (e.key==='Enter') { sendInput(); return; }
  if (e.key==='ArrowUp')   { histIdx=Math.min(histIdx+1,hist.length-1); inp.value=hist[histIdx]||''; }
  if (e.key==='ArrowDown') { histIdx=Math.max(histIdx-1,-1); inp.value=histIdx<0?'':hist[histIdx]; }
});

function connect() {
  ws = new WebSocket('ws://' + location.host + '/ws');
  ws.onopen = () => {
    dot.className='on';
    append('─── 已连接 IRext AtomS3 ───', 'sys');
    ws.send('status');
  };
  ws.onmessage = e => { append(e.data); syncPanel(e.data); };
  ws.onclose = () => {
    dot.className='';
    append('─── 连接断开，3s 后重连 ───', 'sys');
    setTimeout(connect, 3000);
  };
  ws.onerror = () => ws.close();
}
connect();
</script>
</body>
</html>
)rawhtml";

// ════════════════════════════════════════════════════════
// 按键处理
// ════════════════════════════════════════════════════════
static unsigned long g_btnDown   = 0;
static bool          g_btnActive = false;

void handleButton() {
  bool pressed = (digitalRead(BTN_PIN) == LOW);
  if (pressed && !g_btnActive) {
    g_btnActive = true;
    g_btnDown   = millis();
  } else if (!pressed && g_btnActive) {
    g_btnActive = false;
    unsigned long held = millis() - g_btnDown;
    if (held > 600) {
      // 长按 → 标记命中 + 下一条
      handleCommand("hit");
    } else {
      // 短按 → 下一条（如未开始则 start）
      if (g_enumState == IDLE) handleCommand("start");
      else handleCommand("next");
    }
  }
}

// ════════════════════════════════════════════════════════
// WebSocket 事件
// ════════════════════════════════════════════════════════
void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    // 发送历史记录给新连接的客户端
    for (const String& line : g_termLog) {
      client->text(line + "\n");
    }
  } else if (type == WS_EVT_DATA) {
    String cmd = "";
    for (size_t i = 0; i < len; i++) cmd += (char)data[i];
    handleCommand(cmd);
  }
}

// ════════════════════════════════════════════════════════
// setup
// ════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(400);

  pinMode(BTN_PIN, INPUT_PULLUP);
  IrSender.begin(g_irSendPin);
  IrReceiver.begin(IR_RECV_PIN, DISABLE_LED_FEEDBACK);

  // LittleFS
  if (!LittleFS.begin(true)) {
    Serial.println("[E] LittleFS 挂载失败");
  }

  // 加载已学红外码清单
  learnLoadIndex();

  termPrint("╔══════════════════════════════════════╗");
  termPrint("║   IRext AtomS3 Lite — 独立枚举器     ║");
  termPrint("╚══════════════════════════════════════╝");
  termPrint("[*] IR 输出: 内部(GPIO%d)  |  接收: GPIO%d  |  已学码: %d 条",
    IR_PIN_INTERNAL, IR_RECV_PIN, (int)g_learnedList.size());

  // 扫描 bin 文件，决定工作模式
  enumLoadBinList();
  if (g_binList.empty()) {
    termPrint("[*] 模式B：终端模式（等待电脑推送 bin）");
  } else {
    termPrint("[*] 模式A：独立枚举模式");
    termPrint("[*] 输入 start 或短按按键开始枚举");
  }

  // AP 模式
  IPAddress apIP(AP_IP);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255,255,255,0));
  WiFi.softAP(AP_SSID, AP_PASS);
  termPrint("[*] AP 已启动  SSID: %s  IP: %s", AP_SSID, WiFi.softAPIP().toString().c_str());

  // DNS（Captive Portal 重定向）
  dnsServer.start(DNS_PORT, "*", apIP);

  // WebSocket
  ws.onEvent(onWsEvent);
  webServer.addHandler(&ws);

  // HTTP 路由
  // 主页面
  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest* req){
    req->send_P(200, "text/html", WEB_HTML);
  });
  // Captive portal 重定向（iOS / Android / Windows 检测页）
  auto redirect = [](AsyncWebServerRequest* req){
    req->redirect("http://192.168.4.1/");
  };
  webServer.on("/generate_204",        HTTP_GET, redirect);
  webServer.on("/hotspot-detect.html", HTTP_GET, redirect);
  webServer.on("/ncsi.txt",            HTTP_GET, redirect);
  webServer.on("/connecttest.txt",     HTTP_GET, redirect);
  webServer.onNotFound([](AsyncWebServerRequest* req){ req->redirect("http://192.168.4.1/"); });

  webServer.begin();
  termPrint("[*] Web 终端: http://192.168.4.1");

  // TCP（模式B）
  tcpServer.begin();
  termPrint("[*] TCP 端口: %d（模式B 电脑推送用）", TCP_PORT);

  printHelp();
}

// ════════════════════════════════════════════════════════
// loop
// ════════════════════════════════════════════════════════
static String g_serialBuf = "";

void loop() {
  dnsServer.processNextRequest();
  ws.cleanupClients();

  // 按键
  handleButton();

  // IR 学习接收轮询
  if (g_learnMode) learnPollReceiver();

  // 串口命令
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (g_serialBuf.length()) {
        handleCommand(g_serialBuf);
        g_serialBuf = "";
      }
    } else {
      g_serialBuf += c;
    }
  }

  // 模式B TCP
  if (!g_tcpClient || !g_tcpClient.connected()) {
    g_tcpClient = tcpServer.available();
    if (g_tcpClient) {
      termPrint("[TCP] 客户端连入 %s", g_tcpClient.remoteIP().toString().c_str());
      g_tcpClient.println("e_hello");
    }
  }
  if (g_tcpClient && g_tcpClient.connected() && g_tcpClient.available()) {
    String line = g_tcpClient.readStringUntil('\n');
    line.trim();
    if (line.length()) handleTcpLine(line);
  }
}

// IRext 内部 log 回调
extern "C" void serialPrint(int logType, const char* fmt, ...) {
  if (logType < 2) return;  // 只显示 INFO 以上
  char buf[160];
  va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
  Serial.println(buf);
}
