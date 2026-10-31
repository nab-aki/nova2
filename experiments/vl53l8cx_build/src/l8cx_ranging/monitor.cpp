// ToF のモニタリング（見るだけ。スプリント3.6-2 の段1。段2・段3 を前提にした設計は decisions.md・common_tof_checklist.md）
// 方式：esp_http_server。GET / でページを返し、/ws の WebSocket で MonitorPacket（バイナリ）を送る
// タスク：Wi-Fi の開始と送信の合図は monitorTask（コア0）。実際の送信と接続の管理は HTTP サーバーのタスク（コア0）だけが行う
// 受け渡し：loop は最新の1枚を短い排他区間で置くだけ。送信が遅れたら古いものは捨てる（たまらない）
#include "monitor.h"

#include <string.h>
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// wifi_secrets.h は .gitignore 済み。wifi_secrets.example.h をコピーして作る。なければ Wi-Fi なしで動く（測距の試験はできる）
#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#define MONITOR_HAS_WIFI 1
#else
#warning "wifi_secrets.h がないので Wi-Fi なしでビルドする（wifi_secrets.example.h をコピーして作る）"
#define MONITOR_HAS_WIFI 0
#endif

#if MONITOR_HAS_WIFI
#include <WiFi.h>
#include <esp_http_server.h>
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "monitor_page.h"
#endif

// ---- 調整値 ----
#define MONITOR_BASELINE_MS     20000     // 起動してから Wi-Fi を始めるまで（この間の周期を、比べる基準として記録する）
#define MONITOR_SEND_MS         200       // 送る間隔（5Hz。測距の 10Hz は変わらない）
#define MONITOR_WIFI_PS         WIFI_PS_MIN_MODEM   // 省電力：WIFI_PS_NONE（なし）／WIFI_PS_MIN_MODEM（既定）／WIFI_PS_MAX_MODEM
#define MONITOR_CLIENT_SLOTS    2         // 接続の枠（段3 の上限まで）
#define MONITOR_MAX_CLIENTS     1         // いまの上限（段1＝1、段3＝2）。超えたら閉じるコード 4001 で断る
#define MONITOR_CLOSE_FULL      4001      // ページがこのコードを見て「満員」と出す
#define MONITOR_CLIENT_TIMEOUT_MS 5000    // ブラウザから何も来なければ閉じる（スリープしたスマホが枠を占めないように）
#define MONITOR_RECONNECT_MS    10000     // Wi-Fi がつながらないとき、やり直す間隔
#define MONITOR_TASK_TICK_MS    20        // monitorTask の周期
#define MONITOR_TASK_STACK      4096
#define MONITOR_TASK_PRIORITY   2
#define MONITOR_HTTPD_STACK     6144      // ハンドシェイク（SHA1・Base64）があるので既定の 4096 より大きくする
#define MONITOR_HTTPD_SOCKETS   7
#define MONITOR_SEND_TIMEOUT_S  1         // 遅い端末を待つ上限（既定は5秒）
#define MONITOR_CORE            0
#define MONITOR_PS_UNKNOWN      0xFF

#define MONITOR_ZONES           64

// ---- ブラウザへ送る1回分（296バイト。リトルエンディアン。変えたら version と monitor_page.h を一緒に直す）----
struct __attribute__((packed)) PhaseValues {
  uint32_t frames;
  uint32_t periodAvgUs, periodMaxUs;   // フレームの周期
  uint32_t dataAvgUs, dataMaxUs;       // get_ranging_data にかかった時間
};

struct __attribute__((packed)) MonitorPacket {
  uint8_t version;
  uint8_t n;                  // 8 か 4（0＝まだフレームなし）
  uint8_t clients, maxClients;
  uint8_t wifiPs;             // 実際の省電力の設定（esp_wifi_get_ps）
  uint8_t resetReason;        // esp_reset_reason
  int8_t rssi;
  uint8_t flags;              // bit0：測距中、bit1：IP を固定
  uint32_t seq;               // 送った回数
  uint32_t uptimeMs;
  uint32_t frameAgeMs;        // 最後のフレームからの時間
  uint32_t frames, readyFail, dataFail, busErr, stall;
  uint32_t otherLog;          // loop 以外のタスクから出たログの行
  uint32_t sendSkip;          // 送信が間に合わず飛ばした回数
  uint32_t wifiDrops;         // Wi-Fi が切れた回数
  uint32_t heapNow, heapMin;  // 空きメモリ：今と、起動してからの最小
  PhaseValues base;           // Wi-Fi を始める前
  PhaseValues run;            // Wi-Fi を始めたあと
  uint16_t batteryMv;
  uint16_t reserved;
  int16_t dist[MONITOR_ZONES];
  uint8_t status[MONITOR_ZONES];
};
static_assert(sizeof(MonitorPacket) == 296, "monitor_page.h の読み方と合わせる");
#define MONITOR_PACKET_VERSION 1

// ---- loop が書く値 ----
struct PhaseStats {
  uint32_t frames;
  uint32_t periodN, periodMaxUs;
  uint64_t periodSumUs;
  uint32_t dataMaxUs;
  uint64_t dataSumUs;
};

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static int16_t latestDist[MONITOR_ZONES];
static uint8_t latestStatus[MONITOR_ZONES];
static uint8_t latestN = 0;
static uint32_t latestFrameMs = 0;
static uint32_t lastFrameUs = 0;
static bool hasLastFrame = false;
static PhaseStats baseStats;
static PhaseStats runStats;
static volatile bool wifiStarted = false;   // false の間のフレームは基準（baseStats）に数える

// 累計（書くのは loop のタスクだけ。otherLog だけはどのタスクからも増える）
static volatile uint32_t countFrames = 0, countReadyFail = 0, countDataFail = 0, countBusErr = 0, countStall = 0;
static uint32_t countOtherLog = 0;
static volatile uint16_t statusBatteryMv = 0;
static volatile bool statusTofOk = false;

static uint32_t SafeDiv(uint64_t sum, uint32_t n) {
  return n ? (uint32_t)(sum / n) : 0;
}

void MonitorOnFrame(const int16_t *dist, const uint8_t *status, uint8_t n, uint32_t dataUs) {
  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();
  const size_t zones = (size_t)n * n;
  countFrames++;
  portENTER_CRITICAL(&mux);
  memcpy(latestDist, dist, zones * sizeof(int16_t));
  memcpy(latestStatus, status, zones);
  latestN = n;
  latestFrameMs = nowMs;
  PhaseStats &p = wifiStarted ? runStats : baseStats;
  p.frames++;
  p.dataSumUs += dataUs;
  if (dataUs > p.dataMaxUs) p.dataMaxUs = dataUs;
  if (hasLastFrame) {
    const uint32_t period = nowUs - lastFrameUs;
    p.periodN++;
    p.periodSumUs += period;
    if (period > p.periodMaxUs) p.periodMaxUs = period;
  }
  lastFrameUs = nowUs;
  hasLastFrame = true;
  portEXIT_CRITICAL(&mux);
}

void MonitorCountReadyFail(void) { countReadyFail++; }
void MonitorCountDataFail(void) { countDataFail++; }
void MonitorCountBusError(void) { countBusErr++; }
void MonitorCountStall(void) { countStall++; }
void MonitorCountOtherLog(void) { __atomic_add_fetch(&countOtherLog, 1, __ATOMIC_RELAXED); }

void MonitorSetStatus(uint16_t batteryMv, bool tofOk) {
  statusBatteryMv = batteryMv;
  statusTofOk = tofOk;
}

#if !MONITOR_HAS_WIFI

void MonitorSetup(void) {
  Serial.println("[mon] wifi_secrets.h がない：Wi-Fi を使わない");
}

void MonitorPrintStatus(void) {
  Serial.println("[mon] wifi_secrets.h がない：Wi-Fi を使わない");
}

#else  // MONITOR_HAS_WIFI

// wifi_secrets.h の固定 IP は、4つ全部を書くか、全部書かないか
#if defined(WIFI_STATIC_IP) || defined(WIFI_STATIC_GATEWAY) || defined(WIFI_STATIC_SUBNET) || defined(WIFI_STATIC_DNS)
#if !(defined(WIFI_STATIC_IP) && defined(WIFI_STATIC_GATEWAY) && defined(WIFI_STATIC_SUBNET) && defined(WIFI_STATIC_DNS))
#error "IP を固定するときは WIFI_STATIC_IP・GATEWAY・SUBNET・DNS の4つを全部書く"
#endif
#define MONITOR_USE_STATIC_IP 1
#else
#define MONITOR_USE_STATIC_IP 0
#endif

static_assert(MONITOR_MAX_CLIENTS <= MONITOR_CLIENT_SLOTS, "枠より多くは受けられない");

// ---- 接続の枠（書くのは HTTP サーバーのタスクだけ）----
struct ClientSlot {
  int fd;                // -1＝空き
  uint32_t lastRxMs;
};
static ClientSlot slots[MONITOR_CLIENT_SLOTS];
static volatile uint8_t clientCount = 0;

static httpd_handle_t server = nullptr;
static MonitorPacket txPacket;              // 送信中は sendBusy が立っていて、monitorTask は書かない
static volatile bool sendBusy = false;
static volatile uint32_t sendSkip = 0;
static volatile uint32_t wifiDrops = 0;
static volatile uint8_t wifiPsActual = MONITOR_PS_UNKNOWN;
static volatile bool staticIpActive = false;
static uint32_t txSeq = 0;
static uint32_t setupMs = 0;

static bool RemoveSlot(int fd) {
  for (int i = 0; i < MONITOR_CLIENT_SLOTS; i++) {
    if (slots[i].fd == fd) {
      slots[i].fd = -1;
      clientCount--;
      return true;
    }
  }
  return false;
}

// サーバーがセッションを消すときに呼ばれる。close_fn を設定したら、ソケットはここで閉じる（httpd_sess_delete は閉じない）
static void OnSocketClose(httpd_handle_t, int fd) {
  if (RemoveSlot(fd)) {
    Serial.printf("[mon] 切れた fd=%d 接続=%u\n", fd, (unsigned)clientCount);
  }
  close(fd);
}

static void SendCloseFrame(httpd_req_t *req, uint16_t code) {
  uint8_t payload[2] = {(uint8_t)(code >> 8), (uint8_t)(code & 0xFF)};
  httpd_ws_frame_t frame = {};
  frame.final = true;
  frame.type = HTTPD_WS_TYPE_CLOSE;
  frame.payload = payload;
  frame.len = sizeof(payload);
  httpd_ws_send_frame_async(req->handle, httpd_req_to_sockfd(req), &frame);
}

// /ws：最初の GET はハンドシェイクが済んだあとに呼ばれる（esp_http_server の httpd_uri）。以降は受信したフレームごと
static esp_err_t WsHandler(httpd_req_t *req) {
  const int fd = httpd_req_to_sockfd(req);
  if (req->method == HTTP_GET) {
    int freeSlot = -1;
    for (int i = 0; i < MONITOR_CLIENT_SLOTS && freeSlot < 0; i++) {
      if (slots[i].fd < 0) freeSlot = i;
    }
    if (clientCount >= MONITOR_MAX_CLIENTS || freeSlot < 0) {
      SendCloseFrame(req, MONITOR_CLOSE_FULL);
      Serial.printf("[mon] 断った（上限 %d 台）\n", MONITOR_MAX_CLIENTS);
      return ESP_FAIL;   // 失敗を返すと、サーバーがこのソケットを閉じる
    }
    slots[freeSlot].fd = fd;
    slots[freeSlot].lastRxMs = millis();
    clientCount++;
    Serial.printf("[mon] つながった fd=%d 接続=%u\n", fd, (unsigned)clientCount);
    return ESP_OK;
  }
  // ブラウザからは生存の知らせだけが来る。中身は読み捨てる
  httpd_ws_frame_t frame = {};
  uint8_t buf[16];
  frame.payload = nullptr;
  esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);   // 長さだけ
  if (err != ESP_OK || frame.len > sizeof(buf)) {
    return ESP_FAIL;
  }
  if (frame.len > 0) {
    frame.payload = buf;
    err = httpd_ws_recv_frame(req, &frame, frame.len);
    if (err != ESP_OK) {
      return ESP_FAIL;
    }
  }
  for (int i = 0; i < MONITOR_CLIENT_SLOTS; i++) {
    if (slots[i].fd == fd) {
      slots[i].lastRxMs = millis();
    }
  }
  return ESP_OK;
}

// ページを返したらソケットを閉じる。残しておくと、ブラウザの持ち続ける接続でソケットの枠が埋まる
static esp_err_t PageHandler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  esp_err_t err = httpd_resp_send(req, MONITOR_PAGE, HTTPD_RESP_USE_STRLEN);
  httpd_sess_trigger_close(req->handle, httpd_req_to_sockfd(req));
  return err;
}

static esp_err_t FaviconHandler(httpd_req_t *req) {
  httpd_resp_set_status(req, "204 No Content");
  esp_err_t err = httpd_resp_send(req, nullptr, 0);
  httpd_sess_trigger_close(req->handle, httpd_req_to_sockfd(req));
  return err;
}

// HTTP サーバーのタスクで動く：全部の接続に送り、応答のない接続を閉じる
static void SendWork(void *) {
  const uint32_t now = millis();
  txPacket.clients = clientCount;
  httpd_ws_frame_t frame = {};
  frame.final = true;
  frame.type = HTTPD_WS_TYPE_BINARY;
  frame.payload = (uint8_t *)&txPacket;
  frame.len = sizeof(txPacket);
  for (int i = 0; i < MONITOR_CLIENT_SLOTS; i++) {
    const int fd = slots[i].fd;
    if (fd < 0) {
      continue;
    }
    const bool timedOut = (now - slots[i].lastRxMs) >= MONITOR_CLIENT_TIMEOUT_MS;
    if (timedOut || httpd_ws_send_frame_async(server, fd, &frame) != ESP_OK) {
      Serial.printf("[mon] 閉じる fd=%d（%s）\n", fd, timedOut ? "応答なし" : "送信の失敗");
      RemoveSlot(fd);
      httpd_sess_trigger_close(server, fd);
    }
  }
  sendBusy = false;
}

static void BuildPacket(uint32_t nowMs) {
  MonitorPacket &p = txPacket;
  p.version = MONITOR_PACKET_VERSION;
  p.maxClients = MONITOR_MAX_CLIENTS;
  p.wifiPs = wifiPsActual;
  p.resetReason = (uint8_t)esp_reset_reason();
  p.rssi = (int8_t)WiFi.RSSI();
  p.flags = (statusTofOk ? 0x01 : 0) | (staticIpActive ? 0x02 : 0);
  p.seq = ++txSeq;
  p.uptimeMs = nowMs;
  p.frames = countFrames;
  p.readyFail = countReadyFail;
  p.dataFail = countDataFail;
  p.busErr = countBusErr;
  p.stall = countStall;
  p.otherLog = __atomic_load_n(&countOtherLog, __ATOMIC_RELAXED);
  p.sendSkip = sendSkip;
  p.wifiDrops = wifiDrops;
  p.heapNow = esp_get_free_heap_size();
  p.heapMin = esp_get_minimum_free_heap_size();
  p.batteryMv = statusBatteryMv;
  p.reserved = 0;

  PhaseStats base, run;
  uint32_t frameMs;
  portENTER_CRITICAL(&mux);
  memcpy(p.dist, latestDist, sizeof(p.dist));
  memcpy(p.status, latestStatus, sizeof(p.status));
  p.n = latestN;
  frameMs = latestFrameMs;
  base = baseStats;
  run = runStats;
  portEXIT_CRITICAL(&mux);

  p.frameAgeMs = nowMs - frameMs;
  p.base = {base.frames, SafeDiv(base.periodSumUs, base.periodN), base.periodMaxUs, SafeDiv(base.dataSumUs, base.frames), base.dataMaxUs};
  p.run = {run.frames, SafeDiv(run.periodSumUs, run.periodN), run.periodMaxUs, SafeDiv(run.dataSumUs, run.frames), run.dataMaxUs};
}

static void StartServer(void) {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.core_id = MONITOR_CORE;
  cfg.stack_size = MONITOR_HTTPD_STACK;
  cfg.max_open_sockets = MONITOR_HTTPD_SOCKETS;
  cfg.send_wait_timeout = MONITOR_SEND_TIMEOUT_S;
  cfg.close_fn = OnSocketClose;
  esp_err_t err = httpd_start(&server, &cfg);
  if (err != ESP_OK) {
    server = nullptr;
    Serial.printf("[mon] HTTP サーバーの開始に失敗 err=%d\n", (int)err);
    return;
  }
  httpd_uri_t page = {};
  page.uri = "/";
  page.method = HTTP_GET;
  page.handler = PageHandler;
  httpd_register_uri_handler(server, &page);
  httpd_uri_t favicon = {};
  favicon.uri = "/favicon.ico";
  favicon.method = HTTP_GET;
  favicon.handler = FaviconHandler;
  httpd_register_uri_handler(server, &favicon);
  httpd_uri_t ws = {};
  ws.uri = "/ws";
  ws.method = HTTP_GET;
  ws.handler = WsHandler;
  ws.is_websocket = true;
  httpd_register_uri_handler(server, &ws);
}

static const char *PsName(uint8_t ps) {
  switch (ps) {
    case WIFI_PS_NONE:      return "なし（WIFI_PS_NONE）";
    case WIFI_PS_MIN_MODEM: return "MIN_MODEM";
    case WIFI_PS_MAX_MODEM: return "MAX_MODEM";
    default:                return "不明";
  }
}

static void StartWifi(void) {
  Serial.printf("[mon] Wi-Fi を開始 SSID=%s\n", WIFI_SSID);
  wifiStarted = true;
  WiFi.persistent(false);   // フラッシュへ書かない（書いている間は両方のコアが止まる）
  WiFi.setSleep((wifi_ps_type_t)MONITOR_WIFI_PS);
  WiFi.setAutoReconnect(true);
#if MONITOR_USE_STATIC_IP
  IPAddress ip, gateway, subnet, dns;
  if (ip.fromString(WIFI_STATIC_IP) && gateway.fromString(WIFI_STATIC_GATEWAY) && subnet.fromString(WIFI_STATIC_SUBNET) &&
      dns.fromString(WIFI_STATIC_DNS) && WiFi.config(ip, gateway, subnet, dns)) {
    staticIpActive = true;
  } else {
    Serial.println("[mon] IP の固定に失敗（書き方を確かめる）：DHCP で続ける");
  }
#endif
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void MonitorPrintStatus(void) {
  const uint32_t now = millis();
  if (!wifiStarted) {
    uint32_t el = now - setupMs;
    Serial.printf("[mon] Wi-Fi は未開始（あと %lums）\n", (unsigned long)(el < MONITOR_BASELINE_MS ? MONITOR_BASELINE_MS - el : 0));
    return;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("[mon] Wi-Fi 接続待ち status=%d 切れた回数=%lu\n", (int)WiFi.status(), (unsigned long)wifiDrops);
    return;
  }
  Serial.printf("[mon] IP=%s（%s）RSSI=%d 省電力=%s 接続=%u/%d 切れた回数=%lu 空きメモリ=%lu 最小=%lu\n", WiFi.localIP().toString().c_str(),
                staticIpActive ? "固定" : "DHCP", (int)WiFi.RSSI(), PsName(wifiPsActual), (unsigned)clientCount, MONITOR_MAX_CLIENTS,
                (unsigned long)wifiDrops, (unsigned long)esp_get_free_heap_size(), (unsigned long)esp_get_minimum_free_heap_size());
}

static void MonitorTask(void *) {
  // 基準を取る間は Wi-Fi を始めない
  while ((millis() - setupMs) < MONITOR_BASELINE_MS) {
    vTaskDelay(pdMS_TO_TICKS(MONITOR_TASK_TICK_MS));
  }
  StartWifi();
  bool connected = false;
  uint32_t lastTryMs = millis();
  uint32_t lastSendMs = millis();
  for (;;) {
    const uint32_t now = millis();
    const bool nowConnected = (WiFi.status() == WL_CONNECTED);
    if (nowConnected && !connected) {
      wifi_ps_type_t ps = WIFI_PS_NONE;
      wifiPsActual = (esp_wifi_get_ps(&ps) == ESP_OK) ? (uint8_t)ps : MONITOR_PS_UNKNOWN;
      if (server == nullptr) {
        StartServer();
      }
      MonitorPrintStatus();
    } else if (!nowConnected && connected) {
      wifiDrops++;
      lastTryMs = now;
      Serial.println("[mon] Wi-Fi が切れた");
    }
    connected = nowConnected;
    if (!connected && (now - lastTryMs) >= MONITOR_RECONNECT_MS) {
      lastTryMs = now;
      Serial.printf("[mon] Wi-Fi をやり直す status=%d\n", (int)WiFi.status());
      WiFi.reconnect();
    }
    if (connected && server != nullptr && clientCount > 0 && (now - lastSendMs) >= MONITOR_SEND_MS) {
      lastSendMs = now;
      if (sendBusy) {
        sendSkip++;   // 前の送信が終わっていない。古いものは送らない
      } else {
        BuildPacket(now);
        sendBusy = true;
        if (httpd_queue_work(server, SendWork, nullptr) != ESP_OK) {
          sendBusy = false;
          sendSkip++;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(MONITOR_TASK_TICK_MS));
  }
}

void MonitorSetup(void) {
  setupMs = millis();
  for (int i = 0; i < MONITOR_CLIENT_SLOTS; i++) {
    slots[i].fd = -1;
  }
  Serial.printf("[mon] Wi-Fi は %d ms 後に始める（それまでの周期を基準にする）。送信 %dms、上限 %d 台\n", MONITOR_BASELINE_MS, MONITOR_SEND_MS, MONITOR_MAX_CLIENTS);
  xTaskCreatePinnedToCore(MonitorTask, "monitor", MONITOR_TASK_STACK, nullptr, MONITOR_TASK_PRIORITY, nullptr, MONITOR_CORE);
}

#endif  // MONITOR_HAS_WIFI
