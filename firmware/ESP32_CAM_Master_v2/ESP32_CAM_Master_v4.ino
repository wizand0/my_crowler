/*
 * ============================================================
 *  ESP32_CAM_Master.ino  — v2.0
 *  Робот-телеинспектор вентиляции — Мастер (ESP32-CAM AI-Thinker)
 *
 *  НОВОЕ в v2.0:
 *    • Автостарт записи видео при включении (не надо жать REC)
 *    • Приём LOG:-строк от Arduino по UART → запись в route_log.csv
 *    • Команда REC_AUTO от Arduino → запуск сессии
 *    • Отображение расстояния и состояния в веб-интерфейсе
 *
 *  Функции:
 *    • Wi-Fi точка доступа "Crawler_Inspector" / "crawler123"
 *    • Веб-интерфейс (порт 80): MJPEG-видео + джойстик + серво
 *    • MJPEG-поток (порт 81): для iframe/img в браузере
 *    • Запись JPEG-кадров на SD-карту (папки /REC_<n>)
 *    • Лог маршрута: /route_log.csv на SD (новое v2)
 *    • Отправка команд и хартбита по UART0 → Arduino UNO
 *
 *  Подключение UART0:
 *    ESP32 GPIO1 (TX) → TXS0108E A1 → B1 → Arduino pin 10 (RX)
 *    ESP32 GPIO3 (RX) → TXS0108E A2 → B2 → Arduino pin 11 (TX)
 *    TXS0108E VCCA = 3.3V,  VCCB = 5V,  OE = 3.3V
 *
 *  ⚠ После загрузки прошивки через MB-плату
 *    ОТКЛЮЧИТЕ USB-кабель, затем подключайте Arduino к UART.
 *    Иначе Arduino будет "слышать" USB-мусор.
 *
 *  Зависимости (установить через Boards Manager):
 *    • "ESP32 by Espressif Systems" ≥ 2.0
 * ============================================================
 */

#include "esp_camera.h"
#include "esp_timer.h"
#include "Arduino.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_http_server.h"
#include <WiFi.h>
#include "FS.h"
#include "SD_MMC.h"

// ============================================================
//  ПИНЫ КАМЕРЫ (AI-THINKER ESP32-CAM)
// ============================================================
#define PWDN_GPIO_NUM    32
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM     0
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27
#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      21
#define Y4_GPIO_NUM      19
#define Y3_GPIO_NUM      18
#define Y2_GPIO_NUM       5
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22

// ============================================================
//  НАСТРОЙКИ
// ============================================================
const char* AP_SSID        = "Crawler_Inspector";
const char* AP_PASS        = "crawler123";
const IPAddress AP_IP(192, 168, 4, 1);
const IPAddress AP_GW(192, 168, 4, 1);
const IPAddress AP_SN(255, 255, 255, 0);

#define UART_BAUD          9600
#define HEARTBEAT_MS        500
#define FRAME_SAVE_MS       200   // 5 fps
#define CMD_REPEAT_MS       120

// MJPEG
#define PART_BOUNDARY      "frame"
static const char* STREAM_CT  =
  "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_SEP =
  "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_HDR =
  "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// ============================================================
//  СОСТОЯНИЕ
// ============================================================
bool       isRecording  = false;
bool       sdOK         = false;
char       motorCmd     = 'S';
int        servoAngle   = 90;
int        frameNum     = 0;
String     sessionDir   = "";
String     logFilePath  = "";

unsigned long lastHB        = 0;
unsigned long lastFrameSave = 0;
unsigned long lastCmdSend   = 0;

// Состояние для отображения в веб-интерфейсе — НОВОЕ
char   crawlerStatus[32] = "READY";   // текущее состояние
char   lastLogLine[80]   = "";        // последняя строка лога

// Буфер входящих данных от Arduino — НОВОЕ
#define UART_RX_BUF 128
char   uartRxBuf[UART_RX_BUF];
uint8_t uartRxLen = 0;

// ============================================================
//  HTTP СЕРВЕРЫ
// ============================================================
httpd_handle_t webSrv    = NULL;
httpd_handle_t streamSrv = NULL;

// ============================================================
//  HTML/JS СТРАНИЦА (встроена во Flash)
// ============================================================
static const char INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html lang="ru">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>Crawler Inspector</title>
<style>
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent}
body{background:#0d1117;color:#c9d1d9;font-family:'Segoe UI',Arial,sans-serif;
     display:flex;flex-direction:column;align-items:center;min-height:100vh;padding:8px;gap:8px}
h1{color:#58a6ff;font-size:1.1em;letter-spacing:3px;padding:6px 0}
#statusBar{font-size:.75em;color:#8b949e;height:16px}
#vidBox{width:100%;max-width:640px;background:#000;border:2px solid #21262d;border-radius:8px;overflow:hidden;position:relative}
#vidBox img{width:100%;display:block}
.overlay{position:absolute;top:6px;right:8px;background:rgba(0,0,0,.6);
          border-radius:4px;padding:2px 6px;font-size:.7em;color:#f85149;
          display:none}
.overlay.show{display:block;animation:blink 1s step-start infinite}
@keyframes blink{50%{opacity:0}}
.row{display:flex;gap:16px;align-items:flex-start;flex-wrap:wrap;justify-content:center;width:100%;max-width:640px}
#joyWrap{display:flex;flex-direction:column;align-items:center;gap:4px}
#joyWrap span{font-size:.75em;color:#8b949e}
#joyCanvas{cursor:pointer;touch-action:none;border-radius:50%;
           background:radial-gradient(circle at 40% 35%,#1c2128,#0d1117);
           border:2px solid #21262d}
.servoPanel{display:flex;flex-direction:column;align-items:center;gap:6px;min-width:80px}
.servoPanel label{font-size:.75em;color:#8b949e;text-align:center}
#servoSlider{
  -webkit-appearance:slider-vertical;
  writing-mode:vertical-lr;direction:rtl;
  height:140px;width:28px;
  accent-color:#58a6ff;cursor:pointer
}
#servoVal{font-size:1.3em;font-weight:700;color:#58a6ff}
.infoBar{display:flex;gap:12px;flex-wrap:wrap;justify-content:center;
         background:#161b22;border-radius:8px;padding:8px 12px;
         width:100%;max-width:640px;font-size:.8em}
.infoItem{display:flex;flex-direction:column;align-items:center;gap:2px}
.iLabel{color:#8b949e}
.iVal{color:#58a6ff;font-weight:700;min-width:30px;text-align:center}
#recBtn{padding:7px 14px;background:#238636;color:#fff;border:none;
        border-radius:6px;cursor:pointer;font-size:.85em;font-weight:600;
        transition:background .2s}
#recBtn.active{background:#da3633;animation:blink 1s step-start infinite}
#recBtn:active{transform:scale(.96)}
#cmdBox{font-size:2em;font-weight:900;color:#f0883e;min-width:2ch;text-align:center}
#logLine{font-size:.7em;color:#3fb950;font-family:monospace;
         background:#0d1117;padding:4px 8px;border-radius:4px;
         width:100%;max-width:640px;min-height:20px;word-break:break-all}
</style>
</head>
<body>
<h1>🔦 CRAWLER INSPECTOR v2</h1>
<div id="statusBar">Подключение к потоку...</div>

<div id="vidBox">
  <img id="stream" alt="No signal">
  <span class="overlay" id="recOverlay">● REC</span>
</div>

<div class="infoBar">
  <div class="infoItem"><span class="iLabel">Команда</span><div id="cmdBox">S</div></div>
  <div class="infoItem"><span class="iLabel">Камера</span><div class="iVal" id="servoDisp">90°</div></div>
  <div class="infoItem"><span class="iLabel">Запись</span><button id="recBtn" onclick="toggleRec()">● REC</button></div>
  <div class="infoItem"><span class="iLabel">Пинг</span><div class="iVal" id="pingVal">--</div></div>
  <div class="infoItem"><span class="iLabel">Статус</span><div class="iVal" id="stateVal">--</div></div>
</div>

<div class="row">
  <div id="joyWrap">
    <span>ДВИЖЕНИЕ</span>
    <canvas id="joyCanvas" width="180" height="180"></canvas>
  </div>
  <div class="servoPanel">
    <label>Камера<br>0–180°</label>
    <div id="servoVal">90°</div>
    <input type="range" id="servoSlider" min="0" max="180" value="90"
           oninput="onServo(this.value)">
  </div>
</div>

<div id="logLine">Лог маршрута: ожидание...</div>

<script>
// ── MJPEG поток ──────────────────────────────────────────────
const streamImg = document.getElementById('stream');
const streamURL = 'http://' + location.hostname + ':81/stream';
streamImg.src   = streamURL;
streamImg.onload  = () => document.getElementById('statusBar').textContent = '🟢 Видео: ОК';
streamImg.onerror = () => {
  document.getElementById('statusBar').textContent = '🔴 Поток недоступен — переподключение...';
  setTimeout(() => { streamImg.src = streamURL + '?t=' + Date.now(); }, 3000);
};

// ── Джойстик ─────────────────────────────────────────────────
const canvas = document.getElementById('joyCanvas');
const ctx    = canvas.getContext('2d');
const CX = 90, CY = 90, OR = 80, IR = 28, DZ = 22;
let jx = CX, jy = CY, touching = false;
let lastSentCmd = 'S', cmdTimer = null;

function drawJoy() {
  ctx.clearRect(0, 0, 180, 180);
  ctx.beginPath(); ctx.arc(CX,CY,OR,0,Math.PI*2);
  ctx.strokeStyle='#30363d'; ctx.lineWidth=3; ctx.stroke();
  ctx.fillStyle='#30363d'; ctx.font='16px sans-serif'; ctx.textAlign='center';
  ctx.fillText('▲',CX,CY-52); ctx.fillText('▼',CX,CY+60);
  ctx.fillText('◀',CX-52,CY+5); ctx.fillText('▶',CX+52,CY+5);
  const g = ctx.createRadialGradient(jx-6,jy-6,3,jx,jy,IR);
  g.addColorStop(0,'#58a6ff'); g.addColorStop(1,'#1158b5');
  ctx.beginPath(); ctx.arc(jx,jy,IR,0,Math.PI*2);
  ctx.fillStyle = g; ctx.fill();
  ctx.strokeStyle='#79c0ff'; ctx.lineWidth=1.5; ctx.stroke();
}

function joyCmd() {
  const dx=jx-CX, dy=jy-CY, d=Math.hypot(dx,dy);
  if (d < DZ) return 'S';
  const a = Math.atan2(dy,dx);
  if (a>=-Math.PI/4 && a<Math.PI/4)  return 'R';
  if (a>=Math.PI/4  && a<3*Math.PI/4) return 'B';
  if (a<-Math.PI/4  && a>=-3*Math.PI/4) return 'F';
  return 'L';
}

function clamp(x,y){
  const dx=x-CX,dy=y-CY,d=Math.hypot(dx,dy);
  if(d>OR) return {x:CX+dx/d*OR, y:CY+dy/d*OR};
  return {x,y};
}

function sendCmd(c){
  if(c!==lastSentCmd){ lastSentCmd=c; document.getElementById('cmdBox').textContent=c; fetch('/cmd?m='+c); }
}

function joyStart(e){
  touching=true; joyMove(e);
  if(cmdTimer) clearInterval(cmdTimer);
  cmdTimer = setInterval(()=>{ if(touching) sendCmd(joyCmd()); }, 120);
}
function joyMove(e){
  if(!touching) return; e.preventDefault();
  const r=canvas.getBoundingClientRect();
  const src = e.touches ? e.touches[0] : e;
  const p = clamp(src.clientX-r.left, src.clientY-r.top);
  jx=p.x; jy=p.y; drawJoy();
}
function joyStop(){
  touching=false; jx=CX; jy=CY; drawJoy(); sendCmd('S');
  if(cmdTimer){clearInterval(cmdTimer);cmdTimer=null;}
}

canvas.addEventListener('mousedown',  joyStart);
canvas.addEventListener('mousemove',  joyMove);
canvas.addEventListener('mouseup',    joyStop);
canvas.addEventListener('mouseleave', joyStop);
canvas.addEventListener('touchstart', joyStart, {passive:false});
canvas.addEventListener('touchmove',  joyMove,  {passive:false});
canvas.addEventListener('touchend',   joyStop);

// ── Сервопривод ───────────────────────────────────────────────
let servoTimer = null;
function onServo(v){
  document.getElementById('servoVal').textContent  = v + '°';
  document.getElementById('servoDisp').textContent = v + '°';
  clearTimeout(servoTimer);
  servoTimer = setTimeout(()=>fetch('/cmd?s='+v), 60);
}

// ── Запись ────────────────────────────────────────────────────
let recording = false;
function toggleRec(){
  recording = !recording;
  const btn = document.getElementById('recBtn');
  const ov  = document.getElementById('recOverlay');
  btn.textContent = recording ? '⏹ СТОП' : '● REC';
  btn.className   = recording ? 'active' : '';
  ov.className    = recording ? 'overlay show' : 'overlay';
  fetch('/rec?on=' + (recording?1:0));
}

// ── Статус и лог маршрута (новое v2) ─────────────────────────
setInterval(()=>{
  fetch('/status').then(r=>r.json()).then(d=>{
    document.getElementById('stateVal').textContent  = d.state || '--';
    if (d.rec) {
      document.getElementById('recOverlay').className = 'overlay show';
      document.getElementById('recBtn').className = 'active';
      recording = true;
    }
    if (d.log && d.log !== '') {
      // Декодировать лог: LOG:CHK,dist_cm,yaw*10,pitch*10,dF,dL,dR
      const p = d.log.split(',');
      if (p.length >= 7) {
        const evt  = p[0];
        const dist = (parseInt(p[1])/100).toFixed(2);
        const yaw  = (parseInt(p[2])/10).toFixed(1);
        const pch  = (parseInt(p[3])/10).toFixed(1);
        document.getElementById('logLine').textContent =
          `${evt} | путь: ${dist}м | поворот: ${yaw}° | наклон: ${pch}° | F:${p[4]} L:${p[5]} R:${p[6]} см`;
      }
    }
  }).catch(()=>{});
}, 1500);

// ── Пинг-монитор ────────────────────────────────────────────
setInterval(()=>{
  const t0 = Date.now();
  fetch('/ping').then(()=>{
    document.getElementById('pingVal').textContent = (Date.now()-t0) + 'мс';
  }).catch(()=>{
    document.getElementById('pingVal').textContent = '---';
    document.getElementById('statusBar').textContent = '🔴 Нет связи';
  });
}, 2500);

// Инициализация
drawJoy();
// Показать REC как активный (автостарт)
setTimeout(()=>{
  fetch('/status').then(r=>r.json()).then(d=>{
    if(d.rec){
      document.getElementById('recBtn').className='active';
      document.getElementById('recBtn').textContent='⏹ СТОП';
      document.getElementById('recOverlay').className='overlay show';
      recording=true;
    }
  }).catch(()=>{});
}, 2000);
</script>
</body>
</html>
)rawliteral";

// ============================================================
//  ИНИЦИАЛИЗАЦИЯ КАМЕРЫ
// ============================================================
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size   = FRAMESIZE_VGA;
    config.jpeg_quality = 12;
    config.fb_count     = 2;
  } else {
    config.frame_size   = FRAMESIZE_QVGA;
    config.jpeg_quality = 16;
    config.fb_count     = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[CAM] Ошибка init: 0x%x\n", err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  s->set_brightness(s, 1);
  s->set_saturation(s, 0);
  s->set_gainceiling(s, GAINCEILING_8X);
  s->set_exposure_ctrl(s, 1);
  s->set_aec2(s, 1);
  s->set_awb_gain(s, 1);
  s->set_whitebal(s, 1);

  // --- ДОБАВЛЕНО ДЛЯ ПЕРЕВОРОТА КАМЕРЫ НА 180° ---
  s->set_vflip(s, 1);   // Переворот по вертикали (вверх ногами)
  s->set_hmirror(s, 1); // Отзеркаливание (чтобы право и лево не перепутались)
  // -----------------------------------------------

  Serial.println("[CAM] OK");
  return true;
}

// ============================================================
//  ИНИЦИАЛИЗАЦИЯ SD-КАРТЫ
// ============================================================
bool initSD() {
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("[SD] Нет карты или ошибка");
    return false;
  }
  if (SD_MMC.cardType() == CARD_NONE) {
    Serial.println("[SD] Карта не определена");
    return false;
  }
  Serial.printf("[SD] OK, размер: %llu MB\n", SD_MMC.cardSize() / (1024 * 1024));
  return true;
}

// Создать папку для новой сессии записи
bool startRecordingSession() {
  if (!sdOK) return false;
  int n = 0;
  while (true) {
    String path = "/REC_" + String(n, DEC);
    if (!SD_MMC.exists(path.c_str())) {
      SD_MMC.mkdir(path.c_str());
      sessionDir = path;
      frameNum   = 0;
      Serial.printf("[SD] Сессия: %s\n", sessionDir.c_str());
      return true;
    }
    n++;
    if (n > 9999) return false;
  }
}

// Сохранить один кадр
void saveFrame(camera_fb_t* fb) {
  if (!sdOK || sessionDir.length() == 0) return;
  char path[48];
  snprintf(path, sizeof(path), "%s/%05d.jpg", sessionDir.c_str(), frameNum++);
  File f = SD_MMC.open(path, FILE_WRITE);
  if (!f) return;
  f.write(fb->buf, fb->len);
  f.close();
}

// Записать строку лога маршрута в CSV — НОВОЕ
void writeRouteLog(const char* logLine) {
  if (!sdOK) return;
  // Путь лога: /route_NNN.csv рядом с папкой видео
  if (logFilePath.length() == 0) {
    // Создать имя файла из номера сессии
    if (sessionDir.length() > 0) {
      logFilePath = sessionDir + "_route.csv";
    } else {
      logFilePath = "/route_log.csv";
    }
    // Записать заголовок
    File f = SD_MMC.open(logFilePath.c_str(), FILE_WRITE);
    if (f) {
      f.println("time_ms,event,dist_cm,yaw_x10,pitch_x10,dF,dL,dR");
      f.close();
    }
  }
  File f = SD_MMC.open(logFilePath.c_str(), FILE_APPEND);
  if (!f) return;
  f.print(millis());
  f.print(",");
  f.println(logLine);
  f.close();

  // Запомнить последнюю строку для веб-интерфейса
  strncpy(lastLogLine, logLine, sizeof(lastLogLine) - 1);
  lastLogLine[sizeof(lastLogLine) - 1] = '\0';
}

// ============================================================
//  ЧТЕНИЕ UART ОТ ARDUINO — НОВОЕ v2
// ============================================================
void readArduinoUart() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (uartRxLen > 0) {
        uartRxBuf[uartRxLen] = '\0';

        if (strncmp(uartRxBuf, "LOG:", 4) == 0) {
          // Строка лога маршрута от Arduino
          writeRouteLog(uartRxBuf + 4);  // без префикса LOG:
        } else if (strncmp(uartRxBuf, "REC_AUTO", 8) == 0) {
          // Arduino просит запустить запись
          if (!isRecording) {
            isRecording = startRecordingSession();
            if (isRecording) {
              logFilePath = "";  // сбросить, чтобы создался новый файл
              Serial.println("[REC] Автостарт по команде Arduino");
            }
          }
        }
        uartRxLen = 0;
      }
    } else if (uartRxLen < UART_RX_BUF - 1) {
      uartRxBuf[uartRxLen++] = c;
    } else {
      // Переполнение буфера — сброс
      uartRxLen = 0;
    }
  }
}

// ============================================================
//  HTTP ОБРАБОТЧИКИ
// ============================================================

// GET /  → HTML страница
static esp_err_t handleRoot(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  httpd_resp_send(req, INDEX_HTML, strlen(INDEX_HTML));
  return ESP_OK;
}

// GET /cmd?m=X  → команда мотора
// GET /cmd?s=X  → угол сервы
static esp_err_t handleCmd(httpd_req_t* req) {
  char buf[32];
  size_t qLen = httpd_req_get_url_query_len(req);
  if (qLen > 0 && qLen < sizeof(buf)) {
    httpd_req_get_url_query_str(req, buf, qLen + 1);
    char val[8];
    if (httpd_query_key_value(buf, "m", val, sizeof(val)) == ESP_OK) {
      char c = val[0];
      if (c=='F'||c=='B'||c=='L'||c=='R'||c=='S') {
        motorCmd = c;
        Serial.write(motorCmd);
        lastCmdSend = millis();
      }
    }
    if (httpd_query_key_value(buf, "s", val, sizeof(val)) == ESP_OK) {
      int ang = atoi(val);
      servoAngle = constrain(ang, 0, 180);
      char cmd[8];
      snprintf(cmd, sizeof(cmd), "P%d\n", servoAngle);
      Serial.print(cmd);
    }
  }
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_send(req, "OK", 2);
  return ESP_OK;
}

// GET /rec?on=1|0  → включить/выключить запись
static esp_err_t handleRec(httpd_req_t* req) {
  char buf[16];
  size_t qLen = httpd_req_get_url_query_len(req);
  if (qLen > 0 && qLen < sizeof(buf)) {
    httpd_req_get_url_query_str(req, buf, qLen + 1);
    char val[4];
    if (httpd_query_key_value(buf, "on", val, sizeof(val)) == ESP_OK) {
      bool on = (val[0] == '1');
      if (on && !isRecording) {
        isRecording = startRecordingSession();
        logFilePath = "";  // сбросить путь лога
      } else if (!on && isRecording) {
        isRecording = false;
        sessionDir  = "";
        frameNum    = 0;
        logFilePath = "";
        Serial.println("[REC] Запись остановлена");
      }
    }
  }
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_send(req, isRecording ? "REC" : "STOP", -1);
  return ESP_OK;
}

// GET /ping  → измерение задержки
static esp_err_t handlePing(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_send(req, "pong", 4);
  return ESP_OK;
}

// GET /status  → JSON с состоянием (НОВОЕ v2)
static esp_err_t handleStatus(httpd_req_t* req) {
  char json[256];
  snprintf(json, sizeof(json),
    "{\"state\":\"%s\",\"rec\":%s,\"log\":\"%s\",\"servo\":%d}",
    crawlerStatus,
    isRecording ? "true" : "false",
    lastLogLine,
    servoAngle
  );
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_send(req, json, strlen(json));
  return ESP_OK;
}

// GET /stream  → MJPEG поток (порт 81)
static esp_err_t handleStream(httpd_req_t* req) {
  camera_fb_t* fb  = NULL;
  esp_err_t    res = ESP_OK;
  char         hdr[64];

  httpd_resp_set_type(req, STREAM_CT);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) { res = ESP_FAIL; break; }

    if (isRecording && millis() - lastFrameSave >= FRAME_SAVE_MS) {
      saveFrame(fb);
      lastFrameSave = millis();
    }

    res = httpd_resp_send_chunk(req, STREAM_SEP, strlen(STREAM_SEP));
    if (res == ESP_OK) {
      snprintf(hdr, sizeof(hdr), STREAM_HDR, fb->len);
      res = httpd_resp_send_chunk(req, hdr, strlen(hdr));
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    }

    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;
    vTaskDelay(pdMS_TO_TICKS(40));
  }
  return res;
}

// ============================================================
//  ЗАПУСК HTTP СЕРВЕРОВ
// ============================================================
void startWebServer() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port      = 80;
  cfg.max_uri_handlers = 10;

  httpd_uri_t rootUri   = {"/",       HTTP_GET, handleRoot,   NULL};
  httpd_uri_t cmdUri    = {"/cmd",    HTTP_GET, handleCmd,    NULL};
  httpd_uri_t recUri    = {"/rec",    HTTP_GET, handleRec,    NULL};
  httpd_uri_t pingUri   = {"/ping",   HTTP_GET, handlePing,   NULL};
  httpd_uri_t statusUri = {"/status", HTTP_GET, handleStatus, NULL};  // НОВОЕ

  if (httpd_start(&webSrv, &cfg) == ESP_OK) {
    httpd_register_uri_handler(webSrv, &rootUri);
    httpd_register_uri_handler(webSrv, &cmdUri);
    httpd_register_uri_handler(webSrv, &recUri);
    httpd_register_uri_handler(webSrv, &pingUri);
    httpd_register_uri_handler(webSrv, &statusUri);
    Serial.println("[WEB] Сервер запущен на порту 80");
  }
}

void startStreamServer() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port      = 81;
  cfg.ctrl_port        = 32769;
  cfg.max_uri_handlers = 2;

  httpd_uri_t streamUri = {"/stream", HTTP_GET, handleStream, NULL};

  if (httpd_start(&streamSrv, &cfg) == ESP_OK) {
    httpd_register_uri_handler(streamSrv, &streamUri);
    Serial.println("[STREAM] Сервер запущен на порту 81");
  }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // UART0 для связи с Arduino UNO
  Serial.begin(UART_BAUD);
  delay(300);
  Serial.println("[SYS] ESP32-CAM Crawler Inspector v2.0");

  // Инициализация камеры
  if (!initCamera()) {
    Serial.println("[SYS] КРИТИЧНО: камера не инициализирована");
    while (true) { delay(200); }
  }

  // SD-карта
  sdOK = initSD();

  // ── АВТОСТАРТ ЗАПИСИ — НОВОЕ v2 ─────────────────────────
  if (sdOK) {
    isRecording = startRecordingSession();
    if (isRecording) {
      Serial.printf("[REC] Автостарт: %s\n", sessionDir.c_str());
      strncpy(crawlerStatus, "REC_AUTO", sizeof(crawlerStatus));
    }
  }

  // Wi-Fi точка доступа
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GW, AP_SN);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("[WIFI] AP: %s  IP: %s\n",
                AP_SSID, WiFi.softAPIP().toString().c_str());

  // HTTP серверы
  startWebServer();
  startStreamServer();

  lastHB = millis();
  Serial.println("[SYS] Готов к работе!");
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  unsigned long now = millis();
  uint8_t clients = WiFi.softAPgetStationNum();

  // ── Хартбит → Arduino только пока есть подключённый клиент ─────
  // Если клиентов нет — Arduino потеряет хартбит и через HB_TIMEOUT
  // (3 сек) перейдёт в режим автопилота. Это и есть нужное поведение.
  // if (clients > 0 && now - lastHB >= HEARTBEAT_MS) {
  //   Serial.write('H');
  //   lastHB = now;
  // }

  if (now - lastHB >= HEARTBEAT_MS) {
    uint8_t clients = WiFi.softAPgetStationNum();
    
    if (clients > 0) {
      Serial.write('H'); // Клиент есть - ручное управление
    } else {
      Serial.write('A'); // Клиентов нет - сигнал для автопилота
      
      // Если клиент отвалился, принудительно стартуем запись, если еще не пишем
      if (!isRecording && sdOK) {
        isRecording = startRecordingSession();
        if (isRecording) {
          logFilePath = ""; // сброс файла для новой сессии
          Serial.println("[REC] Аварийный автостарт записи (клиент потерян)");
        }
      }
    }
    lastHB = now;
  }


  // ── Когда клиент отвалился — убедиться что запись идёт ─────────
  static bool prevClientsZero = false;
  if (clients == 0 && !prevClientsZero) {
    // Клиент только что отключился
    Serial.println("[WIFI] Клиент отключился → автопилот на Arduino");
    if (sdOK && !isRecording) {
      isRecording = startRecordingSession();
      if (isRecording) {
        logFilePath = "";
        Serial.println("[REC] Запись запущена после потери клиента");
        strncpy(crawlerStatus, "AUTO_REC", sizeof(crawlerStatus));
      }
    }
  }
  prevClientsZero = (clients == 0);

  // ── Повтор команды мотора (удержание джойстика) — только при клиенте
  if (clients > 0 && motorCmd != 'S' && now - lastCmdSend >= CMD_REPEAT_MS) {
    Serial.write(motorCmd);
    lastCmdSend = now;
  }
  // Если клиент отвалился — отправить стоп один раз
  if (clients == 0 && motorCmd != 'S') {
    motorCmd = 'S';
    Serial.write('S');
  }

  // ── Чтение UART от Arduino (LOG: строки) — НОВОЕ v2 ────────
  readArduinoUart();

  // ── Периодический вывод статуса ─────────────────────────────
  static unsigned long lastStatus = 0;
  if (now - lastStatus >= 10000) {
    lastStatus = now;
    Serial.printf("[INFO] Клиентов AP: %d | Запись: %s | Серво: %d°\n",
                  WiFi.softAPgetStationNum(),
                  isRecording ? "ВКЛ" : "ВЫКЛ",
                  servoAngle);
    if (sdOK && isRecording) {
      Serial.printf("[INFO] Кадров записано: %d | Лог: %s\n",
                    frameNum, logFilePath.c_str());
    }
  }

  // Небольшая задержка для FreeRTOS
  delay(5);
}
