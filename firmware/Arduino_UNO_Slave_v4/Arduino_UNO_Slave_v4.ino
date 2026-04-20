/*
 * ============================================================
 *  Arduino_UNO_Slave.ino  — v2.0
 *  Робот-телеинспектор вентиляции — Ведомый (Arduino UNO R3)
 *
 *  НОВОЕ в v2.0:
 *    • Одометрия по времени (SPEED_MPS)
 *    • Накопление рыскания (yaw) через gz гироскопа
 *    • Событийное логирование маршрута → ESP32 → SD-карта
 *      Формат: LOG:EVT,dist_m,yaw_acc,pitch,dF,dL,dR
 *    • Автостарт записи на SD при включении (команда REC_AUTO)
 *
 *  Назначение пинов:
 *  ┌──────────────────────────────────────────────────────────┐
 *  │ SoftwareSerial  RX=10, TX=11  (через TXS0108E)          │
 *  │ TB6612FNG       PWMA=5  AIN1=4  AIN2=3                  │
 *  │                 PWMB=6  BIN1=7  BIN2=8                  │
 *  │                 STBY → 5V (постоянно включён)           │
 *  │ Серводвигатель  PIN=2                                   │
 *  │ Зуммер          PIN=9                                   │
 *  │ HC-SR04 FRONT   TRIG=A0  ECHO=A1                        │
 *  │ HC-SR04 LEFT    TRIG=A2  ECHO=A3                        │
 *  │ HC-SR04 RIGHT   TRIG=12  ECHO=13                        │
 *  │ MPU6050         SDA=A4   SCL=A5  (I2C, адрес 0x68)     │
 *  └──────────────────────────────────────────────────────────┘
 *
 *  Протокол UART (9600 бод):
 *    ESP32 → Arduino:
 *      'F','B','L','R','S'  — команда мотора
 *      'P<угол>\n'          — угол сервопривода (0-180)
 *      'H'                  — хартбит каждые 500 мс
 *    Arduino → ESP32:
 *      'LOG:EVT,...\n'      — строка лога маршрута (новое v2)
 * ============================================================
 */

#include <SoftwareSerial.h>
#include <Servo.h>
#include <Wire.h>

// ============================================================
//  ПИНЫ
// ============================================================
#define SW_RX      10
#define SW_TX      11
#define PIN_PWMA    5
#define PIN_AIN1    4
#define PIN_AIN2    3
#define PIN_PWMB    6
#define PIN_BIN1    7
#define PIN_BIN2    8
#define PIN_SERVO   2
#define PIN_BUZZ    9
#define TRIG_F     A0
#define ECHO_F     A1
#define TRIG_L     A2
#define ECHO_L     A3
#define TRIG_R     12
#define ECHO_R     13
#define MPU_ADDR   0x68

// ============================================================
//  ПАРАМЕТРЫ
// ============================================================
#define BAUD_SW          9600
#define SPEED_FULL       200
#define SPEED_SLOW        76
#define SPEED_AUTO       100   // Уменьшенная базовая скорость автопилота
#define SPEED_AUTO_STEER_HI 120 // Подруливание (быстрая гусеница)
#define SPEED_AUTO_STEER_LO  80 // Подруливание (медленная гусеница)
#define DIST_FRONT_STOP   15
#define DIST_FRONT_CLEAR  20
#define DIST_SIDE_DEAD    12
#define HYSTERESIS         3
#define HB_TIMEOUT      3000
#define DESCENT_TRIG   -30.0f
#define RECOVERY_TRIG  -10.0f
#define DESCENT_HOLD   20000
#define DIST_PERIOD      250
#define IMU_PERIOD        20
#define SCAN_PERIOD      100
#define SCAN_MIN          45
#define SCAN_MAX         135
#define SCAN_STEP          5
#define BUZZ_HZ         2000
#define GYRO_Z_INVERT    -1
#define ABANDON_TIMEOUT 1200000UL   // 20 минут без heartbeat
#define MOTOR_L_INVERT   -1
#define MOTOR_R_INVERT   -1
#define MOTOR_L_SCALE     1.00f
#define MOTOR_R_SCALE     0.90f

// ── Одометрия и логирование ──────────────────────────────
// Скорость при SPEED_FULL: откалибруй один раз!
// Засеки время на 1 метр при PWM=200, раздели 1/время
#define SPEED_MPS       0.25f   // м/с при SPEED_FULL=200 (≈4 сек/метр)
#define CHECKPOINT_DIST  0.50f  // лог каждые 0.5 м
#define MIN_LOG_TURN    20.0f   // лог поворота если накопилось > 20°
#define LOG_PERIOD       500    // мс — период обновления одометрии

// ============================================================
//  КОНЕЧНЫЙ АВТОМАТ
// ============================================================
enum RobotState : uint8_t {
  ST_MANUAL,
  ST_AUTO_FWD,
  ST_AUTO_TURN,
  ST_AUTO_DEAD,
  ST_DESCENT_ALERT,
  ST_DESCENT_CRAWL
};
RobotState robotState = ST_MANUAL;

// ============================================================
//  ОБЪЕКТЫ
// ============================================================
SoftwareSerial comSerial(SW_RX, SW_TX);
Servo camServo;

// ============================================================
//  UART
// ============================================================
char uartBuf[16];
uint8_t uartLen  = 0;
unsigned long lastHB = 0;
bool hbSeen = false;
bool clientConnected = false; // true = клиент подключён по Wi-Fi
bool recAutoSent = false;
bool abandonmentActive = false;
char motorCmd    = 'S';

// ============================================================
//  СЕРВОПРИВОД
// ============================================================
int servoTarget = 90;
int scanAngle   = 90;
int scanDir     =  1;
unsigned long lastScan = 0;

// ============================================================
//  ЗУММЕР
// ============================================================
struct BuzzStep { uint16_t onMs; uint16_t offMs; };

const BuzzStep PAT_BEACON[] PROGMEM = { {100, 2000}, {0, 0} };
const BuzzStep PAT_SOS[]    PROGMEM = {
  {100,100},{100,100},{100,300},
  {300,100},{300,100},{300,300},
  {100,100},{100,100},{100,700},
  {0,0}
};
const BuzzStep PAT_CRAWL[]  PROGMEM = { {200, 800}, {0, 0} };
const BuzzStep PAT_ABANDONED[] PROGMEM = { {500, 20000}, {0, 0} };

enum BuzzMode : uint8_t { BM_OFF, BM_CONTINUOUS, BM_PATTERN };
BuzzMode buzzMode    = BM_OFF;
const BuzzStep* buzzPat = nullptr;
uint8_t  buzzIdx     = 0;
uint16_t buzzLoops   = 0;
uint16_t buzzMaxLoops = 0;
bool     buzzOnPhase = false;
unsigned long buzzTimer = 0;

// ============================================================
//  MPU6050
// ============================================================
float pitch    = 0.0f;
float gyroZ_dps = 0.0f;   // угловая скорость рыскания (°/с) — НОВОЕ
unsigned long lastIMU = 0;

// ============================================================
//  ДИСТАНЦИИ
// ============================================================
long dF = 200, dL = 200, dR = 200;
unsigned long lastDist = 0;

// ============================================================
//  ФЛАГИ СПУСКА
// ============================================================
bool descentAlertActive = false;
bool descentCrawlActive = false;
unsigned long descentStart = 0;

// ============================================================
//  ПОВОРОТ В АВТОПИЛОТЕ
// ============================================================
bool turnToLeft = true;

// ============================================================
//  ОДОМЕТРИЯ — НОВОЕ v2
// ============================================================
float  odo_distance    = 0.0f;  // пройдено, метры
float  odo_last_logged = 0.0f;  // расстояние последней записи в лог
float  yaw_accumulated = 0.0f;  // накопленный поворот с последней записи
unsigned long odo_last_ms = 0;  // время последнего обновления

int leftSpeed  = 0;  // текущие скорости (обновляются в setMotors)
int rightSpeed = 0;

// ============================================================
//  ЛОГИРОВАНИЕ — НОВОЕ v2
// ============================================================
enum LogEvent : uint8_t {
  EVT_CHECKPOINT,   // плановая отметка каждые 0.5 м
  EVT_TURN,         // значимый поворот накопился
  EVT_DESCENT,      // обнаружен спуск
  EVT_DESCENT_END,  // спуск завершён
  EVT_OBSTACLE,     // препятствие спереди
  EVT_DEAD_END,     // тупик
  EVT_SIGNAL_LOST,  // потеря хартбита → автопилот
  EVT_SIGNAL_BACK,  // хартбит восстановился
};

bool logSignalLostSent = false;  // чтобы не спамить при потере связи

void logEvent(LogEvent evt) {
  const char* evtName;
  switch (evt) {
    case EVT_CHECKPOINT:  evtName = "CHK";   break;
    case EVT_TURN:        evtName = "TURN";  break;
    case EVT_DESCENT:     evtName = "DESC";  break;
    case EVT_DESCENT_END: evtName = "DEND";  break;
    case EVT_OBSTACLE:    evtName = "OBS";   break;
    case EVT_DEAD_END:    evtName = "DEAD";  break;
    case EVT_SIGNAL_LOST: evtName = "WLOST"; break;
    case EVT_SIGNAL_BACK: evtName = "WOK";   break;
    default:              evtName = "UNK";   break;
  }

  // Формат: LOG:EVT,dist_m,yaw_acc,pitch,dF,dL,dR
  char buf[64];
  // Умножаем на 100 и делаем целые числа — dtostrf медленный на UNO
  int dist_cm  = (int)(odo_distance * 100);
  int yaw_x10  = (int)(yaw_accumulated * 10);
  int pitch_x10 = (int)(pitch * 10);

  snprintf(buf, sizeof(buf), "LOG:%s,%d,%d,%d,%ld,%ld,%ld\n",
           evtName, dist_cm, yaw_x10, pitch_x10,
           dF, dL, dR);

  // comSerial.print(buf);
  // Serial.print(buf);

  // Сбросить накопленный поворот и отметку расстояния
  yaw_accumulated = 0.0f;
  odo_last_logged = odo_distance;
}

// ============================================================
//  ОДОМЕТРИЯ: обновление (вызывать периодически) — НОВОЕ v2
// ============================================================
void updateOdometry() {
  unsigned long now = millis();
  if (now - odo_last_ms < (unsigned long)LOG_PERIOD) return;

  float dt = (now - odo_last_ms) / 1000.0f;
  odo_last_ms = now;
  if (dt > 1.0f) dt = 0.5f;  // защита от первого вызова

  // Текущая скорость по среднему PWM обоих моторов
  float avgPWM = 0.0f;
  if (robotState == ST_AUTO_FWD ||
      robotState == ST_AUTO_TURN ||
      robotState == ST_MANUAL) {
    int lAbs = abs(leftSpeed);
    int rAbs = abs(rightSpeed);
    avgPWM = (float)(lAbs + rAbs) / 2.0f;
    // Считаем расстояние только при движении вперёд
    if (leftSpeed > 0 && rightSpeed > 0) {
      odo_distance += SPEED_MPS * (avgPWM / (float)SPEED_FULL) * dt;
    }
  }
  // В режиме спуска — скорость неизвестна (трос управляет), не считаем

  // Накопление рыскания из гироскопа
  yaw_accumulated += gyroZ_dps * dt;

  // ── Проверка триггеров логирования ───────────────────────
  if (odo_distance - odo_last_logged >= CHECKPOINT_DIST) {
    logEvent(EVT_CHECKPOINT);
  }
  if (fabsf(yaw_accumulated) >= MIN_LOG_TURN) {
    logEvent(EVT_TURN);
  }
}

// ============================================================
//  ВСПОМОГАТЕЛЬНАЯ: применить один мотор TB6612
// ============================================================
static void applyMotor(int spd,
                       uint8_t pin_in1, uint8_t pin_in2, uint8_t pin_pwm) {
  int s = constrain(abs(spd), 0, 255);
  if (spd > 0) {
    digitalWrite(pin_in1, HIGH); digitalWrite(pin_in2, LOW);
    analogWrite(pin_pwm, s);
  } else if (spd < 0) {
    digitalWrite(pin_in1, LOW); digitalWrite(pin_in2, HIGH);
    analogWrite(pin_pwm, s);
  } else {
    digitalWrite(pin_in1, LOW); digitalWrite(pin_in2, LOW);
    analogWrite(pin_pwm, 0);
  }
}

void setMotors(int left, int right) {
  // Сохраняем "идеальные" скорости для одометрии
  leftSpeed  = left;
  rightSpeed = right;

  // Аппаратная калибровка: направление и баланс
  int final_L = (int)(left  * MOTOR_L_SCALE * MOTOR_L_INVERT);
  int final_R = (int)(right * MOTOR_R_SCALE * MOTOR_R_INVERT);

  applyMotor(final_L, PIN_AIN1, PIN_AIN2, PIN_PWMA);
  applyMotor(final_R, PIN_BIN1, PIN_BIN2, PIN_PWMB);
}
void stopMotors() { setMotors(0, 0); }

// ============================================================
//  MPU6050: инициализация
// ============================================================
void mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0x00);  // гироскоп ±250°/с
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0x00);  // акселерометр ±2g
  Wire.endTransmission(true);

  lastIMU = millis();
}

// ============================================================
//  MPU6050: обновление тангажа + рыскания (комплементарный фильтр)
// ============================================================
void mpuUpdate() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);

  int16_t rax = ((int16_t)Wire.read() << 8) | Wire.read();
  int16_t ray = ((int16_t)Wire.read() << 8) | Wire.read();
  int16_t raz = ((int16_t)Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();            // температура — не нужна
  int16_t rgx = ((int16_t)Wire.read() << 8) | Wire.read();
  int16_t rgy = ((int16_t)Wire.read() << 8) | Wire.read();
  int16_t rgz = ((int16_t)Wire.read() << 8) | Wire.read();

  // Программный поворот осей под физическую установку датчика:
  // X робота вперёд = -Y сенсора, Y робота вправо = -X сенсора
  int16_t ax = -ray;
  int16_t ay = -rax;
  int16_t az = raz;
  int16_t gx = -rgy;
  int16_t gy = -rgx;
  int16_t gz = rgz;

  unsigned long now = millis();
  float dt = (float)(now - lastIMU) * 0.001f;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.02f;
  lastIMU = now;

  // Тангаж (pitch) по акселерометру
  float aPitch = atan2f(-(float)ax,
                  sqrtf((float)ay * (float)ay + (float)az * (float)az))
                 * (180.0f / PI);

  // Угловая скорость тангажа
  float gRate = (float)gx / 131.0f;

  // Комплементарный фильтр тангажа
  pitch = 0.98f * (pitch + gRate * dt) + 0.02f * aPitch;

  // Угловая скорость рыскания (для одометрии)
  gyroZ_dps = (float)(GYRO_Z_INVERT * gz) / 131.0f;
}
// ============================================================
//  HC-SR04: медианный фильтр (3 быстрых замера)
// ============================================================
static void swapL(long &a, long &b) { long t = a; a = b; b = t; }

long medDist(uint8_t trig, uint8_t echo) {
  long s[3];
  for (uint8_t i = 0; i < 3; i++) {
    digitalWrite(trig, LOW);  delayMicroseconds(2);
    digitalWrite(trig, HIGH); delayMicroseconds(10);
    digitalWrite(trig, LOW);
    long dur = pulseIn(echo, HIGH, 12000UL);
    s[i] = (dur == 0) ? 210 : (dur / 58L);
    delayMicroseconds(300);
  }
  if (s[0] > s[1]) swapL(s[0], s[1]);
  if (s[1] > s[2]) swapL(s[1], s[2]);
  if (s[0] > s[1]) swapL(s[0], s[1]);
  return s[1];
}

// ============================================================
//  ЗУММЕР: управление (без delay!)
// ============================================================
void buzzOff() {
  buzzMode = BM_OFF;
  noTone(PIN_BUZZ);
  buzzIdx = 0;
  buzzLoops = 0;
  buzzMaxLoops = 0;
  buzzOnPhase = false;
}

void buzzContinuous() {
  if (buzzMode == BM_CONTINUOUS) return;
  buzzMode = BM_CONTINUOUS;
  buzzLoops = 0;
  buzzMaxLoops = 0;
  tone(PIN_BUZZ, BUZZ_HZ);
}

void buzzPattern(const BuzzStep* pat, uint16_t maxLoops = 0) {
  if (buzzPat == pat && buzzMode == BM_PATTERN && buzzMaxLoops == maxLoops) return;
  noTone(PIN_BUZZ);
  buzzPat      = pat;
  buzzIdx      = 0;
  buzzLoops    = 0;
  buzzMaxLoops = maxLoops;
  buzzMode     = BM_PATTERN;
  buzzOnPhase  = true;
  tone(PIN_BUZZ, BUZZ_HZ);
  buzzTimer    = millis();
}

void buzzUpdate() {
  if (buzzMode != BM_PATTERN || buzzPat == nullptr) return;
  unsigned long now = millis();

  if (buzzPat[buzzIdx].onMs == 0 && buzzPat[buzzIdx].offMs == 0) {
    buzzLoops++;
    if (buzzMaxLoops != 0 && buzzLoops >= buzzMaxLoops) {
      buzzOff();
      return;
    }
    buzzIdx = 0;
    tone(PIN_BUZZ, BUZZ_HZ);
    buzzOnPhase = true;
    buzzTimer   = now;
    return;
  }

  if (buzzOnPhase) {
    if (now - buzzTimer >= buzzPat[buzzIdx].onMs) {
      noTone(PIN_BUZZ);
      buzzOnPhase = false;
      buzzTimer   = now;
    }
  } else {
    if (now - buzzTimer >= buzzPat[buzzIdx].offMs) {
      buzzIdx++;
      if (buzzPat[buzzIdx].onMs == 0 && buzzPat[buzzIdx].offMs == 0) {
        buzzIdx = 0;
      }
      tone(PIN_BUZZ, BUZZ_HZ);
      buzzOnPhase = true;
      buzzTimer   = now;
    }
  }
}

// ============================================================
//  UART: чтение и разбор команд от ESP32-CAM
// ============================================================
void uartRead() {
  while (comSerial.available()) {
  // while (Serial.available()) {
    char c = (char)comSerial.read();
    // char c = (char)Serial.read();

    if (c == 'H') {
      lastHB = millis();
      hbSeen = true;
      clientConnected = true; // Клиент в сети
      continue;
    }
    if (c == 'A') {
      lastHB = millis();
      hbSeen = true;
      clientConnected = false; // Клиент отвалился -> автопилот
      continue;
    }

    if (c == 'F' || c == 'B' || c == 'L' || c == 'R' || c == 'S') {
      motorCmd = c;
      uartLen  = 0;
      continue;
    }

    if (c == 'P') {
      uartLen = 0;
      uartBuf[uartLen++] = c;
      continue;
    }
    if (uartLen > 0 && uartBuf[0] == 'P') {
      if (c == '\n' || c == '\r') {
        uartBuf[uartLen] = '\0';
        int ang = atoi(uartBuf + 1);
        servoTarget = constrain(ang, 0, 180);
        camServo.write(servoTarget);
        uartLen = 0;
      } else if (isdigit(c) && uartLen < 14) {
        uartBuf[uartLen++] = c;
      } else {
        uartLen = 0;
      }
    }
  }
}

// ============================================================
//  ОБРАБОТЧИК: Ручной режим
// ============================================================
void handleManual() {
  switch (motorCmd) {
    case 'F':
      if (dF < DIST_FRONT_STOP) {
        stopMotors();
        logEvent(EVT_OBSTACLE);   // лог препятствия — НОВОЕ
      } else {
        setMotors(SPEED_FULL, SPEED_FULL);
      }
      break;
    case 'B': setMotors(-SPEED_FULL, -SPEED_FULL); break;
    case 'L': setMotors(-SPEED_FULL,  SPEED_FULL); break;
    case 'R': setMotors( SPEED_FULL, -SPEED_FULL); break;
    default:  stopMotors(); break;
  }
}

// ============================================================
//  ОБРАБОТЧИК: Автопилот — вперёд
// ============================================================
void handleAutoFwd() {
  if (dF < DIST_FRONT_STOP) {
    stopMotors();
    logEvent(EVT_OBSTACLE);
    if (dL < DIST_SIDE_DEAD && dR < DIST_SIDE_DEAD) {
      robotState = ST_AUTO_DEAD;
      buzzPattern(PAT_SOS);
      logEvent(EVT_DEAD_END);
    } else {
      turnToLeft = (dL >= dR);
      robotState = ST_AUTO_TURN;
    }
    return;
  }

  long diff = (long)dR - (long)dL;
  int lSpd = SPEED_AUTO, rSpd = SPEED_AUTO;
  if (diff > HYSTERESIS) {
    lSpd = SPEED_AUTO_STEER_HI; rSpd = SPEED_AUTO_STEER_LO;
  } else if (diff < -HYSTERESIS) {
    lSpd = SPEED_AUTO_STEER_LO; rSpd = SPEED_AUTO_STEER_HI;
  }
  setMotors(lSpd, rSpd);
}

// ============================================================
//  ОБРАБОТЧИК: Автопилот — разворот
// ============================================================
void handleAutoTurn() {
  if (dF > DIST_FRONT_CLEAR) {
    robotState = ST_AUTO_FWD;
    return;
  }
  if (turnToLeft) setMotors(-SPEED_AUTO,  SPEED_AUTO);
  else            setMotors( SPEED_AUTO, -SPEED_AUTO);
}

// ============================================================
//  ОБРАБОТЧИК: Тупик
// ============================================================
void handleAutoDead() {
  stopMotors();
}

// ============================================================
//  ЗАЩИТА ОТ СПУСКА (абсолютный приоритет)
// ============================================================
void checkDescent(unsigned long now) {
  if (pitch < DESCENT_TRIG) {
    if (!descentAlertActive && !descentCrawlActive) {
      descentAlertActive = true;
      descentStart       = now;
      robotState         = ST_DESCENT_ALERT;
      stopMotors();
      buzzContinuous();
      logEvent(EVT_DESCENT);    // лог начала спуска — НОВОЕ

    } else if (descentAlertActive && !descentCrawlActive) {
      if (now - descentStart >= DESCENT_HOLD) {
        descentAlertActive = false;
        descentCrawlActive = true;
        robotState         = ST_DESCENT_CRAWL;
        buzzPattern(PAT_CRAWL);
        setMotors(SPEED_SLOW, SPEED_SLOW);
      }
    }

  } else if (pitch > RECOVERY_TRIG) {
    if (descentAlertActive || descentCrawlActive) {
      bool wasConnected  = (now - lastHB <= HB_TIMEOUT);
      descentAlertActive = false;
      descentCrawlActive = false;
      stopMotors();
      buzzOff();
      motorCmd   = 'S';
      logEvent(EVT_DESCENT_END);  // лог конца спуска — НОВОЕ
      robotState = wasConnected ? ST_MANUAL : ST_AUTO_FWD;
      if (!wasConnected) buzzPattern(PAT_BEACON);
    }
  }
}

// ============================================================
//  СКАНИРОВАНИЕ СЕРВОЙ В АВТОПИЛОТЕ
// ============================================================
void updateScan(unsigned long now) {
  if (robotState != ST_AUTO_FWD &&
      robotState != ST_AUTO_TURN &&
      robotState != ST_AUTO_DEAD) return;

  if (now - lastScan < SCAN_PERIOD) return;
  lastScan = now;

  scanAngle += scanDir * SCAN_STEP;
  if (scanAngle >= SCAN_MAX) { scanAngle = SCAN_MAX; scanDir = -1; }
  if (scanAngle <= SCAN_MIN) { scanAngle = SCAN_MIN; scanDir =  1; }
  camServo.write(scanAngle);
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  // Моторы
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT); pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT); pinMode(PIN_PWMB, OUTPUT);
  stopMotors();

  // Зуммер
  pinMode(PIN_BUZZ, OUTPUT);
  noTone(PIN_BUZZ);

  // HC-SR04
  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
  digitalWrite(TRIG_F, LOW);
  digitalWrite(TRIG_L, LOW);
  digitalWrite(TRIG_R, LOW);

  // Сервопривод
  camServo.attach(PIN_SERVO);
  camServo.write(90);

  // I2C + MPU6050
  Wire.begin();
  Wire.setClock(400000);
  mpuInit();

  // UART
  comSerial.begin(BAUD_SW);
  // Serial.begin(BAUD_SW);

  // Инициализировать хартбит
  lastHB = millis();

  // Инициализировать одометрию — НОВОЕ
  odo_last_ms = millis();

  // Самотест: 2 пика зуммером
  tone(PIN_BUZZ, 1500, 80); delay(200);
  tone(PIN_BUZZ, 2500, 80);

  // REC_AUTO отправится после первого heartbeat от ESP32
}

// ============================================================
//  LOOP (без delay!)
// ============================================================
void loop() {
  unsigned long now = millis();

  // ── 1. Приём команд от ESP32-CAM ───────────────────────────
  uartRead();

  // Отправляем REC_AUTO только после первого heartbeat от ESP32
  if (!recAutoSent && hbSeen) {
    // comSerial.print("REC_AUTO\n");
    // Serial.print("REC_AUTO\n");
    recAutoSent = true;
  }

  // ── 2. Обновление тангажа + рыскания (IMU_PERIOD мс) ───────
  if (now - lastIMU >= IMU_PERIOD) {
    mpuUpdate();
  }

  // ── 3. Замер дистанций по одному датчику за раз ────────────
  static uint8_t sonarPhase = 0;
  if (now - lastDist >= (DIST_PERIOD / 3)) {
    lastDist = now;
    switch (sonarPhase) {
      case 0: dF = medDist(TRIG_F, ECHO_F); break;
      case 1: dL = medDist(TRIG_L, ECHO_L); break;
      case 2: dR = medDist(TRIG_R, ECHO_R); break;
    }
    sonarPhase = (sonarPhase + 1) % 3;
  }

  // ── 4. АБСОЛЮТНЫЙ ПРИОРИТЕТ: защита от спуска ──────────────
  checkDescent(now);

  // ── 5. Одометрия и логирование маршрута — НОВОЕ ────────────
  if (robotState != ST_DESCENT_ALERT && robotState != ST_DESCENT_CRAWL) {
    updateOdometry();
  }

  // ── 6. Защита от потери оператора на долгое время ───────────
  if (!abandonmentActive &&
      (now - lastHB >= ABANDON_TIMEOUT) &&
      robotState != ST_DESCENT_ALERT &&
      robotState != ST_DESCENT_CRAWL) {
    abandonmentActive = true;
    robotState = ST_AUTO_DEAD;
    motorCmd = 'S';
    stopMotors();
    buzzPattern(PAT_ABANDONED, 100);
  }

  // ── 7. Основная логика (если нет тревоги спуска) ───────────
  if (robotState != ST_DESCENT_ALERT && robotState != ST_DESCENT_CRAWL) {
    bool espLost = (now - lastHB > HB_TIMEOUT);

    if (espLost) {
      // КРИТИЧЕСКАЯ ПОТЕРЯ ESP32: Стоим на месте! Никакого автопилота!
      if (robotState != ST_MANUAL || motorCmd != 'S') {
        robotState = ST_MANUAL;
        motorCmd = 'S';
        stopMotors();
      }
      buzzPattern(PAT_BEACON); // Просто пищим, прося о помощи

    } else {
      // ESP32 на связи. Смотрим, есть ли клиент по Wi-Fi
      if (clientConnected) {
        // Управление с телефона
        if (robotState != ST_MANUAL) {
          robotState = ST_MANUAL;
          stopMotors();
          buzzOff();
          abandonmentActive = false;
          if (logSignalLostSent) {
            logEvent(EVT_SIGNAL_BACK);
            logSignalLostSent = false;
          }
        }
      } else {
        // Телефон отвалился, включаем самостоятельный АВТОПИЛОТ
        if (robotState == ST_MANUAL && hbSeen) {
          robotState = ST_AUTO_FWD;
          buzzPattern(PAT_BEACON);
          if (!logSignalLostSent) {
            logEvent(EVT_SIGNAL_LOST);
            logSignalLostSent = true;
          }
        }
      }
    }

    // Выполнение команд текущего состояния
    if (!espLost) {
      switch (robotState) {
        case ST_MANUAL:    handleManual();   break;
        case ST_AUTO_FWD:  handleAutoFwd();  break;
        case ST_AUTO_TURN: handleAutoTurn(); break;
        case ST_AUTO_DEAD: handleAutoDead(); break;
        default: break;
      }
    }

    if (clientConnected || espLost) {
      // В ручном режиме угол сервы идет из браузера
    } else {
      updateScan(now); // В автопилоте вертим камерой сами
    }
  }

  // ── 8. Обновление зуммера ───────────────────────────────────
  buzzUpdate();
}
