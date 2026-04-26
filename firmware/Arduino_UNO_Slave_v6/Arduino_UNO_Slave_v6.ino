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
// #define PIN_PWMA    5
// #define PIN_AIN1    4
// #define PIN_AIN2    3
// #define PIN_PWMB    6
// #define PIN_BIN1    7
// #define PIN_BIN2    8
#define SW_RX      10
#define SW_TX      11
#define PIN_PWMA    6
#define PIN_AIN1    7
#define PIN_AIN2    8
#define PIN_PWMB    5
#define PIN_BIN1    4
#define PIN_BIN2    3
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
#define SPEED_AUTO          55    // базовая скорость автопилота
#define SPEED_AUTO_MIN      42    // минимальная скорость в тесных местах
#define SPEED_AUTO_MAX      95    // максимальная скорость на свободном участке


#define DIST_FRONT_STOP     15
#define DIST_FRONT_CLEAR    24
#define DIST_SIDE_DEAD      12
#define DIST_SIDE_WARN      18

#define DESCENT_TRIG     -30.0f
#define RECOVERY_TRIG    -10.0f

#define DIST_PERIOD        250
#define IMU_PERIOD          20
#define SCAN_PERIOD        100
#define SCAN_MIN            45
#define SCAN_MAX           135
#define SCAN_STEP            5

#define BUZZ_HZ           2000
#define GYRO_Z_INVERT      -1

#define AUTO_MISSION_TIME 1200000UL   // 20 минут автономного движения
#define MANUAL_FAILSAFE_TIMEOUT 3000UL
#define TILT_PAUSE_MS        8000UL
#define TILT_TRY_MOVE_MS      900UL   // короткая попытка проехать после паузы
#define TILT_TRY_SPEED         65     // скорость короткой попытки
#define TILT_RETRY_LIMIT       20

// --- Фильтр наклона ---
#define TILT_CONFIRM_COUNT     5       // сколько подряд плохих измерений нужно
#define TILT_RELEASE_COUNT    5       // сколько подряд хороших измерений нужно
#define TILT_CONFIRM_MS      200UL     // наклон должен держаться минимум столько

// --- Реверс перед разворотом ---
#define REVERSE_BEFORE_TURN_MS 350UL
#define REVERSE_SPEED           70

// --- Память препятствия / застревания ---
#define TURN_FAIL_LIMIT          5    // сколько неудачных разворотов подряд считаем тупиком
#define OBSTACLE_MEMORY_MS    12000UL // окно памяти одного и того же препятствия

#define MOTOR_L_INVERT        -1
#define MOTOR_R_INVERT        -1
#define MOTOR_L_SCALE       1.00f
#define MOTOR_R_SCALE       0.90f

unsigned long lastCmdTime = 0;
#define CMD_TIMEOUT 400   // мс

// --- Верификация тупика (НОВОЕ) ---
#define DEAD_VERIFY_BACK_MS    500UL   // отъезд назад перед разворотом
#define DEAD_VERIFY_BACK_SPD    70     // скорость отъезда
#define DEAD_ROTATE_YAW        170.0f  // целевой угол разворота (≈180°)
#define DEAD_ROTATE_TIMEOUT_MS 4500UL  // максимум на разворот
#define DEAD_ROTATE_SPEED       70     // скорость разворота
#define WALL_FOLLOW_TIME_MS    6000UL  // время попытки движения вдоль стены
#define WALL_FOLLOW_TARGET_CM    20    // желаемое расстояние до стены
#define DEAD_ATTEMPT_LIMIT       3     // кол-во попыток вырваться перед SOS

// --- Состояние верификации тупика (НОВОЕ) ---
unsigned long deadVerifyStart = 0;
unsigned long deadRotateStart = 0;
unsigned long deadRotateLastMs = 0;
float         deadRotateYawAcc = 0.0f;   // накопленный yaw за время разворота
unsigned long wallFollowStart = 0;
uint8_t       deadAttempts = 0;           // сколько раз уже пытались выбраться
bool          wallFollowLeft = true;      // вдоль какой стены едем

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
  ST_AUTO_REVERSE,
  ST_AUTO_TURN,
  ST_AUTO_DEAD_VERIFY,   // НОВОЕ: отъезд назад перед проверкой
  ST_AUTO_DEAD_ROTATE,   // НОВОЕ: разворот на 180° для проверки тупика
  ST_AUTO_WALL_FOLLOW,   // НОВОЕ: движение вдоль стены
  ST_AUTO_DEAD,          // настоящий тупик (после всех проверок)
  ST_TILT_PAUSE,
  ST_TILT_TRY_MOVE
};
RobotState robotState = ST_MANUAL;

bool autoMode = false;

bool autoMissionActive = false;          // сейчас выполняется автономная миссия
bool autoMissionExpired = false;   // миссия завершилась по таймауту, повторно не запускать
unsigned long autoMissionStart = 0;      // старт миссии
unsigned long tiltPauseStart = 0;        // начало паузы из-за наклона
unsigned long tiltTryMoveStart = 0;      // старт короткой попытки движения
uint8_t tiltRetryCount = 0;              // число повторных попыток после опасного наклона
bool tiltPauseActive = false;            // активна пауза по наклону


// --- Фильтр наклона ---
uint8_t tiltBadCount = 0;
uint8_t tiltGoodCount = 0;
unsigned long tiltFirstBadMs = 0;

// --- Память препятствия ---
uint8_t repeatedObstacleCount = 0;
unsigned long lastObstacleMs = 0;

// --- Реверс перед разворотом ---
unsigned long reverseStartMs = 0;
bool reverseTurnToLeft = true;

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
  EVT_CHECKPOINT,
  EVT_TURN,
  EVT_TILT,
  EVT_TILT_RESUME,
  EVT_OBSTACLE,
  EVT_DEAD_END,
  EVT_SIGNAL_LOST,
  EVT_SIGNAL_BACK,
  EVT_AUTO_START,
  EVT_AUTO_END
};

bool logSignalLostSent = false;  // чтобы не спамить при потере связи

void logEvent(LogEvent evt) {
  // Не спамим лог в ручном режиме — только важные события
  if (robotState == ST_MANUAL && 
      evt != EVT_AUTO_START && evt != EVT_AUTO_END) {
    return;
  }
  const char* evtName;
  switch (evt) {
    case EVT_CHECKPOINT:  evtName = "CHK";    break;
    case EVT_TURN:        evtName = "TURN";   break;
    case EVT_TILT:        evtName = "TILT";   break;
    case EVT_TILT_RESUME: evtName = "TRES";   break;
    case EVT_OBSTACLE:    evtName = "OBS";    break;
    case EVT_DEAD_END:    evtName = "DEAD";   break;
    case EVT_SIGNAL_LOST: evtName = "WLOST";  break;
    case EVT_SIGNAL_BACK: evtName = "WOK";    break;
    case EVT_AUTO_START:  evtName = "ASTART"; break;
    case EVT_AUTO_END:    evtName = "AEND";   break;
    default:              evtName = "UNK";    break;
  }

  char buf[64];
  int dist_cm   = (int)(odo_distance * 100);
  int yaw_x10   = (int)(yaw_accumulated * 10);
  int pitch_x10 = (int)(pitch * 10);

  snprintf(buf, sizeof(buf), "LOG:%s,%d,%d,%d,%ld,%ld,%ld\n",
           evtName, dist_cm, yaw_x10, pitch_x10, dF, dL, dR);

  comSerial.print(buf);

  yaw_accumulated = 0.0f;
  odo_last_logged = odo_distance;
}

// ============================================================
//  ОДОМЕТРИЯ: обновление (вызывать периодически) — НОВОЕ v2
// ============================================================
void updateOdometry() {
  unsigned long now = millis();
  if (now - odo_last_ms < (unsigned long)LOG_PERIOD) return;

  // Если только что был приём команды — подождём следующего цикла
  if (now - lastCmdTime < 50) return;

  float dt = (now - odo_last_ms) / 1000.0f;
  odo_last_ms = now;
  if (dt > 1.0f) { odo_last_ms = now; return; }  // защита от первого вызова

  // Текущая скорость по среднему PWM обоих моторов
  float avgPWM = 0.0f;
  if (robotState == ST_AUTO_FWD ||
      robotState == ST_AUTO_TURN ||
      robotState == ST_TILT_TRY_MOVE ||
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
        bool wasDisconnected = !clientConnected;
        lastHB = millis();
        hbSeen = true;
        clientConnected = true;

        if (wasDisconnected) {
            // Оператор вернулся. Если был в автономной миссии (не явно через 'X'),
            // то прекращаем её и передаём управление оператору.
            // Если autoMode==true (оператор сам включил AUTO) — оставляем миссию.
            if (autoMissionActive && !autoMode) {
                autoMissionActive = false;
                logEvent(EVT_AUTO_END);
            }
            autoMissionExpired = false;
            if (logSignalLostSent) {
                logEvent(EVT_SIGNAL_BACK);
                logSignalLostSent = false;
            }
        }
        continue;
    }

    if (c == 'A') {
        lastHB = millis();
        hbSeen = true;

        if (clientConnected && !logSignalLostSent) {
            logEvent(EVT_SIGNAL_LOST);
            logSignalLostSent = true;
        }
        clientConnected = false;
        // Инициализацию миссии делает loop() — здесь ничего не трогаем
        continue;
    }

    if (c == 'X') {   // включить автопилот вручную из web
        autoMode = true;
        autoMissionExpired = false;
        lastHB = millis();
        // Если сидели в тупике — принудительно перезапускаем движение
        if (robotState == ST_AUTO_DEAD) {
            robotState = ST_AUTO_FWD;
            deadAttempts = 0;
            repeatedObstacleCount = 0;
            lastObstacleMs = 0;
            buzzOff();
        }
        continue;
    }


    if (c == 'M') {   // ручной режим
      autoMode = false;
      autoMissionActive = false;
      autoMissionExpired = false;
      tiltPauseActive = false;
      tiltPauseStart = 0;
      tiltTryMoveStart = 0;
      tiltRetryCount = 0;
      tiltBadCount = 0;
      tiltGoodCount = 0;
      tiltFirstBadMs = 0;
      repeatedObstacleCount = 0;
      lastObstacleMs = 0;
      deadAttempts = 0;
      deadRotateYawAcc = 0.0f;
      logEvent(EVT_AUTO_END);
      continue;
    }

    if (c == 'F' || c == 'B' || c == 'L' || c == 'R' || c == 'S') {
      motorCmd = c;
      lastCmdTime = millis();   // 👈 добавь это
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
  if (millis() - lastCmdTime > CMD_TIMEOUT) {
    motorCmd = 'S';
  }

  switch (motorCmd) {
    case 'F': setMotors(SPEED_FULL, SPEED_FULL); break;
    case 'B': setMotors(-SPEED_FULL, -SPEED_FULL); break;
    case 'L': setMotors(-SPEED_FULL, SPEED_FULL); break;
    case 'R': setMotors(SPEED_FULL, -SPEED_FULL); break;
    default:  stopMotors(); break;
  }
}

// ============================================================
//  ОБРАБОТЧИК: Автопилот — вперёд
// ============================================================
void handleAutoFwd() {
  static bool obstacleLogged = false;

  // --- Препятствие спереди ---
  if (dF < DIST_FRONT_STOP) {
    stopMotors();

    if (!obstacleLogged) {
      logEvent(EVT_OBSTACLE);
      obstacleLogged = true;
    }

    // Память препятствия
    if (millis() - lastObstacleMs <= OBSTACLE_MEMORY_MS) {
      if (repeatedObstacleCount < 255) repeatedObstacleCount++;
    } else {
      repeatedObstacleCount = 1;
    }
    lastObstacleMs = millis();

    // Подозрение на тупик: спереди и с боков тесно.
    // НЕ объявляем тупик сразу — запускаем верификацию
    // (отъезд назад + разворот + повторный замер).
    if (dL < DIST_SIDE_DEAD && dR < DIST_SIDE_DEAD) {
      stopMotors();
      deadVerifyStart = millis();
      robotState = ST_AUTO_DEAD_VERIFY;
      return;
    }

    // Много раз подряд упираемся — подозрение на тупик, запускаем верификацию
    if (repeatedObstacleCount >= TURN_FAIL_LIMIT) {
      stopMotors();
      deadVerifyStart = millis();
      robotState = ST_AUTO_DEAD_VERIFY;
      return;
    }

    reverseTurnToLeft = (dL >= dR);
    reverseStartMs = millis();
    robotState = ST_AUTO_REVERSE;
    return;
  }

  obstacleLogged = false;

  // Сбрасываем память препятствия, если уехали
  if (dF > 35 && millis() - lastObstacleMs > OBSTACLE_MEMORY_MS) {
    repeatedObstacleCount = 0;
  }

  // --- Адаптивная скорость по свободе впереди ---
  int base;
  if (dF > 60) {
    base = SPEED_AUTO_MAX;
  } else if (dF > 40) {
    base = 75;
  } else if (dF > 25) {
    base = SPEED_AUTO;
  } else {
    base = SPEED_AUTO_MIN;
  }

  // По бокам тесно — снижаем базу
  if (dL < DIST_SIDE_WARN || dR < DIST_SIDE_WARN) {
    base = min(base, 60);
  }

  // --- Подруливание ---
  long diff = (long)dR - (long)dL;

  int steerGain = 2;
  if (dL < DIST_SIDE_WARN || dR < DIST_SIDE_WARN) steerGain = 4;

  int steer = constrain((int)(diff * steerGain), -40, 40);

  int lSpd, rSpd;
  if (steer >= 0) {
    // diff > 0 → dR > dL → справа простор → поворот направо
    // => замедляем ПРАВОЕ колесо
    lSpd = base;
    rSpd = constrain(base - steer, SPEED_AUTO_MIN, SPEED_AUTO_MAX);
  } else {
    // diff < 0 → dL > dR → слева простор → поворот налево
    // => замедляем ЛЕВОЕ колесо
    lSpd = constrain(base + steer, SPEED_AUTO_MIN, SPEED_AUTO_MAX);
    rSpd = base;
  }

  // Аварийный отскок от опасно близкой стенки
  if (dL < DIST_SIDE_DEAD) {
    // Слева вплотную — уходим вправо: правое замедляем
    lSpd = base;
    rSpd = SPEED_AUTO_MIN;
  }
  if (dR < DIST_SIDE_DEAD) {
    // Справа вплотную — уходим влево: левое замедляем
    lSpd = SPEED_AUTO_MIN;
    rSpd = base;
  }

  setMotors(lSpd, rSpd);
}

void handleAutoReverse(unsigned long now) {
  // Короткий откат назад перед разворотом
  setMotors(-REVERSE_SPEED, -REVERSE_SPEED);

  if (now - reverseStartMs >= REVERSE_BEFORE_TURN_MS) {
    stopMotors();
    turnToLeft = reverseTurnToLeft;
    robotState = ST_AUTO_TURN;
  }
}

// ============================================================
//  ОБРАБОТЧИК: Автопилот — разворот
// ============================================================
void handleAutoTurn() {
    static unsigned long turnStart = 0;
    if (turnStart == 0) turnStart = millis();
    
    if (dF > DIST_FRONT_CLEAR) {
        turnStart = 0;
        robotState = ST_AUTO_FWD;
        return;
    }

    // Защита от зацикливания: не крутимся дольше 5 сек → верификация
    if (millis() - turnStart > 5000UL) {  
        turnStart = 0;
        stopMotors();
        deadVerifyStart = millis();
        robotState = ST_AUTO_DEAD_VERIFY;
        return;
    }
    
    if (dF < DIST_FRONT_STOP && dL < DIST_SIDE_DEAD && dR < DIST_SIDE_DEAD) {
        turnStart = 0;
        stopMotors();
        deadVerifyStart = millis();
        robotState = ST_AUTO_DEAD_VERIFY;
        return;
    }
    
    if (turnToLeft) setMotors(-SPEED_AUTO_MIN, SPEED_AUTO_MIN);
    else            setMotors(SPEED_AUTO_MIN, -SPEED_AUTO_MIN);
}

// ============================================================
//  ОБРАБОТЧИК: Тупик
// ============================================================

// ============================================================
//  ВЕРИФИКАЦИЯ ТУПИКА: отъезд назад (НОВОЕ)
//  Цель: оторваться от угла, создать пространство для разворота
// ============================================================
void handleAutoDeadVerify(unsigned long now) {
  setMotors(-DEAD_VERIFY_BACK_SPD, -DEAD_VERIFY_BACK_SPD);

  if (now - deadVerifyStart >= DEAD_VERIFY_BACK_MS) {
    stopMotors();
    // Переходим к развороту на ~180°
    deadRotateStart   = now;
    deadRotateYawAcc  = 0.0f;
    deadRotateLastMs  = 0;
    // Разворачиваемся в ту сторону, где больше свободного места
    wallFollowLeft    = (dL >= dR);  // если слева свободнее — крутим налево
    robotState = ST_AUTO_DEAD_ROTATE;
  }
}

// ============================================================
//  ВЕРИФИКАЦИЯ ТУПИКА: разворот на ~180° (НОВОЕ)
//  Цель: физически проверить, действительно ли мы в углу,
//  или просто все три датчика одновременно «увидели» стены
// ============================================================
void handleAutoDeadRotate(unsigned long now) {
  // Поворот на месте в выбранном направлении
  if (wallFollowLeft) {
    setMotors(-DEAD_ROTATE_SPEED, DEAD_ROTATE_SPEED);   // налево
  } else {
    setMotors(DEAD_ROTATE_SPEED, -DEAD_ROTATE_SPEED);   // направо
  }

  // Интегрируем гироскоп для контроля угла поворота
  // (gyroZ_dps обновляется в mpuUpdate)
  if (deadRotateLastMs == 0) deadRotateLastMs = now;
  float dt = (now - deadRotateLastMs) / 1000.0f;
  deadRotateLastMs = now;
  if (dt > 0.1f) dt = 0.02f;
  deadRotateYawAcc += gyroZ_dps * dt;

  bool yawDone = (fabsf(deadRotateYawAcc) >= DEAD_ROTATE_YAW);
  bool timeout = (now - deadRotateStart >= DEAD_ROTATE_TIMEOUT_MS);

  if (yawDone || timeout) {
    stopMotors();
    deadRotateLastMs = 0;    // сброс для следующего раза

    // После разворота проверяем ситуацию свежими замерами датчиков
    // (они обновляются асинхронно, но медиана актуальна)
    bool frontClear = (dF > DIST_FRONT_CLEAR);
    bool sideClear  = (dL > DIST_SIDE_WARN || dR > DIST_SIDE_WARN);

    if (frontClear) {
      // Ура, впереди свободно — продолжаем автопилот, тупик был ложный
      repeatedObstacleCount = 0;
      lastObstacleMs = 0;
      robotState = ST_AUTO_FWD;
      buzzPattern(PAT_BEACON, 1);
      return;
    }

    if (sideClear) {
      // Впереди всё ещё стена, но сбоку есть место — идём вдоль стены
      wallFollowStart = now;
      // Выбираем сторону стены: ту, что ближе
      wallFollowLeft = (dL < dR);
      robotState = ST_AUTO_WALL_FOLLOW;
      return;
    }

    // Всё по-прежнему тесно — учитываем попытку
    if (deadAttempts < 255) deadAttempts++;
    if (deadAttempts >= DEAD_ATTEMPT_LIMIT) {
      // Это действительно тупик — SOS
      robotState = ST_AUTO_DEAD;
      buzzPattern(PAT_SOS);
      logEvent(EVT_DEAD_END);
      deadAttempts = 0;
    } else {
      // Попробуем ещё раз: откат + разворот в противоположную сторону
      deadVerifyStart = now;
      robotState = ST_AUTO_DEAD_VERIFY;
    }
  }
}

// ============================================================
//  ДВИЖЕНИЕ ВДОЛЬ СТЕНЫ (НОВОЕ)
//  Цель: когда впереди препятствие, но сбоку есть проход —
//  попробовать проехать вдоль стены и найти обход
// ============================================================
void handleAutoWallFollow(unsigned long now) {
  // Таймаут попытки
  if (now - wallFollowStart >= WALL_FOLLOW_TIME_MS) {
    stopMotors();
    // Время вышло — повторная верификация тупика
    deadVerifyStart = now;
    robotState = ST_AUTO_DEAD_VERIFY;
    return;
  }

  // Если впереди внезапно освободилось — обратно в обычный автопилот
  if (dF > DIST_FRONT_CLEAR) {
    repeatedObstacleCount = 0;
    deadAttempts = 0;
    robotState = ST_AUTO_FWD;
    buzzPattern(PAT_BEACON, 1);
    return;
  }

  // Если спереди вплотную — отступаем и снова проверяем
  if (dF < DIST_FRONT_STOP) {
    stopMotors();
    deadVerifyStart = now;
    robotState = ST_AUTO_DEAD_VERIFY;
    return;
  }

  // Пропорциональное управление: держим WALL_FOLLOW_TARGET_CM до стены
  long sideDist = wallFollowLeft ? dL : dR;
  long err = sideDist - (long)WALL_FOLLOW_TARGET_CM;

  // Ограничиваем ошибку
  if (err >  30) err =  30;
  if (err < -30) err = -30;

  int base = SPEED_AUTO_MIN + 8;  // медленно и аккуратно
  int steer = (int)err;           // положит. ошибка → отдаляемся от стены → доворачиваем к ней

  int lSpd, rSpd;
  if (wallFollowLeft) {
    // Стена слева: если err>0 (далеко) — уходим левее (замедляем левое колесо)
    lSpd = constrain(base - steer, SPEED_AUTO_MIN, SPEED_AUTO);
    rSpd = constrain(base + steer, SPEED_AUTO_MIN, SPEED_AUTO);
  } else {
    // Стена справа: если err>0 (далеко) — уходим правее (замедляем правое колесо)
    lSpd = constrain(base + steer, SPEED_AUTO_MIN, SPEED_AUTO);
    rSpd = constrain(base - steer, SPEED_AUTO_MIN, SPEED_AUTO);
  }

  setMotors(lSpd, rSpd);
}


void handleAutoDead() {
  stopMotors();
  if (scanAngle != 90) {
    scanAngle = 90;
    camServo.write(90);
  }
}



// ============================================================
//  ЗАЩИТА ОТ СПУСКА (абсолютный приоритет)
// ============================================================
void checkTilt(unsigned long now) {
  // В ручном режиме наклон игнорируем полностью
  if (robotState == ST_MANUAL) {
    tiltBadCount = 0;
    tiltGoodCount = 0;
    tiltFirstBadMs = 0;
    return;
  }

  // --- Подтверждение опасного наклона ---
  if (pitch < DESCENT_TRIG) {
    tiltGoodCount = 0;

    if (tiltBadCount == 0) {
      tiltFirstBadMs = now;
    }

    if (tiltBadCount < 255) tiltBadCount++;

    bool confirmedByCount = (tiltBadCount >= TILT_CONFIRM_COUNT);
    bool confirmedByTime  = (tiltFirstBadMs != 0 && (now - tiltFirstBadMs >= TILT_CONFIRM_MS));

    if ((confirmedByCount || confirmedByTime) && !tiltPauseActive) {
      tiltPauseActive = true;
      tiltPauseStart = now;
      robotState = ST_TILT_PAUSE;
      stopMotors();
      buzzContinuous();
      logEvent(EVT_TILT);
      odo_last_ms = now;
    }
    return;
  }

  // --- Если наклон стал безопаснее ---
  tiltBadCount = 0;
  tiltFirstBadMs = 0;

  if (pitch > RECOVERY_TRIG) {
    if (tiltGoodCount < 255) tiltGoodCount++;
  } else {
    tiltGoodCount = 0;
  }
}

void handleTiltPause(unsigned long now) {
  stopMotors();

  if (!tiltPauseActive) {
    robotState = ST_AUTO_FWD;
    return;
  }

  // Если за время паузы робот уже выровнялся — возвращаем обычный автопилот
  if (tiltGoodCount >= TILT_RELEASE_COUNT) {
    tiltPauseActive = false;
    tiltRetryCount = 0;
    tiltBadCount = 0;
    tiltGoodCount = 0;
    tiltFirstBadMs = 0;
    buzzPattern(PAT_BEACON, 1);
    logEvent(EVT_TILT_RESUME);
    robotState = ST_AUTO_FWD;
    return;
  }

  // Пауза ещё не закончилась
  if (now - tiltPauseStart < TILT_PAUSE_MS) {
    return;
  }

  // Достигли лимита попыток — считаем участок непреодолимым
  if (tiltRetryCount >= TILT_RETRY_LIMIT) {
    tiltPauseActive = false;
    stopMotors();
    deadVerifyStart = millis();
    robotState = ST_AUTO_DEAD_VERIFY;
    return;
  }

  // После паузы делаем короткую попытку движения
  tiltTryMoveStart = now;
  robotState = ST_TILT_TRY_MOVE;
  buzzPattern(PAT_BEACON, 1);
}

void handleTiltTryMove(unsigned long now) {
  // Короткая попытка движения вперёд
  setMotors(TILT_TRY_SPEED, TILT_TRY_SPEED);

  // Если робот уже выровнялся — возвращаемся в обычный автопилот
  if (tiltGoodCount >= TILT_RELEASE_COUNT) {
    tiltPauseActive = false;
    tiltRetryCount = 0;
    tiltBadCount = 0;
    tiltGoodCount = 0;
    tiltFirstBadMs = 0;
    buzzPattern(PAT_BEACON, 1);
    logEvent(EVT_TILT_RESUME);
    robotState = ST_AUTO_FWD;
    return;
  }

  // Закончилась короткая попытка, но наклон ещё не нормализовался
  if (now - tiltTryMoveStart >= TILT_TRY_MOVE_MS) {
    stopMotors();
    if (tiltRetryCount < 255) tiltRetryCount++;
    tiltPauseStart = now;
    tiltPauseActive = true;
    robotState = ST_TILT_PAUSE;
    buzzContinuous();
  }
}

// ============================================================
//  СКАНИРОВАНИЕ СЕРВОЙ В АВТОПИЛОТЕ
// ============================================================
void updateScan(unsigned long now) {
  // Сканируем только когда активно двигаемся
  if (robotState != ST_AUTO_FWD &&
      robotState != ST_AUTO_REVERSE &&
      robotState != ST_AUTO_TURN &&
      robotState != ST_AUTO_WALL_FOLLOW &&
      robotState != ST_TILT_TRY_MOVE) return;

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
    comSerial.print("REC_AUTO\n");
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
  if (robotState != ST_MANUAL) {
    checkTilt(now);
  }

  // ── 5. Одометрия и логирование маршрута — НОВОЕ ────────────
  if (robotState != ST_TILT_PAUSE) {
    updateOdometry();
  }

  // ── 6. Защита от потери оператора на долгое время ───────────


  // ── 7. Основная логика (если нет тревоги спуска) ───────────
  {
      bool hbTimedOut = (now - lastHB > MANUAL_FAILSAFE_TIMEOUT);

      // Если heartbeat вообще пропал (ESP32 отвалилась полностью) —
      // считаем клиента отключённым и запускаем автономную миссию
      if (hbTimedOut && clientConnected) {
          clientConnected = false;
          if (!logSignalLostSent) {
              logEvent(EVT_SIGNAL_LOST);
              logSignalLostSent = true;
          }
      }

      // Автопилот включается если:
      //  1) оператор явно нажал AUTO ('X')
      //  2) клиент отвалился (нет heartbeat или пришло 'A')
      //  3) уже идёт автономная миссия
      bool shouldAuto = (autoMode || !clientConnected || autoMissionActive || hbTimedOut) && !autoMissionExpired; 

      // Ручной режим ТОЛЬКО когда клиент живой и heartbeat свежий
      bool shouldManual = clientConnected && !hbTimedOut && !autoMode && !autoMissionActive && !autoMissionExpired;

      if (shouldAuto) {
          if (!autoMissionActive) {
              autoMissionActive = true;
              autoMissionStart = now;
              tiltRetryCount = 0;
              tiltPauseStart = 0;
              tiltTryMoveStart = 0;
              tiltPauseActive = false;
              tiltBadCount = 0;
              tiltGoodCount = 0;
              tiltFirstBadMs = 0;
              repeatedObstacleCount = 0;
              lastObstacleMs = 0;
              deadAttempts = 0;
              deadRotateYawAcc = 0.0f;  
              logEvent(EVT_AUTO_START);
          }

          // Ограничение миссии: 20 минут
          if (now - autoMissionStart >= AUTO_MISSION_TIME) {
              autoMissionActive = false;
              autoMode = false;
              autoMissionExpired = true;
              robotState = ST_AUTO_DEAD;
              stopMotors();
              buzzPattern(PAT_ABANDONED, 20);
              logEvent(EVT_AUTO_END);
          } else {
              if (robotState == ST_MANUAL) {
                  robotState = ST_AUTO_FWD;
                  buzzPattern(PAT_BEACON, 1);
              } else if (robotState != ST_AUTO_FWD &&
                        robotState != ST_AUTO_REVERSE &&
                        robotState != ST_AUTO_TURN &&
                        robotState != ST_AUTO_DEAD_VERIFY &&
                        robotState != ST_AUTO_DEAD_ROTATE &&
                        robotState != ST_AUTO_WALL_FOLLOW &&
                        robotState != ST_AUTO_DEAD &&
                        robotState != ST_TILT_PAUSE &&
                        robotState != ST_TILT_TRY_MOVE) {
                  robotState = ST_AUTO_FWD;
              }

          }
      } else if (shouldManual) {
          if (robotState != ST_MANUAL) {
              robotState = ST_MANUAL;
              autoMissionActive = false;
              tiltPauseActive = false;
              tiltPauseStart = 0;
              tiltTryMoveStart = 0;
              tiltRetryCount = 0;
              tiltBadCount = 0;
              tiltGoodCount = 0;
              tiltFirstBadMs = 0;
              repeatedObstacleCount = 0;
              lastObstacleMs = 0;
              deadAttempts = 0;              // НОВОЕ
              deadRotateYawAcc = 0.0f;       // НОВОЕ
              stopMotors();
              buzzOff();
              logEvent(EVT_AUTO_END);
          }
      }
      // УБИРАЕМ ветку else со stopMotors() — теперь она не нужна,
      // потому что shouldAuto покрывает случай потери связи

      // ── Выполнение ───────────────────────────────

      switch (robotState) {
          case ST_MANUAL:            handleManual();              break;
          case ST_AUTO_FWD:          handleAutoFwd();             break;
          case ST_AUTO_REVERSE:      handleAutoReverse(now);      break;
          case ST_AUTO_TURN:         handleAutoTurn();            break;
          case ST_AUTO_DEAD_VERIFY:  handleAutoDeadVerify(now);   break;  // НОВОЕ
          case ST_AUTO_DEAD_ROTATE:  handleAutoDeadRotate(now);   break;  // НОВОЕ
          case ST_AUTO_WALL_FOLLOW:  handleAutoWallFollow(now);   break;  // НОВОЕ
          case ST_AUTO_DEAD:         handleAutoDead();            break;
          case ST_TILT_PAUSE:        handleTiltPause(now);        break;
          case ST_TILT_TRY_MOVE:     handleTiltTryMove(now);      break;
          default: break;
      }

      if (robotState != ST_MANUAL) {
          updateScan(now);
      }
  }

  // ── 8. Обновление зуммера ───────────────────────────────────
  buzzUpdate();
}
