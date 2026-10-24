/*
  너클 밴더 제어 코드 (새콤달콤)
  - 재활 모드 1/2/3단계 : 굽혔다 폈다 반복
  - 유지 모드           : 일정 각도로 굽힌 채 30초 유지
  - 비상정지            : 버튼 / 힘 초과 / 센서 이상 (+ 과전류, INA219 사용 시) → 즉시 줄 풀기
  - 조작                : 시리얼 모니터에 키 입력 (1, 2, 3, h, +, -, s, x, ?)
*/

#include <Arduino.h>
#include <Servo.h>
#include <HX711.h>

#define SIMULATION         1   // 1 = Wokwi(가상 손 모델), 0 = 실물(HX711 로드셀)
#define USE_CURRENT_SENSOR 0   // 1 = INA219 전류 감시 사용 (현재는 예비 부품이라 0)

#if USE_CURRENT_SENSOR && !SIMULATION
#include <Wire.h>
#include <Adafruit_INA219.h>
Adafruit_INA219 ina219;
#endif

// ================= 핀 =================
const int SERVO_PIN       = 9;
const int HX711_DT_PIN    = 2;
const int HX711_SCK_PIN   = 3;
const int STOP_BUTTON_PIN = 4;
const int SPASM_PIN       = A0;   // 시뮬레이션용: 왼쪽 가변저항 = 갑자기 들어간 힘 (0~1kg)
const int STIFFNESS_PIN   = A1;   // 시뮬레이션용: 오른쪽 가변저항 = 손 뻣뻣함 (0.1~1.0kg/cm)

// ================= 기구 설정 (실측값) =================
const float HORN_RADIUS_CM = 1.5;    // 스풀 반경 (페트병 뚜껑 / 3D 프린팅 스풀 기준)
const float FULL_PULL_CM   = 3.0;    // 손 폈을 때 고리 간격 14cm - 주먹 쥐었을 때 11cm
const int   RELEASE_ANGLE  = 0;      // 줄 다 풀린 각도 = 손 편 상태
const int   MAX_SAFE_ANGLE = 115;    // 3cm 당기는 각도. 이 이상은 절대 안 감음
const unsigned long STEP_MS = 20;    // 1도 움직이는 간격 (20ms = 초당 50도)

// ================= 재활 모드 3단계 =================
const float LEVEL_RATIO[3]    = {0.4, 0.7, 1.0};   // 전체 당김 거리의 40%, 70%, 100%
const float LEVEL_FORCE_KG[3] = {0.5, 0.8, 1.2};   // 단계별 힘 상한 (닿으면 더 안 당김)
const unsigned long AT_FLEX_MS = 1000;             // 굽힌 상태 잠깐 유지
const unsigned long AT_EXT_MS  = 1500;             // 편 상태에서 쉬기

// ================= 유지 모드 =================
int holdAngle = 70;                                // 기본 약 1.8cm
const unsigned long HOLD_DURATION_MS = 30000;      // 30초

// ================= 비상정지 기준 =================
const float FORCE_HARD_LIMIT_KG = 1.5;
const float CURRENT_LIMIT_MA    = 600.0;           // INA219 사용 시에만 적용
const float FORCE_MIN_VALID_KG  = -0.3;            // 이보다 작으면 센서 이상
const float FORCE_MAX_VALID_KG  = 1.9;             // 2kg 로드셀 한계 근처면 센서 이상

// ================= 로드셀 보정 (실물용) =================
const float CAL_FACTOR = 253.0;   // 실물은 무게 아는 물건 걸어서 다시 보정
const float FORCE_SIGN = 1.0;     // 실물에서 당길 때 값이 음수로 나오면 -1.0으로

// ================= 상태 =================
enum Mode  { IDLE, REHAB, HOLD, ESTOP };
enum Phase { FLEXING, AT_FLEX, EXTENDING, AT_EXT };

Servo fingerServo;
HX711 scale;

Mode  mode  = IDLE;
Phase phase = AT_EXT;
int level = 0;                     // 0,1,2 = 1,2,3단계
int currentAngle = RELEASE_ANGLE;
int targetAngle  = RELEASE_ANGLE;
unsigned long phaseStart = 0, lastStep = 0, lastPrint = 0;
int cycles = 0;

// ================= 변환 =================
int cmToAngle(float cm) {
  return (int)(cm / HORN_RADIUS_CM * 180.0 / PI);
}

float angleToCm(int angle) {
  return HORN_RADIUS_CM * angle * PI / 180.0;
}

int levelAngle(int lv) {
  return min(cmToAngle(FULL_PULL_CM * LEVEL_RATIO[lv]), MAX_SAFE_ANGLE);
}

// ================= 센서 =================
float readForceKg() {
#if SIMULATION
  // 가상 손 모델: 힘 = 손 뻣뻣함 × 당긴 길이 + 갑자기 들어간 힘
  float stiffness = 0.1 + analogRead(STIFFNESS_PIN) * 0.9 / 1023.0;   // 0.1~1.0 kg/cm
  float spasm     = analogRead(SPASM_PIN) * 1.0 / 1023.0;             // 0~1.0 kg
  return stiffness * angleToCm(currentAngle) + spasm;
#else
  static float last = 0;
  if (scale.is_ready()) last = FORCE_SIGN * scale.get_units(1);   // 기다리지 않고 준비됐을 때만 읽기
  return last;
#endif
}

float readCurrentmA() {
#if USE_CURRENT_SENSOR && !SIMULATION
  return fabs(ina219.getCurrent_mA());
#else
  return 0;
#endif
}

// ================= 동작 =================
bool moveTowardTarget() {          // 1도씩 천천히 이동, 도착하면 true
  targetAngle = constrain(targetAngle, RELEASE_ANGLE, MAX_SAFE_ANGLE);
  if (currentAngle == targetAngle) return true;
  if (millis() - lastStep >= STEP_MS) {
    lastStep = millis();
    currentAngle += (targetAngle > currentAngle) ? 1 : -1;
    fingerServo.write(currentAngle);
  }
  return currentAngle == targetAngle;
}

void emergencyStop(const __FlashStringHelper* reason) {
  if (mode == ESTOP) return;
  currentAngle = targetAngle = RELEASE_ANGLE;
  fingerServo.write(RELEASE_ANGLE);   // 비상시엔 천천히 말고 즉시 풀기
  mode = ESTOP;
  Serial.print(F("[비상정지] "));
  Serial.println(reason);
  Serial.println(F("원인 해결 후 'x' 입력하면 해제"));
}

void runRehab(float force) {
  switch (phase) {
    case FLEXING:
      targetAngle = levelAngle(level);
      if (force >= LEVEL_FORCE_KG[level]) targetAngle = currentAngle;   // 힘 상한 도달 → 여기까지만
      if (moveTowardTarget()) { phase = AT_FLEX; phaseStart = millis(); }
      break;
    case AT_FLEX:
      if (millis() - phaseStart >= AT_FLEX_MS) phase = EXTENDING;
      break;
    case EXTENDING:
      targetAngle = RELEASE_ANGLE;
      if (moveTowardTarget()) { phase = AT_EXT; phaseStart = millis(); cycles++; }
      break;
    case AT_EXT:
      if (millis() - phaseStart >= AT_EXT_MS) phase = FLEXING;
      break;
  }
}

void runHold(float force) {
  float cap = LEVEL_FORCE_KG[level];
  switch (phase) {
    case FLEXING:
      targetAngle = holdAngle;
      if (force >= cap) targetAngle = currentAngle;
      if (moveTowardTarget()) { phase = AT_FLEX; phaseStart = millis(); }
      break;
    case AT_FLEX:
      // 유지 중 힘이 상한을 넘으면 1도씩 살짝 풀어줌
      if (force > cap && currentAngle > RELEASE_ANGLE && millis() - lastStep >= STEP_MS * 10) {
        lastStep = millis();
        currentAngle--;
        fingerServo.write(currentAngle);
      }
      if (millis() - phaseStart >= HOLD_DURATION_MS) phase = EXTENDING;
      break;
    case EXTENDING:
      targetAngle = RELEASE_ANGLE;
      if (moveTowardTarget()) { mode = IDLE; Serial.println(F("[유지 모드 종료]")); }
      break;
    default:
      phase = FLEXING;
  }
}

// ================= 시리얼 명령 =================
void printHelp() {
  Serial.println(F("---- 명령 ----"));
  Serial.println(F("1/2/3 : 재활 모드 시작 (단계)"));
  Serial.println(F("h     : 유지 모드 (마지막 선택 단계의 힘 상한 사용)"));
  Serial.println(F("+ / - : 유지 각도 5도씩 조절"));
  Serial.println(F("s     : 정지 (천천히 풀기)"));
  Serial.println(F("x     : 비상정지 해제"));
  Serial.println(F("?     : 도움말"));
}

void printLevels() {
  for (int i = 0; i < 3; i++) {
    Serial.print(i + 1);
    Serial.print(F("단계: "));
    Serial.print(levelAngle(i));
    Serial.print(F("도 / "));
    Serial.print(angleToCm(levelAngle(i)), 1);
    Serial.print(F("cm / 힘 상한 "));
    Serial.print(LEVEL_FORCE_KG[i], 1);
    Serial.println(F("kg"));
  }
}

void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == '\n' || c == '\r' || c == ' ') return;

  if (mode == ESTOP && c != 'x') {
    Serial.println(F("비상정지 상태입니다. 'x'로 먼저 해제하세요."));
    return;
  }

  switch (c) {
    case '1': case '2': case '3':
      level = c - '1';
      mode = REHAB; phase = FLEXING; cycles = 0;
      Serial.print(F("[재활 모드] ")); Serial.print(level + 1);
      Serial.print(F("단계 → ")); Serial.print(levelAngle(level));
      Serial.print(F("도 ("));      Serial.print(angleToCm(levelAngle(level)), 1);
      Serial.println(F("cm)"));
      break;
    case 'h':
      mode = HOLD; phase = FLEXING;
      Serial.print(F("[유지 모드] ")); Serial.print(holdAngle);
      Serial.print(F("도 ("));         Serial.print(angleToCm(holdAngle), 1);
      Serial.print(F("cm), "));        Serial.print(HOLD_DURATION_MS / 1000);
      Serial.println(F("초"));
      break;
    case '+':
      holdAngle = min(holdAngle + 5, MAX_SAFE_ANGLE);
      Serial.print(F("유지 각도: ")); Serial.print(holdAngle);
      Serial.print(F("도 / "));       Serial.print(angleToCm(holdAngle), 1); Serial.println(F("cm"));
      break;
    case '-':
      holdAngle = max(holdAngle - 5, RELEASE_ANGLE + 5);
      Serial.print(F("유지 각도: ")); Serial.print(holdAngle);
      Serial.print(F("도 / "));       Serial.print(angleToCm(holdAngle), 1); Serial.println(F("cm"));
      break;
    case 's':
      mode = IDLE;
      Serial.println(F("[정지] 천천히 푸는 중"));
      break;
    case 'x':
      if (digitalRead(STOP_BUTTON_PIN) == LOW) {
        Serial.println(F("정지 버튼이 아직 눌려 있어요."));
      } else {
        mode = IDLE; phase = AT_EXT;
        Serial.println(F("[해제] 대기 상태"));
      }
      break;
    case '?':
      printHelp();
      printLevels();
      break;
  }
}

void printStatus(float force, float current) {
  if (millis() - lastPrint < 500) return;
  lastPrint = millis();
  const char* names[] = {"대기", "재활", "유지", "비상정지"};
  Serial.print(F("모드: "));          Serial.print(names[mode]);
  if (mode == REHAB || mode == HOLD) { Serial.print(F(" ")); Serial.print(level + 1); Serial.print(F("단계")); }
  Serial.print(F(" | 각도: "));       Serial.print(currentAngle);
  Serial.print(F(" | 줄당김(cm): ")); Serial.print(angleToCm(currentAngle), 2);
  Serial.print(F(" | 힘(kg): "));     Serial.print(force, 2);
#if USE_CURRENT_SENSOR && !SIMULATION
  Serial.print(F(" | 전류(mA): "));   Serial.print(current, 0);
#endif
  if (mode == REHAB) { Serial.print(F(" | 반복: ")); Serial.print(cycles); }
  Serial.println();
}

// ================= 메인 =================
void setup() {
  Serial.begin(115200);
  fingerServo.attach(SERVO_PIN);
  fingerServo.write(RELEASE_ANGLE);
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);

  scale.begin(HX711_DT_PIN, HX711_SCK_PIN);
  scale.set_scale(CAL_FACTOR);
  scale.tare();   // 실물: 시작할 때 줄이 느슨한 상태여야 영점이 맞음

#if USE_CURRENT_SENSOR && !SIMULATION
  if (!ina219.begin()) Serial.println(F("[INA219] 연결 실패"));
#endif

  Serial.println(F("=== 너클 밴더 제어 시작 ==="));
  printHelp();
  printLevels();
}

void loop() {
  handleSerial();

  float force   = readForceKg();
  float current = readCurrentmA();

  // 안전 체크는 매 루프 최우선
  if (digitalRead(STOP_BUTTON_PIN) == LOW)                          emergencyStop(F("정지 버튼"));
  else if (force < FORCE_MIN_VALID_KG || force > FORCE_MAX_VALID_KG) emergencyStop(F("센서 이상값"));
  else if (force > FORCE_HARD_LIMIT_KG)                             emergencyStop(F("힘 한계 초과"));
#if USE_CURRENT_SENSOR && !SIMULATION
  else if (current > CURRENT_LIMIT_MA)                              emergencyStop(F("과전류"));
#endif

  switch (mode) {
    case IDLE:  targetAngle = RELEASE_ANGLE; moveTowardTarget(); break;
    case REHAB: runRehab(force); break;
    case HOLD:  runHold(force);  break;
    case ESTOP: break;
  }

  printStatus(force, current);
}
