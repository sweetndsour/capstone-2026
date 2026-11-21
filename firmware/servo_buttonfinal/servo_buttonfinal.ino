// MG996R 서보 시험 코드 (시리얼 명령) - servo_test_2 기반 + 재활/유지 모드 + 버튼
// 시리얼 모니터: 9600 baud
//
// s     : 시작 (START_ANGLE = 0도에서 시작)
// a     : A_ANGLE(기본 30도)로 이동
// d     : D_ANGLE(기본 150도)로 이동
// c     : 90도로 이동
// x     : 정지 (모드 종료, 지금 각도에서 멈춤)
// z     : 신호 끊기 (서보 힘 풀림)
// 숫자  : 그 각도로 이동 (예: 120 입력 후 Enter)
//
// [추가]
// r     : 재활 모드 - 0도 <-> 180도 왕복을 REHAB_REPS회 반복 (x로 중간 정지)
// h     : 유지 모드 - 180도로 가서 3초 유지 후 0도로 복귀
// 버튼  : D4 - GND
//         짧게 1번     : 180도 <-> 0도 (0.2초 기다렸다가 움직임, 재활/유지 모드 중에는 무시)
//         빠르게 2번   : 정지 (0.2초 안에 다시 누르면 그 자리에서 멈춰 버팀)
//                        재활/유지 모드 중에도 두 번째 누르는 순간 바로 정지
//         2초 이상 누름 : 재활 모드 (2초 되면 보드 L 불빛이 켜짐 -> 손 떼기)
//         5초 이상 누름 : 유지 모드 (5초 되면 L 불빛이 깜빡임 -> 손 떼기)
//         ※ s를 안 눌러도 r / h / 버튼은 자동으로 시작됨
//
// CONTINUOUS = false : 180도 서보 (프로젝트용)
// CONTINUOUS = true  : 360도(연속회전) 서보 시험용 (값이 각도가 아니라 속도, 재활/유지/버튼 사용 안 함)

#include <Servo.h>

const bool CONTINUOUS = false;    // 360도 서보면 true

const byte MOTOR_PIN  = 9;
const byte BUTTON_PIN = 4;        // 버튼 대각선 다리: D4 / GND (INPUT_PULLUP, 누르면 LOW)

const int  MIN_ANGLE   = 0;
const int  MAX_ANGLE   = 180;
const int  START_ANGLE = 0;       // s로 시작할 때 각도 (줄 푼 위치)
const int  A_ANGLE     = 30;      // a 키 각도
const int  D_ANGLE     = 150;     // d 키 각도
const int  STOP_VALUE  = 90;      // 360도 서보 정지 값 (조금씩 돌면 88~92로 조정)
const unsigned long STEP_MS = 20; // 1도 움직이는 간격 (작을수록 빠름). 20이면 0->180에 약 3.6초

// 재활 모드
const int  REHAB_MIN = 0;                    // 편 각도
const int  REHAB_MAX = 180;                  // 굽힌 각도
const int  REHAB_REPS = 5;                   // 반복 횟수
const unsigned long REHAB_WAIT_MS = 1000;    // 끝에서 잠깐 멈추는 시간

// 유지 모드
const int  HOLD_ANGLE  = 180;                // 유지 각도
const unsigned long HOLD_MS = 3000;          // 유지 시간 (3초)
const int  HOLD_RETURN = 0;                  // 유지 끝나고 돌아갈 각도

// 버튼
const int  BUTTON_ANGLE = 180;               // 버튼 누르면 갈 각도 (한 번 더 누르면 0도)
const unsigned long DEBOUNCE_MS = 50;        // 버튼 떨림 무시 시간
const unsigned long LONG_MS     = 2000;      // 이만큼 누르면 재활 모드
const unsigned long LONGER_MS   = 5000;      // 이만큼 누르면 유지 모드
const unsigned long DOUBLE_TAP_MS = 200;     // 이 시간 안에 두 번 누르면 정지 (너무 빠르면 300으로)
const byte LED_PIN = 13;                     // 보드에 있는 L 불빛 (누른 시간 알려줌)

Servo motor;

int angle = START_ANGLE;
int target = START_ANGLE;
bool enabled = false;
unsigned long lastStep = 0;

int numBuf = 0;            // 숫자 입력 모으기
bool numActive = false;
unsigned long lastChar = 0;

enum Mode { MANUAL, REHAB, HOLD };
Mode mode = MANUAL;
byte phase = 0;            // 재활: 0 굽히는 중 / 1 위 대기 / 2 펴는 중 / 3 아래 대기
                           // 유지: 0 가는 중 / 1 유지 중 / 2 복귀 중
int rep = 0;
unsigned long phaseStart = 0;

bool lastBtn = HIGH;
unsigned long lastBtnChange = 0;
unsigned long pressStart = 0;
bool pendingTap = false;   // 한 번 눌렀고 두 번째를 기다리는 중
bool secondTap  = false;   // 지금 누른 게 두 번째 탭
unsigned long tapTime = 0; // 첫 번째 탭을 뗀 시각
bool modeStopped = false;  // [추가] 재활/유지 모드를 두 번 탭으로 멈춤

void printHelp() {
  Serial.println(F("s: start at 0"));
  Serial.print(F("a: move to ")); Serial.println(A_ANGLE);
  Serial.print(F("d: move to ")); Serial.println(D_ANGLE);
  Serial.println(F("c: move to 90"));
  Serial.println(F("x: STOP / HOLD (mode off)"));
  Serial.println(F("z: release (detach)"));
  Serial.println(F("number + Enter: move to that angle (e.g. 120)"));
  Serial.print(F("r: REHAB ")); Serial.print(REHAB_MIN); Serial.print(F(" <-> "));
  Serial.print(REHAB_MAX); Serial.print(F(" x")); Serial.println(REHAB_REPS);
  Serial.print(F("h: HOLD at ")); Serial.print(HOLD_ANGLE); Serial.print(F(" for "));
  Serial.print(HOLD_MS / 1000); Serial.println(F(" s, then 0"));
  Serial.print(F("BUTTON(D4): short = ")); Serial.print(BUTTON_ANGLE); Serial.println(F(" <-> 0 toggle"));
  Serial.println(F("BUTTON(D4): double tap = STOP, 2s = REHAB, 5s = HOLD"));
  Serial.print(F("mode: "));
  Serial.println(CONTINUOUS ? F("360 (continuous)") : F("180 (position)"));
}

void startMotor() {
  angle = START_ANGLE;
  target = START_ANGLE;
  motor.write(angle);        // attach 전에 시작 값을 정해 갑자기 튀지 않게 함
  motor.attach(MOTOR_PIN);
  enabled = true;
  mode = MANUAL;
  lastStep = millis();
  Serial.print(F("START: "));
  Serial.println(START_ANGLE);
}

void releaseMotor() {
  if (CONTINUOUS) {
    motor.write(STOP_VALUE);
    delay(50);
  }
  motor.detach();
  enabled = false;
  mode = MANUAL;
  angle = START_ANGLE;
  target = START_ANGLE;
}

void stopMotor() {
  if (!enabled) {
    Serial.println(F("Not started"));
    return;
  }
  mode = MANUAL;             // 재활/유지 모드 종료
  if (CONTINUOUS) {
    releaseMotor();
    Serial.println(F("STOP (360 servo, detached)"));
  } else {
    target = angle;          // 지금 각도에서 멈춰 버팀
    motor.write(angle);
    Serial.print(F("STOP / HOLD: "));
    Serial.println(angle);
  }
}

void setTarget(int t) {
  if (!enabled) {
    Serial.println(F("Not started (press s)"));
    return;
  }
  mode = MANUAL;             // 직접 각도를 주면 모드 취소
  if (CONTINUOUS) {          // 360도 서보: 값 = 속도, 바로 보냄
    angle = target = constrain(t, 0, 180);
    motor.write(angle);
    Serial.print(F("SPEED VALUE: "));
    Serial.println(angle);
    return;
  }
  target = constrain(t, MIN_ANGLE, MAX_ANGLE);
  Serial.print(F("TARGET: "));
  Serial.println(target);
}

void applyNumber() {
  if (numActive) {
    setTarget(numBuf);
    numBuf = 0;
    numActive = false;
  }
}

// ---------- 재활 / 유지 / 버튼 ----------
bool readyForMode() {
  if (CONTINUOUS) {
    Serial.println(F("180 servo only"));
    return false;
  }
  if (!enabled) startMotor();
  return true;
}

void startRehab() {
  if (!readyForMode()) return;
  mode = REHAB;
  phase = 0;
  rep = 0;
  target = REHAB_MAX;
  Serial.print(F("REHAB START: ")); Serial.print(REHAB_MIN); Serial.print(F(" <-> "));
  Serial.print(REHAB_MAX); Serial.print(F(" x")); Serial.println(REHAB_REPS);
}

void startHold() {
  if (!readyForMode()) return;
  mode = HOLD;
  phase = 0;
  target = HOLD_ANGLE;
  Serial.print(F("HOLD START: go to ")); Serial.println(HOLD_ANGLE);
}

void buttonToggle() {
  if (!readyForMode()) return;
  mode = MANUAL;             // 모드 실행 중이면 취소
  if (target >= BUTTON_ANGLE / 2) target = MIN_ANGLE;   // 굽힌 쪽이면 -> 0도
  else target = BUTTON_ANGLE;                            // 편 쪽이면 -> 180도
  Serial.print(F("BUTTON -> "));
  Serial.println(target);
}

void handleButton() {
  bool r = digitalRead(BUTTON_PIN);
  unsigned long now = millis();

  if (r != lastBtn && now - lastBtnChange > DEBOUNCE_MS) {
    lastBtnChange = now;
    lastBtn = r;
    if (r == LOW) {
      pressStart = now;                      // 누른 순간 기록
      if (pendingTap && now - tapTime <= DOUBLE_TAP_MS) {  // 0.2초 안에 또 누름
        secondTap = true;
        if (mode != MANUAL) {                // [추가] 재활/유지 모드 중: 두 번째 누르는 순간 바로 정지
          Serial.println(F("DOUBLE TAP (mode) -> STOP"));
          stopMotor();
          modeStopped = true;
        }
      }
      pendingTap = false;
    } else if (modeStopped) {                // [추가] 정지에 쓴 두 번째 탭은 뗄 때 아무것도 안 함
      modeStopped = false;
      secondTap = false;
      digitalWrite(LED_PIN, LOW);
    } else {                                 // 뗄 때 얼마나 눌렀는지 보고 결정
      unsigned long held = now - pressStart;
      digitalWrite(LED_PIN, LOW);
      if (held >= LONGER_MS)      startHold();     // 5초 이상: 유지 모드
      else if (held >= LONG_MS)   startRehab();    // 2초 이상: 재활 모드
      else if (secondTap) {                        // 두 번 탭: 그 자리에서 정지
        Serial.println(F("DOUBLE TAP"));
        stopMotor();
      } else {                                     // 한 번 탭: 0.2초 기다려 보고 결정
        pendingTap = true;
        tapTime = now;
      }
      secondTap = false;
    }
  }

  // 한 번만 누르고 0.2초 동안 더 안 누르면 -> 굽힘/폄
  if (pendingTap && lastBtn == HIGH && now - tapTime > DOUBLE_TAP_MS) {
    pendingTap = false;
    if (mode == MANUAL) buttonToggle();
    // [추가] 재활/유지 모드 중 한 번만 누른 건 무시 (두 번 탭이 조금 늦어도 엉뚱하게 안 움직이게)
  }

  // 누르고 있는 동안 L 불빛으로 시간 알려주기
  if (lastBtn == LOW) {
    unsigned long held = now - pressStart;
    if (held >= LONGER_MS)    digitalWrite(LED_PIN, (now / 150) % 2);  // 5초: 깜빡임
    else if (held >= LONG_MS) digitalWrite(LED_PIN, HIGH);             // 2초: 켜짐
  }
}

void updateMode() {
  if (!enabled || CONTINUOUS) return;
  bool reached = (angle == target);
  unsigned long now = millis();

  if (mode == REHAB) {
    switch (phase) {
      case 0:   // 굽히는 중
        if (reached) { phase = 1; phaseStart = now; }
        break;
      case 1:   // 위에서 대기
        if (now - phaseStart >= REHAB_WAIT_MS) { target = REHAB_MIN; phase = 2; }
        break;
      case 2:   // 펴는 중
        if (reached) {
          rep++;
          Serial.print(F("REHAB rep ")); Serial.print(rep);
          Serial.print(F("/")); Serial.println(REHAB_REPS);
          if (rep >= REHAB_REPS) {
            mode = MANUAL;
            Serial.println(F("REHAB DONE"));
          } else {
            phase = 3;
            phaseStart = now;
          }
        }
        break;
      case 3:   // 아래에서 대기
        if (now - phaseStart >= REHAB_WAIT_MS) { target = REHAB_MAX; phase = 0; }
        break;
    }
  } else if (mode == HOLD) {
    if (phase == 0 && reached) {
      phase = 1;
      phaseStart = now;
      Serial.print(F("HOLDING at ")); Serial.print(HOLD_ANGLE);
      Serial.print(F(" for ")); Serial.print(HOLD_MS / 1000); Serial.println(F(" s"));
    } else if (phase == 1 && now - phaseStart >= HOLD_MS) {
      target = HOLD_RETURN;
      phase = 2;
      Serial.print(F("HOLD DONE -> ")); Serial.println(HOLD_RETURN);
    } else if (phase == 2 && reached) {
      mode = MANUAL;
    }
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  printHelp();
}

void loop() {
  handleButton();

  while (Serial.available() > 0) {
    char cmd = Serial.read();
    lastChar = millis();

    if (cmd >= '0' && cmd <= '9') {          // 숫자 모으기
      numBuf = numBuf * 10 + (cmd - '0');
      if (numBuf > 999) numBuf = 999;
      numActive = true;
      continue;
    }
    applyNumber();                           // 숫자 뒤에 Enter나 다른 글자가 오면 적용

    if (cmd >= 'A' && cmd <= 'Z') cmd += 'a' - 'A';
    switch (cmd) {
      case 's':
        if (!enabled) startMotor();
        else Serial.println(F("Already started"));
        break;
      case 'a': setTarget(A_ANGLE); break;
      case 'd': setTarget(D_ANGLE); break;
      case 'c': setTarget(CONTINUOUS ? STOP_VALUE : 90); break;
      case 'x': stopMotor(); break;
      case 'z':
        if (enabled) { releaseMotor(); Serial.println(F("RELEASED")); }
        else Serial.println(F("Not started"));
        break;
      case 'r': startRehab(); break;
      case 'h': startHold(); break;
      default:
        break;                               // 줄바꿈 등은 무시
    }
  }

  // 줄바꿈 없이 숫자만 보냈을 때도 0.3초 뒤 적용
  if (numActive && millis() - lastChar > 300) applyNumber();

  // 1도씩 목표로 이동
  if (enabled && !CONTINUOUS && millis() - lastStep >= STEP_MS) {
    lastStep = millis();
    if (angle < target) {
      angle++;
      motor.write(angle);
    } else if (angle > target) {
      angle--;
      motor.write(angle);
    }
  }

  // 재활 / 유지 모드 진행
  updateMode();
}
