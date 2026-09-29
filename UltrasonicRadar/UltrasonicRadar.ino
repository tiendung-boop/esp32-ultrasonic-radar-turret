#include <ESP32Servo.h>

// Radar sieu am: HC-SR04 gan tren servo pan, quet qua lai roi gui "goc,khoang_cach_cm" qua Serial
// cho man hinh rada tren PC (RadarDisplay/radar_display.py). Xem CLAUDE.md muc "Radar siêu âm HC-SR04".
//
// Dau day:
//   Servo  : tin hieu -> GPIO PAN_SERVO_PIN, nguon 5V RIENG, chung GND voi ESP32
//   HC-SR04: VCC -> 5V, GND -> GND (chung), TRIG -> GPIO TRIG_PIN
//            ECHO ra muc 5V -> BAT BUOC qua cau chia ap (1k noi ECHO->GPIO, 2k noi GPIO->GND)
//            truoc khi vao GPIO ECHO_PIN, vi chan ESP32 chi chiu 3.3V.
#define PAN_SERVO_PIN  18
#define TRIG_PIN       25
#define ECHO_PIN       26

#define SWEEP_MIN_DEG   15    // khong quet sat 0/180 de servo khong ket/rung o hai dau
#define SWEEP_MAX_DEG   165
#define SWEEP_STEP_DEG  2     // chum tia HC-SR04 rong ~15 do nen buoc 2 do la du day

#define MEASURE_CYCLE_MS  60  // khoang cach toi thieu giua 2 lan phat xung (datasheet: > 60ms), dong thoi la thoi gian cho servo on dinh
#define MIN_DISTANCE_CM   2   // HC-SR04 do duoc 2-400 cm
#define MAX_DISTANCE_CM   400
#define ECHO_TIMEOUT_US   (MAX_DISTANCE_CM * 58UL)  // song am di-ve mat ~58 us moi cm

Servo panServo;

int angle = SWEEP_MIN_DEG;
int direction = 1;
unsigned long lastPingMillis = 0;

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);

  panServo.setPeriodHertz(50);
  panServo.attach(PAN_SERVO_PIN, 500, 2400);
  panServo.write(angle);
  delay(600);  // cho servo ve dau hanh trinh quet

  Serial.println("UltrasonicRadar san sang.");
  Serial.println("Moi dong: goc,khoang_cach_cm  (khoang cach -1 = khong co echo)");
}

void loop() {
  panServo.write(angle);
  waitForNextPing();

  float cm = measureDistanceCm();

  Serial.print(angle);
  Serial.print(',');
  if (cm < 0) {
    Serial.println(-1);
  } else {
    Serial.println(cm, 1);
  }

  advanceAngle();
}

void waitForNextPing() {
  while (millis() - lastPingMillis < MEASURE_CYCLE_MS) {
    delay(1);
  }
  lastPingMillis = millis();
}

// Tra ve khoang cach (cm), hoac -1 neu khong co echo / ngoai tam do.
float measureDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return -1;

  float cm = duration / 58.0;
  if (cm < MIN_DISTANCE_CM || cm > MAX_DISTANCE_CM) return -1;
  return cm;
}

void advanceAngle() {
  angle += direction * SWEEP_STEP_DEG;
  if (angle >= SWEEP_MAX_DEG) {
    angle = SWEEP_MAX_DEG;
    direction = -1;
  } else if (angle <= SWEEP_MIN_DEG) {
    angle = SWEEP_MIN_DEG;
    direction = 1;
  }
}
