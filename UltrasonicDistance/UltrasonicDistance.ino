// Do khoang cach bang HC-SR04 dung yen (KHONG servo): gui "khoang_cach_cm" qua Serial cho
// man hinh rada tren PC (RadarDisplay/distance_display.py). Dung de thu rieng cam bien truoc khi
// lap servo (ban quet: UltrasonicRadar/). Xem CLAUDE.md muc "Radar siêu âm HC-SR04".
//
// Dau day (giong UltrasonicRadar, bo qua phan servo):
//   HC-SR04: VCC -> 5V, GND -> GND (chung voi ESP32), TRIG -> GPIO TRIG_PIN
//            ECHO ra muc 5V -> BAT BUOC qua cau chia ap (1k noi ECHO->GPIO, 2k noi GPIO->GND)
//            truoc khi vao GPIO ECHO_PIN, vi chan ESP32 chi chiu 3.3V.
#define TRIG_PIN  25
#define ECHO_PIN  26

#define MEASURE_CYCLE_MS  60  // khoang cach toi thieu giua 2 lan phat xung (datasheet HC-SR04: > 60ms)
#define MIN_DISTANCE_CM   2   // HC-SR04 do duoc 2-400 cm
#define MAX_DISTANCE_CM   400
#define ECHO_TIMEOUT_US   (MAX_DISTANCE_CM * 58UL)  // song am di-ve mat ~58 us moi cm

unsigned long lastPingMillis = 0;

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);

  Serial.println("UltrasonicDistance san sang.");
  Serial.println("Moi dong: khoang_cach_cm  (-1 = khong co echo)");
}

void loop() {
  waitForNextPing();

  float cm = measureDistanceCm();
  if (cm < 0) {
    Serial.println(-1);
  } else {
    Serial.println(cm, 1);
  }
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
