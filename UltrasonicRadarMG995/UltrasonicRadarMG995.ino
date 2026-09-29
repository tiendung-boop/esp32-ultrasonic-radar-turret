#include <ESP32Servo.h>

// Radar sieu am - ban danh cho servo MG995 (TowerPro, analog, banh rang kim loai).
// Giao thuc Serial GIONG HET UltrasonicRadar: moi dong "goc,khoang_cach_cm" (-1 = khong co echo)
// -> dung chung RadarDisplay/radar_display.py, khong can sua gi phia PC.
//
// Khac ban goc o cho:
//   1. Xuat xung truc tiep bang writeMicroseconds() voi SERVO_MIN_US/SERVO_MAX_US chinh duoc.
//      MG995 moi lo mot khac, nhieu con chi di het hanh trinh trong ~1000-2000us; ep xung
//      qua tam lam servo ti vao chan co khi -> rung, keu, keo dong lon (co the reset ESP32).
//      Do tam that bang ServoCalibration truoc (xem ghi chu SERVO_MIN_US ben duoi).
//   2. Cho servo on dinh (SERVO_SETTLE_MS) truoc khi do: MG995 la servo analog, cham hon va
//      rung/vot lo hon servo so, do ngay khi dang lac thi HC-SR04 bi nhieu.
//   3. Moi goc do SAMPLES_PER_STEP lan roi lay trung vi: MG995 keo dong dinh ~1A+ gay nhieu
//      nguon, trung vi loai duoc cac so do nhay bat thuong.
//
// LUU Y: nhieu MG995 hang nhai ban ra la loai quay lien tuc 360 do (continuous rotation),
// KHONG giu duoc goc -> khong dung cho radar duoc. Kiem tra bang ServoCalibration truoc.
//
// Dau day:
//   MG995  : tin hieu (cam) -> GPIO PAN_SERVO_PIN, do -> 5-6V nguon RIENG (>= 2A, tot nhat 3A),
//            nau -> GND chung voi ESP32. Tu dien phan 1000uF sat nguon servo.
//   HC-SR04: VCC -> 5V, GND -> GND (chung), TRIG -> GPIO TRIG_PIN
//            ECHO ra muc 5V -> BAT BUOC qua cau chia ap (1k noi ECHO->GPIO, 2k noi GPIO->GND)
//            truoc khi vao GPIO ECHO_PIN, vi chan ESP32 chi chiu 3.3V.
#define PAN_SERVO_PIN  18
#define TRIG_PIN       25
#define ECHO_PIN       26

// Do rong xung ung voi 0 do va 180 do. Mac dinh 500/2500 hop voi da so MG995.
// Neu servo keu/rung o hai dau quet: chay ServoCalibration, go dan tu 1500 xuong (vd 1000, 800,
// 700...) den khi servo KHONG quay them nua -> lay gia tri do + ~50us lam SERVO_MIN_US;
// lam tuong tu tu 1500 len cho SERVO_MAX_US.
#define SERVO_MIN_US   500
#define SERVO_MAX_US   2500

#define SWEEP_MIN_DEG   15    // khong quet sat 0/180 de servo khong ket/rung o hai dau
#define SWEEP_MAX_DEG   165
#define SWEEP_STEP_DEG  3     // chum tia HC-SR04 rong ~15 do nen buoc 3 do van du day

#define SERVO_SETTLE_MS    60   // MG995 ~0.2s/60 do -> 3 do mat ~10ms, cong them thoi gian het rung
#define STARTUP_MOVE_MS    1200 // luc boot servo co the dang o bat ky dau -> cho di toi dau hanh trinh
#define SAMPLES_PER_STEP   3    // so lan do moi goc, lay trung vi (dat 1 de tat loc)
#define PING_GAP_MS        35   // khoang cach giua 2 lan phat xung; > thoi gian echo toi da + du am

#define MIN_DISTANCE_CM   2
#define MAX_DISTANCE_CM   300   // ha tu 400 xuong 300 de rut timeout, do nhanh hon (HC-SR04 xa hon 3m it tin cay)
#define ECHO_TIMEOUT_US   (MAX_DISTANCE_CM * 58UL)  // song am di-ve mat ~58 us moi cm

Servo panServo;

int angle = SWEEP_MIN_DEG;
int direction = 1;

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);

  panServo.setPeriodHertz(50);
  panServo.attach(PAN_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  moveServo(angle);
  delay(STARTUP_MOVE_MS);

  Serial.println("UltrasonicRadarMG995 san sang.");
  Serial.println("Moi dong: goc,khoang_cach_cm  (khoang cach -1 = khong co echo)");
}

void loop() {
  moveServo(angle);
  delay(SERVO_SETTLE_MS);

  float cm = measureMedianCm();

  Serial.print(angle);
  Serial.print(',');
  if (cm < 0) {
    Serial.println(-1);
  } else {
    Serial.println(cm, 1);
  }

  advanceAngle();
}

void moveServo(int deg) {
  int us = map(deg, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  panServo.writeMicroseconds(us);
}

// Trung vi cua SAMPLES_PER_STEP lan do hop le; -1 neu qua nua so lan khong co echo.
float measureMedianCm() {
  float samples[SAMPLES_PER_STEP];
  int valid = 0;

  for (int i = 0; i < SAMPLES_PER_STEP; i++) {
    if (i > 0) delay(PING_GAP_MS);
    float cm = measureDistanceCm();
    if (cm >= 0) samples[valid++] = cm;
  }

  if (valid == 0 || valid * 2 < SAMPLES_PER_STEP) return -1;

  // sap xep chen, mang nho
  for (int i = 1; i < valid; i++) {
    float v = samples[i];
    int j = i - 1;
    while (j >= 0 && samples[j] > v) {
      samples[j + 1] = samples[j];
      j--;
    }
    samples[j + 1] = v;
  }
  return samples[valid / 2];
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
