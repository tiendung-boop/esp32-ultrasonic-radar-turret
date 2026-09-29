#include <ESP32Servo.h>
#include <math.h>

// Chan GPIO va gioi han goc: xem CLAUDE.md muc "Target hardware" / "Đề xuất chân GPIO".
// Chinh PAN_MIN/MAX, TILT_MIN/MAX theo khung co khi that truoc khi lap servo that.
#define PAN_SERVO_PIN   18
#define TILT_SERVO_PIN  19
#define LASER_PIN       21

// Do rong xung ung voi 0 do va 180 do cua tung servo. Mac dinh 500/2500 hop voi da so
// MG995/MG996R/SG90. Neu servo keu/rung o hai dau: chay ServoCalibration, go dan tu 1500
// ra hai phia den khi servo KHONG quay them nua -> lay gia tri do lui vao ~50us.
#define PAN_MIN_US   500
#define PAN_MAX_US   2500
#define TILT_MIN_US  500
#define TILT_MAX_US  2500

// Khong de sat 0/180: servo (nhat la MG995) de ti vao chan co khi o hai dau.
#define PAN_MIN   10
#define PAN_MAX   170
#define TILT_MIN  10
#define TILT_MAX  170

// Lam muot: van toc gioi han + tang/giam toc tu tu (profile hinh thang) thay vi nhay deu
// tung buoc -> khoi hanh va dung em, khong giat, khong keo dong dinh lon.
#define UPDATE_INTERVAL_MS     20     // chu ky cap nhat servo (= 1 chu ky PWM 50Hz)
#define MAX_SPEED_DEG_PER_S    120.0  // MG995 toi da ~300 do/s khong tai; de du phong khi co tai
#define MAX_ACCEL_DEG_PER_S2   600.0  // giam neu khung nang con rung khi dung

#define AUTO_STOP_TIMEOUT_MS 2000  // qua thoi gian nay khong co lenh moi -> tat laser (an toan)
#define HOLD_SERVO_ON_TIMEOUT 1    // 1 = khi timeout servo van giu goc (tilt khong bi sup do trong luong)
                                   // 0 = detach servo luon khi timeout (nhu lenh STOP)

struct Axis {
  Servo servo;
  int pin;
  int minUs, maxUs;
  float pos;       // goc hien tai dang xuat ra servo (do, so thuc -> xung min mang hon)
  float vel;       // do/s
  float target;
};

Axis pan  = { Servo(), PAN_SERVO_PIN,  PAN_MIN_US,  PAN_MAX_US,  90, 0, 90 };
Axis tilt = { Servo(), TILT_SERVO_PIN, TILT_MIN_US, TILT_MAX_US, 90, 0, 90 };

bool laserOn = false;

unsigned long lastCommandMillis = 0;
unsigned long lastUpdateMillis = 0;
bool autoStopped = false;

String inputBuffer = "";

void setup() {
  Serial.begin(115200);

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);

  attachAxis(pan);
  attachAxis(tilt);

  lastCommandMillis = millis();

  Serial.println("LaserTurretControl san sang.");
  Serial.println("Gui lenh dang: pan,tilt,laser  (vd: 90,45,1)");
}

void loop() {
  readSerialCommands();
  applySafetyTimeout();
  updateServoSmoothing();
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      processCommand(inputBuffer);
      inputBuffer = "";
    } else if (c != '\r') {
      inputBuffer += c;
    }
  }
}

void processCommand(const String &line) {
  if (line.length() == 0) return;

  String trimmed = line;
  trimmed.trim();
  if (trimmed.equalsIgnoreCase("STOP")) {
    emergencyStop("DA DUNG KHAN CAP (lenh STOP tay): servo detach, laser tat.", true);
    lastCommandMillis = millis();
    Serial.println("Gui lenh pan,tilt,laser binh thuong de servo tu gan (attach) lai.");
    return;
  }

  int panDeg, tiltDeg, laser;
  int parsed = sscanf(line.c_str(), "%d,%d,%d", &panDeg, &tiltDeg, &laser);
  if (parsed != 3) {
    Serial.print("Loi: lenh khong hop le -> ");
    Serial.println(line);
    return;
  }

  if (!pan.servo.attached()) attachAxis(pan);
  if (!tilt.servo.attached()) attachAxis(tilt);

  pan.target = constrain(panDeg, PAN_MIN, PAN_MAX);
  tilt.target = constrain(tiltDeg, TILT_MIN, TILT_MAX);
  laserOn = (laser != 0);
  digitalWrite(LASER_PIN, laserOn ? HIGH : LOW);

  lastCommandMillis = millis();
  autoStopped = false;

  Serial.print("OK pan=");
  Serial.print((int)pan.target);
  Serial.print(" tilt=");
  Serial.print((int)tilt.target);
  Serial.print(" laser=");
  Serial.println(laserOn ? 1 : 0);
}

// Gan servo va xuat ngay goc hien tai (pos). Sau detach servo van nam yen o goc cu nen
// giu nguyen pos -> gan lai khong bi nhay. Ghi 2 lan: lan ghi dau ngay sau attach() doi khi
// khong kip chot vao thanh ghi PWM (LEDC) truoc chu ky, bi bo qua.
void attachAxis(Axis &a) {
  a.servo.setPeriodHertz(50);
  a.servo.attach(a.pin, a.minUs, a.maxUs);
  writeAxis(a);
  delay(25);
  writeAxis(a);
  a.vel = 0;
}

void writeAxis(Axis &a) {
  float us = a.minUs + (a.maxUs - a.minUs) * a.pos / 180.0;
  a.servo.writeMicroseconds((int)round(us));
}

void emergencyStop(const char *reason, bool detachServos) {
  laserOn = false;
  digitalWrite(LASER_PIN, LOW);

  // Dung tai cho: dich = vi tri hien tai, khong reset ve 90 (servo that van o goc cu,
  // reset ve 90 se lam no nhay thang ve giua voi toc do toi da khi gan lai).
  pan.target = pan.pos;
  tilt.target = tilt.pos;
  pan.vel = 0;
  tilt.vel = 0;

  if (detachServos) {
    // Cat han tin hieu PWM -> dang tin cay hon de dung servo khi servo loi.
    pan.servo.detach();
    tilt.servo.detach();
  }

  Serial.println(reason);
}

void applySafetyTimeout() {
  if (!autoStopped && millis() - lastCommandMillis > AUTO_STOP_TIMEOUT_MS) {
    if (HOLD_SERVO_ON_TIMEOUT) {
      emergencyStop("Canh bao: qua 2s khong co lenh moi -> tat laser, servo giu nguyen goc.", false);
    } else {
      emergencyStop("Canh bao: qua 2s khong co lenh moi -> tu dong STOP (servo detach, laser tat).", true);
    }
    autoStopped = true;
  }
}

void updateServoSmoothing() {
  unsigned long now = millis();
  if (now - lastUpdateMillis < UPDATE_INTERVAL_MS) return;
  float dt = (now - lastUpdateMillis) / 1000.0;
  lastUpdateMillis = now;
  if (dt > 0.1) dt = 0.1;  // loop bi cham bat thuong -> khong cho nhay mot buoc qua dai

  stepAxis(pan, dt);
  stepAxis(tilt, dt);
}

// Profile hinh thang: tang toc den MAX_SPEED, bat dau giam toc dung luc de dung vua khit o dich.
// Dich doi giua chung (muc tieu di chuyen) thi van toc doi chieu tu tu, khong dao chieu dot ngot.
void stepAxis(Axis &a, float dt) {
  if (!a.servo.attached()) return;

  float diff = a.target - a.pos;
  if (fabs(diff) < 0.05 && fabs(a.vel) < 1.0) {
    a.pos = a.target;
    a.vel = 0;
    writeAxis(a);
    return;
  }

  // van toc lon nhat van con kip phanh dung tai dich: v^2 = 2*a*s
  float brakeSpeed = sqrt(2.0 * MAX_ACCEL_DEG_PER_S2 * fabs(diff));
  float desiredVel = copysign(min((float)MAX_SPEED_DEG_PER_S, brakeSpeed), diff);

  float maxDv = MAX_ACCEL_DEG_PER_S2 * dt;
  float dv = constrain(desiredVel - a.vel, -maxDv, maxDv);
  a.vel += dv;

  float step = a.vel * dt;
  // khong vuot qua dich trong 1 buoc
  if ((diff > 0 && step > diff) || (diff < 0 && step < diff)) {
    step = diff;
    a.vel = 0;
  }
  a.pos += step;
  writeAxis(a);
}
