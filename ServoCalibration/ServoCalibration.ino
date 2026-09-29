#include <ESP32Servo.h>

// Sketch chuan doan rieng: xuat truc tiep do rong xung (microgiay) ra 1 servo,
// khong qua buoc quy doi goc 0-180 nhu Servo::write(). Dung de xac dinh:
//   - Servo la loai positional (giu goc) hay continuous-rotation (quay lien tuc)
//   - Neu la continuous-rotation: tim dung diem dung (neutral) thuc te cua no
//
// Cach dung: mo Serial Monitor (115200, Newline), go 1 so nguyen (vd 1500)
// roi Enter -> servo nhan ngay xung do rong day.
//   ~500-600us  = mot dau (0 do neu positional, quay full toc 1 chieu neu continuous)
//   ~1500us     = giua (90 do neu positional, DUNG neu continuous can chinh dung)
//   ~2400-2500us= dau con lai (180 do neu positional, quay full toc chieu nguoc lai neu continuous)
//
// Neu servo DUNG han va giu nguyen o 1 gia tri roi ban doi gia tri khac no
// CHUYEN SANG VI TRI KHAC va DUNG LAI (khong tu quay tiep) -> do la servo
// positional binh thuong.
// Neu no CU QUAY LIEN TUC (khong dung) tru mot khoang gia tri hep quanh ~1500us
// (vung "dead band") -> do la servo continuous-rotation, gia tri lam no dung
// (hoac gan dung nhat) chinh la diem trim/neutral thuc te cua no.

#define SERVO_PIN 18

Servo testServo;

void setup() {
  Serial.begin(115200);
  testServo.setPeriodHertz(50);
  testServo.attach(SERVO_PIN, 500, 2500);

  Serial.println("ServoCalibration san sang.");
  Serial.println("Go 1 so nguyen (400-2600, don vi microgiay) roi Enter de xuat xung.");
  Serial.println("Vd: 500 , 1500 , 2500 -> quan sat servo dung han hay quay lien tuc.");
}

void loop() {
  if (Serial.available() > 0) {
    int us = Serial.parseInt();

    // don sach phan con lai cua dong (vd ky tu newline) truoc khi doc lenh ke tiep
    while (Serial.available() > 0 && Serial.peek() != '\n') {
      Serial.read();
    }
    if (Serial.available() > 0) {
      Serial.read();
    }

    if (us < 400 || us > 2600) {
      Serial.println("Loi: gia tri ngoai khoang hop le (400-2600 us).");
      return;
    }

    // Ghi 2 lan lien tiep: lan ghi dau ngay sau attach() doi khi khong kip
    // chot vao thanh ghi PWM (LEDC) truoc chu ky, bi bo qua -> ghi lan 2
    // moi thuc su ap dung. Delay ngan de chac chan qua it nhat 1 chu ky 50Hz.
    testServo.writeMicroseconds(us);
    delay(25);
    testServo.writeMicroseconds(us);
    Serial.print("Da xuat xung: ");
    Serial.print(us);
    Serial.println(" us");
  }
}
