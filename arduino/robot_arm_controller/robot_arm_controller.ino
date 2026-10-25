/**
 * ======================================================================================
 * 6-DOF / 3-6 DOF ENDÜSTRİYEL ROBOT KOLU ARDUINO C/C++ KONTROL YAZILIMI
 * Üniversite Robotik ve Mekatronik Öğrenci Projeleri İçin Özel Geliştirilmiştir.
 * ======================================================================================
 * Senaryo:
 * 1. Robot ilk açıldığında tüm eklemleri 0° (orta kalibrasyon) olan HOME pozisyonuna gider.
 * 2. Öğrencinin tanımladığı A Noktasına (X_A, Y_A, Z_A) sarsıntısız geçişle hareket eder.
 * 3. Belirlenen bekleme süresinden sonra B Noktasına (X_B, Y_B, Z_B) geçer.
 * 4. Döngüsel olarak bu hareketi tekrarlar veya Seri Port üzerinden anlık komut alır.
 *
 * Donanım Uyumluluğu:
 * - Standart Arduino Uno / Nano / Mega / ESP32
 * - PWM Servo Motorlar (MG996R, SG90, RDS3115 vb.)
 * - Doğrudan PWM Pinleri veya PCA9685 I2C 16-Kanal Sürücü (#define USE_PCA9685)
 * ======================================================================================
 */

#include <Arduino.h>
#include <math.h>

// --------------------------------------------------------------------------------------
// 1. DONANIM VE SÜRÜCÜ AYARLARI
// --------------------------------------------------------------------------------------
// 0: Doğrudan Arduino PWM pinleri (Servo.h kütüphanesi kullanılır)
// 1: PCA9685 I2C 16-Kanal PWM Sürücü Kartı (A4=SDA, A5=SCL)
#define USE_PCA9685 0
#define ACTIVE_DOF  6 // Aktif Eksen Sayısı: 3, 4, 5 veya 6 seçilebilir (Varsayılan: 6)

#if USE_PCA9685
  #include <Wire.h>
  #include <Adafruit_PWMServoDriver.h>
  Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
  #define SERVOMIN  150 // 0 derece için pulse uzunluğu (4096 üzerinden)
  #define SERVOMAX  600 // 180 derece için pulse uzunluğu
#else
  #include <Servo.h>
  // Doğrudan Arduino Dijital PWM Pin Tanımları:
  const int SERVO_PINS[6] = {3, 5, 6, 9, 10, 11}; // Eklem 1...6
  Servo servos[6];
#endif

// --------------------------------------------------------------------------------------
// 2. ROBOTUN MEKANİK BAĞLANTI UZUNLUKLARI (DH Parametreleri - mm Cinsinden)
// Öğrencinin 3D baskı veya alüminyum robot ölçülerine göre güncellenebilir:
// --------------------------------------------------------------------------------------
const float LINK_D1 = 150.0; // Taban yüksekliği (Eklem 1 ekseni) [mm]
const float LINK_A2 = 160.0; // Omuz - Dirsek kol uzunluğu        [mm]
const float LINK_D4 = 150.0; // Dirsek - Bilek ön kol uzunluğu   [mm]
const float LINK_D6 =  70.0; // Bilek merkezi - Uç Takım (TCP)   [mm]

// --------------------------------------------------------------------------------------
// 3. ÖĞRENCİ HEDEF NOKTALARI (Kartezyen Koordinatlar - mm Cinsinden)
// --------------------------------------------------------------------------------------
// A NOKTASI (Örnek: Malzeme Alma Noktası)
const float POINT_A_X =  180.0;
const float POINT_A_Y = -100.0;
const float POINT_A_Z =  140.0;

// B NOKTASI (Örnek: Malzeme Bırakma / Kaynak Bitiş Noktası)
const float POINT_B_X =  160.0;
const float POINT_B_Y =  120.0;
const float POINT_B_Z =  200.0;

// Hedef Takım Açısı (Roll, Pitch, Yaw - Derece cinsinden)
// Pitch = 90°: Alet ucu tam aşağıya bakar (Pick & Place / Dikey Kaynak)
const float TOOL_ROLL_DEG  =   0.0;
const float TOOL_PITCH_DEG =  90.0;
const float TOOL_YAW_DEG   =   0.0;

// --------------------------------------------------------------------------------------
// 4. HAREKET VE YUMUŞATMA (Anti-Jerk) PARAMETRELERİ
// --------------------------------------------------------------------------------------
// Her 1 derecelik servo hareketi arasındaki gecikme (milisaniye)
// Değeri artırırsanız robot daha yavaş ve sarsıntısız hareket eder.
const int STEP_DELAY_MS   = 15;  // 10 - 25 ms arası idealdir
const int DWELL_TIME_MS   = 1500; // Noktalarda bekleme süresi (1.5 sn)

// Servo Kalibrasyon ve Sıfır Noktası Dengeleme (Fiziksel 90° = Kinematik 0°)
// Kinematik açıları (-90° ile +90°) servo fiziksel açısına (0° - 180°) dönüştürür.
const float SERVO_CALIB_CENTER[6] = {90.0, 90.0, 90.0, 90.0, 90.0, 90.0};
const int   SERVO_DIR[6]          = { 1,   1,   1,   1,   1,   1}; // Yön tersleme: -1 veya 1
const float SERVO_MIN_DEG[6]      = { 0.0, 10.0, 15.0,  0.0,  0.0,  0.0};
const float SERVO_MAX_DEG[6]      = {180.0, 170.0, 165.0, 180.0, 180.0, 180.0};

// Mevcut ve Hedef Açı Durumları (Derece cinsinden)
float currentJointsDeg[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Kinematik (0° = HOME)
float targetJointsDeg[6]  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// Otomatik döngü bayrağı (Seri porttan kapatılıp açılabilir)
bool autoLoopEnabled = true;

// --------------------------------------------------------------------------------------
// 5. FONKSİYON PROTOTİPLERİ
// --------------------------------------------------------------------------------------
bool solveInverseKinematics(float x, float y, float z, float rollDeg, float pitchDeg, float yawDeg, float qDegOut[6]);
void writeServoAngle(int jointIndex, float physicalAngle);
void moveToJointAnglesSmooth(const float targetAngles[6]);
void moveToCartesian(float x, float y, float z, const char* pointName);
void checkSerialCommands();

// --------------------------------------------------------------------------------------
// SETUP: Başlangıç Ayarları
// --------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000); // Seri port başlatma toleransı

  Serial.println(F("\n========================================================"));
  Serial.println(F("   6-DOF ROBOT KOLU ARDUINO KONTROL SISTEMI BASLATILDI  "));
  Serial.println(F("========================================================"));

#if USE_PCA9685
  Serial.println(F("Surucu: PCA9685 I2C 16-Kanal PWM Aktif (0x40)"));
  pwm.begin();
  pwm.setPWMFreq(50); // Standart analog/dijital servo 50 Hz
#else
  Serial.println(F("Surucu: Standart Arduino PWM Pinleri (Servo.h)"));
  for (int i = 0; i < ACTIVE_DOF; i++) {
    servos[i].attach(SERVO_PINS[i]);
  }
#endif

  // Başlangıçta aktif servoları doğrudan HOME (0° kinematik = 90° fiziksel) yap
  Serial.println(F("\n[1/3] Robot HOME (Sifir Noktasi) Pozisyonuna Gonderiliyor..."));
  for (int i = 0; i < ACTIVE_DOF; i++) {
    currentJointsDeg[i] = 0.0;
    float phys = SERVO_CALIB_CENTER[i];
    writeServoAngle(i, phys);
  }
  delay(2000); // Mekanizmanın oturması için bekleme

  Serial.println(F("✓ Robot HOME pozisyonunda hazir!"));
  Serial.println(F("Komutlar: 'HOME', 'A', 'B', 'AUTO', 'GO X Y Z'\n"));
}

// --------------------------------------------------------------------------------------
// LOOP: Ana Çalışma Döngüsü
// --------------------------------------------------------------------------------------
void loop() {
  // 1. Seri porttan gelen öğrenci komutlarını kontrol et
  checkSerialCommands();

  if (autoLoopEnabled) {
    // 2. A Noktasına Git
    Serial.println(F(">>> A NOKTASINA HAREKET BASLATILIYOR..."));
    moveToCartesian(POINT_A_X, POINT_A_Y, POINT_A_Z, "A Noktasi");
    delay(DWELL_TIME_MS);

    // 3. B Noktasına Git
    Serial.println(F(">>> B NOKTASINA HAREKET BASLATILIYOR..."));
    moveToCartesian(POINT_B_X, POINT_B_Y, POINT_B_Z, "B Noktasi");
    delay(DWELL_TIME_MS);

    // 4. Yeniden A Noktasına Dön (veya HOME)
    Serial.println(F(">>> TEKRAR A NOKTASINA DONULUYOR..."));
    moveToCartesian(POINT_A_X, POINT_A_Y, POINT_A_Z, "A Noktasi (Donus)");
    delay(DWELL_TIME_MS);
  } else {
    delay(20);
  }
}

// --------------------------------------------------------------------------------------
// 6. TERS KİNEMATİK ÇÖZÜCÜ (Analitik Pieper Teoremi - Arduino C++)
// Verilen X, Y, Z ve Euler yönelim açılarından eklem açılarını (q1...q6) hesaplar.
// --------------------------------------------------------------------------------------
bool solveInverseKinematics(float x, float y, float z, float rollDeg, float pitchDeg, float yawDeg, float qDegOut[6]) {
  float rRad = rollDeg  * DEG_TO_RAD;
  float pRad = pitchDeg * DEG_TO_RAD;
  float yRad = yawDeg   * DEG_TO_RAD;

  // Z-Y-X Euler Rotasyon Matrisinin 3. sütunu (Takım Z ekseni yaklaşım vektörü)
  float zToolX = cos(yRad) * sin(pRad) * cos(rRad) + sin(yRad) * sin(rRad);
  float zToolY = sin(yRad) * sin(pRad) * cos(rRad) - cos(yRad) * sin(rRad);
  float zToolZ = cos(pRad) * cos(rRad);

  // Adım 1: Bilek Merkezi (Wrist Center - Pw)
  float xw, yw, zw;
#if ACTIVE_DOF == 3
  // 3-DOF: Takım ucu doğrudan önkolun bittiği noktadır
  xw = x;
  yw = y;
  zw = z;
#else
  // 4, 5, 6-DOF: Takım flanş mesafesi (LINK_D6) geri çekilerek bilek merkezi bulunur
  xw = x - LINK_D6 * zToolX;
  yw = y - LINK_D6 * zToolY;
  zw = z - LINK_D6 * zToolZ;
#endif

  // Adım 2: İlk 3 Eklem (q1, q2, q3) Geometrisi
  float r_planar = sqrt(xw * xw + yw * yw);
  float s_height = zw - LINK_D1;
  float D2 = r_planar * r_planar + s_height * s_height;
  float D  = sqrt(D2);

  // Erişim Zarfı Kontrolü
  float maxReach = LINK_A2 + LINK_D4;
  float minReach = fabs(LINK_A2 - LINK_D4);
  if (D > maxReach || D < minReach) {
    Serial.print(F("HATA: Hedef nokta calisma alani disinda! Mesafe D: "));
    Serial.print(D);
    Serial.print(F(" mm, Max: "));
    Serial.println(maxReach);
    return false;
  }

  // q1 Açısı (Taban Dönüşü)
  float q1 = atan2(yw, xw);

  // q3 Açısı (Dirsek - Kosinüs Teoremi)
  float cosGamma = (D2 - LINK_A2 * LINK_A2 - LINK_D4 * LINK_D4) / (2.0 * LINK_A2 * LINK_D4);
  cosGamma = constrain(cosGamma, -1.0, 1.0);
  float q3 = atan2(cosGamma, sqrt(1.0 - cosGamma * cosGamma)); // Elbow-Up dalı

  // q2 Açısı (Omuz)
  float Xp = LINK_A2 + LINK_D4 * sin(q3);
  float Yp = -LINK_D4 * cos(q3);
  float q2 = atan2(s_height, r_planar) - atan2(Yp, Xp);

  // Adım 3: Bilek Oryantasyonu (q4, q5, q6)
  float q4 = 0.0;
  float q5 = 0.0;
  float q6 = 0.0;

#if ACTIVE_DOF >= 4
  q5 = (pitchDeg * DEG_TO_RAD) - (q2 + q3);
#endif
#if ACTIVE_DOF >= 6
  q6 = (yawDeg * DEG_TO_RAD) - q1;
#endif

  // Dereceye dönüştür
  qDegOut[0] = q1 * RAD_TO_DEG;
  qDegOut[1] = q2 * RAD_TO_DEG;
  qDegOut[2] = q3 * RAD_TO_DEG;
  qDegOut[3] = q4 * RAD_TO_DEG;
  qDegOut[4] = q5 * RAD_TO_DEG;
  qDegOut[5] = q6 * RAD_TO_DEG;

  return true;
}

// --------------------------------------------------------------------------------------
// 7. SARSINTISIZ HAREKET MOTORU (Anti-Jerk Interpolator)
// Eklem açılarını kademe kademe yumuşatarak fiziksel servo motorlara iletir.
// --------------------------------------------------------------------------------------
void moveToJointAnglesSmooth(const float targetAngles[6]) {
  // En büyük açısal farkı bul (en çok hareket edecek eklem)
  float maxDiff = 0.0;
  for (int i = 0; i < ACTIVE_DOF; i++) {
    float diff = fabs(targetAngles[i] - currentJointsDeg[i]);
    if (diff > maxDiff) maxDiff = diff;
  }

  // Toplam interpolasyon adım sayısı (en az 10 adım)
  int steps = max(10, (int)(maxDiff * 1.5));

  for (int s = 1; s <= steps; s++) {
    // Kosinüs yumuşatma (S-Curve) katsayısı: 0.0 -> 1.0
    float t = (float)s / (float)steps;
    float factor = 0.5 * (1.0 - cos(t * PI));

    for (int i = 0; i < ACTIVE_DOF; i++) {
      float intermediateKinematic = currentJointsDeg[i] + factor * (targetAngles[i] - currentJointsDeg[i]);
      
      // Kinematik açıyı fiziksel servo açısına dönüştür
      float physicalServoAngle = SERVO_CALIB_CENTER[i] + (intermediateKinematic * SERVO_DIR[i]);
      physicalServoAngle = constrain(physicalServoAngle, SERVO_MIN_DEG[i], SERVO_MAX_DEG[i]);

      writeServoAngle(i, physicalServoAngle);
    }
    delay(STEP_DELAY_MS);
  }

  // Son konumu kaydet
  for (int i = 0; i < ACTIVE_DOF; i++) {
    currentJointsDeg[i] = targetAngles[i];
  }
}

// --------------------------------------------------------------------------------------
// 8. TEKİL SERVO YAZMA SÜRÜCÜSÜ (PWM veya PCA9685)
// --------------------------------------------------------------------------------------
void writeServoAngle(int jointIndex, float physicalAngle) {
  if (jointIndex < 0 || jointIndex >= 6) return;

#if USE_PCA9685
  // 0 - 180 dereceyi SERVOMIN - SERVOMAX pulse aralığına haritala
  int pulse = map((int)physicalAngle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(jointIndex, 0, pulse);
#else
  servos[jointIndex].write((int)physicalAngle);
#endif
}

// --------------------------------------------------------------------------------------
// 9. KARTEZYEN KOORDİNATA GİTME YARDIMCISI
// --------------------------------------------------------------------------------------
void moveToCartesian(float x, float y, float z, const char* pointName) {
  Serial.print(F("Hedef: "));
  Serial.print(pointName);
  Serial.print(F(" -> X: ")); Serial.print(x);
  Serial.print(F(" Y: "));    Serial.print(y);
  Serial.print(F(" Z: "));    Serial.println(z);

  float solvedAngles[6];
  if (solveInverseKinematics(x, y, z, TOOL_ROLL_DEG, TOOL_PITCH_DEG, TOOL_YAW_DEG, solvedAngles)) {
    Serial.print(F("  Hesaplanan Eklem Acilari (q1..q6): ["));
    for (int i = 0; i < 6; i++) {
      Serial.print(solvedAngles[i], 1);
      if (i < 5) Serial.print(F(", "));
    }
    Serial.println(F("] deg"));

    moveToJointAnglesSmooth(solvedAngles);
    Serial.println(F("  ✓ Konuma ulasildi."));
  } else {
    Serial.println(F("  ✗ Kinematik hesaplanamadi! Hedef atlandi."));
  }
}

// --------------------------------------------------------------------------------------
// 10. SERİ PORT KOMUT ARAYÜZÜ (Öğrencilerin Elle Test Edebilmesi İçin)
// --------------------------------------------------------------------------------------
void checkSerialCommands() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "HOME") {
      autoLoopEnabled = false;
      Serial.println(F("[KOMUT] HOME Pozisyonuna gidiliyor..."));
      float homeAngles[6] = {0, 0, 0, 0, 0, 0};
      moveToJointAnglesSmooth(homeAngles);
    } 
    else if (cmd == "A") {
      autoLoopEnabled = false;
      Serial.println(F("[KOMUT] A Noktasina gidiliyor..."));
      moveToCartesian(POINT_A_X, POINT_A_Y, POINT_A_Z, "A Noktasi (Manuel)");
    } 
    else if (cmd == "B") {
      autoLoopEnabled = false;
      Serial.println(F("[KOMUT] B Noktasina gidiliyor..."));
      moveToCartesian(POINT_B_X, POINT_B_Y, POINT_B_Z, "B Noktasi (Manuel)");
    } 
    else if (cmd == "AUTO") {
      autoLoopEnabled = !autoLoopEnabled;
      Serial.print(F("[KOMUT] Otomatik Dongu: "));
      Serial.println(autoLoopEnabled ? F("ACIK (A -> B -> A)") : F("KAPALI"));
    } 
    else if (cmd.startsWith("GO ")) {
      autoLoopEnabled = false;
      // GO X Y Z formatında koordinat ayrıştırma
      float gx, gy, gz;
      int count = sscanf(cmd.c_str(), "GO %f %f %f", &gx, &gy, &gz);
      if (count == 3) {
        moveToCartesian(gx, gy, gz, "Manuel Hedef");
      } else {
        Serial.println(F("Hatali format! Kullanim: GO <X> <Y> <Z> (Ornek: GO 180 0 150)"));
      }
    }
  }
}
