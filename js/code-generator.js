/**
 * Code Generator for Arduino C/C++ and MATLAB
 * Generates ready-to-run physical robot microcontroller code (.ino)
 * and MATLAB scientific simulation scripts (.m) synced with the web simulator.
 */

class CodeGenerator {
  constructor(app) {
    this.app = app;
    this.currentCodeType = 'arduino'; // 'arduino' or 'matlab'
  }

  /**
   * Generate Arduino C/C++ (.ino) code using current simulator coordinates and robot geometry
   */
  generateArduinoCode(poseA, poseB, dh, dof = 6) {
    const d1 = (dh && dh[0] ? dh[0].d : 350.0).toFixed(1);
    const a2 = (dh && dh[1] ? dh[1].a : 400.0).toFixed(1);
    const d4 = (dh && dh[3] ? dh[3].d : 380.0).toFixed(1);
    const d6 = (dh && dh[5] ? dh[5].d : 150.0).toFixed(1);

    const ax = poseA.x.toFixed(1);
    const ay = poseA.y.toFixed(1);
    const az = poseA.z.toFixed(1);
    const apitch = poseA.pitchDeg.toFixed(1);
    const ayaw = poseA.yawDeg.toFixed(1);
    const aroll = poseA.rollDeg.toFixed(1);

    const bx = poseB.x.toFixed(1);
    const by = poseB.y.toFixed(1);
    const bz = poseB.z.toFixed(1);

    return `/**
 * ======================================================================================
 * ${dof}-DOF ROBOT KOLU ARDUINO C/C++ KONTROL YAZILIMI
 * 6-Axis Robot Arm Simulator Web Arayuzunden Otomatik Uretilmistir.
 * ======================================================================================
 * Hareket Plani:
 * 1. Robot once kalibrasyon merkezi olan HOME (0.0° Acilari) pozisyonuna gider.
 * 2. Simulatorde tanimlanan A Noktasina (X:${ax}, Y:${ay}, Z:${az}) puruzsuz hareket eder.
 * 3. Bekleme suresinden sonra B Noktasina (X:${bx}, Y:${by}, Z:${bz}) gecer.
 * 4. Otomatik dongude A ve B arasinda stabil konumlandirma yapar.
 * ======================================================================================
 */

#include <Arduino.h>
#include <math.h>

// --------------------------------------------------------------------------------------
// 1. DONANIM AYARLARI
// --------------------------------------------------------------------------------------
#define USE_PCA9685 0 // 0: Standart Arduino PWM Pinleri (Servo.h), 1: PCA9685 I2C Modulu
#define ACTIVE_DOF  ${dof} // Aktif Serbestlik Derecesi (DOF)

#if USE_PCA9685
  #include <Wire.h>
  #include <Adafruit_PWMServoDriver.h>
  Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
  #define SERVOMIN  150
  #define SERVOMAX  600
#else
  #include <Servo.h>
  const int SERVO_PINS[6] = {3, 5, 6, 9, 10, 11}; // Eklem 1...6 PWM Pinleri
  Servo servos[6];
#endif

// --------------------------------------------------------------------------------------
// 2. ROBOT GEOMETRISI (DH Parametreleri - mm Cinsinden)
// --------------------------------------------------------------------------------------
const float LINK_D1 = ${d1}; // Taban yuksekligi (Link 1) [mm]
const float LINK_A2 = ${a2}; // Omuz kol uzunlugu (Link 2)  [mm]
const float LINK_D4 = ${d4}; // Dirsek on kol uzunlugu (Link 4) [mm]
const float LINK_D6 = ${d6}; // Flanş / TCP mesafesi (Link 6)   [mm]

// --------------------------------------------------------------------------------------
// 3. SIMULATORDEN AKTARILAN HEDEF KOORDINATLAR (mm Cinsinden)
// --------------------------------------------------------------------------------------
// A NOKTASI (Baslangic Pozisyonu)
const float POINT_A_X = ${ax};
const float POINT_A_Y = ${ay};
const float POINT_A_Z = ${az};

// B NOKTASI (Bitis Pozisyonu)
const float POINT_B_X = ${bx};
const float POINT_B_Y = ${by};
const float POINT_B_Z = ${bz};

// Hedef Takim Oryantasyonu (Derece)
const float TOOL_ROLL  = ${aroll};
const float TOOL_PITCH = ${apitch};
const float TOOL_YAW   = ${ayaw};

// Hareket Parametreleri
const int STEP_DELAY_MS = 15;   // Her 1 derecelik adimdaki gecikme (ms) - Sarsintiyi onler
const int DWELL_TIME_MS = 1500; // Noktalarda bekleme suresi (ms)

// Servo Kalibrasyon Merkezleri (Fiziksel 90° = Kinematik 0.0°)
const float SERVO_CENTER[6] = {90.0, 90.0, 90.0, 90.0, 90.0, 90.0};
const int   SERVO_DIR[6]    = {1, 1, 1, 1, 1, 1}; // 1 veya -1 (Yön tersleme)

float currentJointsDeg[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// Fonksiyon Prototipleri
bool solveInverseKinematics(float x, float y, float z, float rollDeg, float pitchDeg, float yawDeg, float qDegOut[6]);
void writeServoAngle(int jointIndex, float physicalAngle);
void moveToJointAnglesSmooth(const float targetAngles[6]);
void moveToCartesian(float x, float y, float z, const char* name);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 1500);

  Serial.println(F("\\n=== ${dof}-DOF ROBOT KOLU ARDUINO KONTROLCUSU BASLATILDI ==="));

#if USE_PCA9685
  pwm.begin();
  pwm.setPWMFreq(50);
#else
  for (int i = 0; i < ACTIVE_DOF; i++) {
    servos[i].attach(SERVO_PINS[i]);
  }
#endif

  // 1. Ilk acilista robotu HOME (0° kinematik) pozisyonuna gonder
  Serial.println(F("[1/3] Robot HOME (Sifir) Pozisyonuna Gonderiliyor..."));
  for (int i = 0; i < 6; i++) {
    currentJointsDeg[i] = 0.0;
    writeServoAngle(i, SERVO_CENTER[i]);
  }
  delay(2000);
  Serial.println(F("✓ Robot HOME pozisyonunda hazir!"));
}

void loop() {
  // 2. A Noktasina Git
  Serial.println(F(">>> [A NOKTASINA HAREKET]"));
  moveToCartesian(POINT_A_X, POINT_A_Y, POINT_A_Z, "A Noktasi");
  delay(DWELL_TIME_MS);

  // 3. B Noktasina Git
  Serial.println(F(">>> [B NOKTASINA HAREKET]"));
  moveToCartesian(POINT_B_X, POINT_B_Y, POINT_B_Z, "B Noktasi");
  delay(DWELL_TIME_MS);

  // 4. Tekrar A Noktasina Don
  Serial.println(F(">>> [A NOKTASINA GERI DONUS]"));
  moveToCartesian(POINT_A_X, POINT_A_Y, POINT_A_Z, "A Noktasi (Geri)");
  delay(DWELL_TIME_MS);
}

// --------------------------------------------------------------------------------------
// TERS KINEMATIK (Analytical Pieper Closed-Form IK)
// --------------------------------------------------------------------------------------
bool solveInverseKinematics(float x, float y, float z, float rollDeg, float pitchDeg, float yawDeg, float qDegOut[6]) {
  float rRad = rollDeg  * DEG_TO_RAD;
  float pRad = pitchDeg * DEG_TO_RAD;
  float yRad = yawDeg   * DEG_TO_RAD;

  // Takim Z yaklasim vektoru
  float zToolX = cos(yRad) * sin(pRad) * cos(rRad) + sin(yRad) * sin(rRad);
  float zToolY = sin(yRad) * sin(pRad) * cos(rRad) - cos(yRad) * sin(rRad);
  float zToolZ = cos(pRad) * cos(rRad);

  // Bilek Merkezi
  float xw = x - LINK_D6 * zToolX;
  float yw = y - LINK_D6 * zToolY;
  float zw = z - LINK_D6 * zToolZ;

  float r_planar = sqrt(xw * xw + yw * yw);
  float s_height = zw - LINK_D1;
  float D2 = r_planar * r_planar + s_height * s_height;
  float D  = sqrt(D2);

  float maxReach = LINK_A2 + LINK_D4;
  float minReach = fabs(LINK_A2 - LINK_D4);
  if (D > maxReach || D < minReach) {
    Serial.print(F("HATA: Hedef erisim disi! D: ")); Serial.println(D);
    return false;
  }

  // q1 (Taban)
  float q1 = atan2(yw, xw);

  // q3 (Dirsek)
  float cosGamma = (D2 - LINK_A2 * LINK_A2 - LINK_D4 * LINK_D4) / (2.0 * LINK_A2 * LINK_D4);
  cosGamma = constrain(cosGamma, -1.0, 1.0);
  float q3 = atan2(cosGamma, sqrt(1.0 - cosGamma * cosGamma));

  // q2 (Omuz)
  float Xp = LINK_A2 + LINK_D4 * sin(q3);
  float Yp = -LINK_D4 * cos(q3);
  float q2 = atan2(s_height, r_planar) - atan2(Yp, Xp);

  // Bilek Acilari
  float q4 = 0.0;
  float q5 = (pitchDeg * DEG_TO_RAD) - (q2 + q3);
  float q6 = (yawDeg * DEG_TO_RAD) - q1;

  qDegOut[0] = q1 * RAD_TO_DEG;
  qDegOut[1] = q2 * RAD_TO_DEG;
  qDegOut[2] = q3 * RAD_TO_DEG;
  qDegOut[3] = q4 * RAD_TO_DEG;
  qDegOut[4] = q5 * RAD_TO_DEG;
  qDegOut[5] = q6 * RAD_TO_DEG;
  return true;
}

// --------------------------------------------------------------------------------------
// PURUZSUZ MOTOR HAREKETI (Anti-Jerk S-Curve)
// --------------------------------------------------------------------------------------
void moveToJointAnglesSmooth(const float targetAngles[6]) {
  float maxDiff = 0.0;
  for (int i = 0; i < ACTIVE_DOF; i++) {
    float diff = fabs(targetAngles[i] - currentJointsDeg[i]);
    if (diff > maxDiff) maxDiff = diff;
  }

  int steps = max(10, (int)(maxDiff * 1.5));
  for (int s = 1; s <= steps; s++) {
    float t = (float)s / (float)steps;
    float factor = 0.5 * (1.0 - cos(t * PI)); // Kosinus yumusatmasi

    for (int i = 0; i < ACTIVE_DOF; i++) {
      float qInter = currentJointsDeg[i] + factor * (targetAngles[i] - currentJointsDeg[i]);
      float phys = SERVO_CENTER[i] + (qInter * SERVO_DIR[i]);
      phys = constrain(phys, 0.0, 180.0);
      writeServoAngle(i, phys);
    }
    delay(STEP_DELAY_MS);
  }

  for (int i = 0; i < ACTIVE_DOF; i++) {
    currentJointsDeg[i] = targetAngles[i];
  }
}

void writeServoAngle(int jointIndex, float physicalAngle) {
  if (jointIndex < 0 || jointIndex >= 6) return;
#if USE_PCA9685
  int pulse = map((int)physicalAngle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(jointIndex, 0, pulse);
#else
  servos[jointIndex].write((int)physicalAngle);
#endif
}

void moveToCartesian(float x, float y, float z, const char* name) {
  Serial.print(name);
  Serial.print(F(" -> X:")); Serial.print(x);
  Serial.print(F(" Y:"));     Serial.print(y);
  Serial.print(F(" Z:"));     Serial.println(z);

  float solved[6];
  if (solveInverseKinematics(x, y, z, TOOL_ROLL, TOOL_PITCH, TOOL_YAW, solved)) {
    moveToJointAnglesSmooth(solved);
    Serial.println(F("  ✓ Hedefe basariyla ulasildi."));
  }
}
`;
  }

  /**
   * Generate MATLAB (.m) script using current simulator coordinates and robot geometry
   */
  generateMatlabCode(poseA, poseB, dh, dof = 6) {
    // Convert mm to meters for scientific standard
    const d1_m = ((dh && dh[0] ? dh[0].d : 350.0) / 1000.0).toFixed(3);
    const a2_m = ((dh && dh[1] ? dh[1].a : 400.0) / 1000.0).toFixed(3);
    const d4_m = ((dh && dh[3] ? dh[3].d : 380.0) / 1000.0).toFixed(3);
    const d6_m = ((dh && dh[5] ? dh[5].d : 150.0) / 1000.0).toFixed(3);

    const ax_m = (poseA.x / 1000.0).toFixed(3);
    const ay_m = (poseA.y / 1000.0).toFixed(3);
    const az_m = (poseA.z / 1000.0).toFixed(3);

    const bx_m = (poseB.x / 1000.0).toFixed(3);
    const by_m = (poseB.y / 1000.0).toFixed(3);
    const bz_m = (poseB.z / 1000.0).toFixed(3);

    return `%% =========================================================================
% ${dof}-DOF ROBOT KOLU KINEMATIK SIMULASYONU VE HAREKET PLANI
% 6-Axis Robot Arm Simulator Web Arayuzunden Otomatik Uretilmistir.
% =========================================================================
% Senaryo:
% 1. Robot ilk once tum eklem acilari 0° olan HOME (Sifir) pozisyonuna gider.
% 2. Simulatorde belirlenen A Noktasina (X:${ax_m}m, Y:${ay_m}m, Z:${az_m}m) hareket eder.
% 3. A Noktasindan B Noktasina (X:${bx_m}m, Y:${by_m}m, Z:${bz_m}m) puruzsuz gecer.
% 4. 3 Boyutlu robot kolu animasyonu, TCP yorunge izi ve eklem grafikleri cizilir.
% =========================================================================

clear; clc; close all;

fprintf('====================================================\\n');
fprintf('   ${dof}-DOF ROBOT KOLU MATLAB SIMULATORU BASLATILIYOR   \\n');
fprintf('====================================================\\n\\n');

%% 1. ROBOT GEOMETRISI VE DENAVIT-HARTENBERG (DH) PARAMETRELERI (Metre)
params.d1 = ${d1_m};   % Taban yukseltmesi (Link 1) [m]
params.a2 = ${a2_m};   % Omuz kol uzunlugu (Link 2)  [m]
params.d4 = ${d4_m};   % On kol uzunlugu   (Link 4)  [m]
params.d6 = ${d6_m};   % Flanş TCP mesafesi (Link 6)  [m]
params.dof = ${dof};

%% 2. SIMULATORDEN AKTARILAN HEDEF KOORDINATLAR (Metre)
P_A = [${ax_m}; ${ay_m}; ${az_m}]; % A Noktasi
P_B = [${bx_m}; ${by_m}; ${bz_m}]; % B Noktasi

% Hedef Takim Oryantasyonu (Z ekseni asagiya bakan alet)
R_hedef = [ 1,  0,  0;
            0, -1,  0;
            0,  0, -1 ];

%% 3. TERS KINEMATIK (IK) HESAPLAMA
q_home = zeros(6, 1); % HOME Pozisyonu: [0, 0, 0, 0, 0, 0]^T

[q_A, ok_A] = ters_kinematik(P_A, R_hedef, params, q_home);
if ~ok_A, error('HATA: A Noktasi robot calisma zarfi disindadir!'); end

[q_B, ok_B] = ters_kinematik(P_B, R_hedef, params, q_A);
if ~ok_B, error('HATA: B Noktasi robot calisma zarfi disindadir!'); end

fprintf('Hesaplanan Eklem Acilari (Derece):\\n');
fprintf('  HOME : [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]°\\n', rad2deg(q_home));
fprintf('  A    : [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]°\\n', rad2deg(q_A));
fprintf('  B    : [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]°\\n\\n', rad2deg(q_B));

%% 4. YORUNGE PLANLAMA (HOME -> A -> B)
N_A = 60;  % HOME -> A adim sayisi
N_B = 80;  % A -> B adim sayisi

q_traj1 = yorunge_olustur(q_home, q_A, N_A);
q_bekle = repmat(q_A, 1, 15);
q_traj2 = yorunge_olustur(q_A, q_B, N_B);

q_tum = [q_traj1, q_bekle, q_traj2];
toplam_adim = size(q_tum, 2);

%% 5. 3 BOYUTLU ROBOT ANİMASYONU
fig = figure('Name', '${dof}-DOF Robot Simulasyonu', 'Color', [0.07 0.09 0.14], 'Position', [100 100 1100 650]);

% Sol panel: 3D Robot
ax3d = subplot(1, 3, [1 2]);
hold(ax3d, 'on'); grid(ax3d, 'on');
set(ax3d, 'Color', [0.05 0.07 0.11], 'GridColor', [0.2 0.3 0.4]);
view(ax3d, 125, 22); axis(ax3d, 'equal');
xlim(ax3d, [-0.6 0.8]); ylim(ax3d, [-0.6 0.8]); zlim(ax3d, [-0.1 0.9]);
xlabel(ax3d, 'X (m)', 'Color', 'w'); ylabel(ax3d, 'Y (m)', 'Color', 'w'); zlabel(ax3d, 'Z (m)', 'Color', 'w');
title(ax3d, '${dof}-DOF Robot: HOME \\rightarrow A \\rightarrow B Hareketi', 'Color', [0.2 0.8 1.0], 'FontSize', 12);

% Hedef Noktalar
scatter3(ax3d, P_A(1), P_A(2), P_A(3), 120, [0.0 0.9 0.4], 'filled');
text(ax3d, P_A(1)+0.03, P_A(2)+0.03, P_A(3)+0.03, 'A Noktasi', 'Color', [0.0 0.9 0.4], 'FontWeight', 'bold');
scatter3(ax3d, P_B(1), P_B(2), P_B(3), 120, [1.0 0.4 0.2], 'filled');
text(ax3d, P_B(1)+0.03, P_B(2)+0.03, P_B(3)+0.03, 'B Noktasi', 'Color', [1.0 0.4 0.2], 'FontWeight', 'bold');
plot3(ax3d, [P_A(1) P_B(1)], [P_A(2) P_B(2)], [P_A(3) P_B(3)], 'w--', 'LineWidth', 1.2);

num_pts = ${dof} + 1;
h_arm = plot3(ax3d, zeros(1, num_pts), zeros(1, num_pts), zeros(1, num_pts), '-o', 'LineWidth', 4, ...
              'Color', [0.2 0.7 1.0], 'MarkerSize', 8, 'MarkerFaceColor', [1.0 0.8 0.2]);
h_trail = plot3(ax3d, nan, nan, nan, 'Color', [0.0 1.0 0.8], 'LineWidth', 2);
tcp_trail = zeros(3, toplam_adim);

% Sag panel: Eklem Grafikleri
ax_graph = subplot(1, 3, 3);
hold(ax_graph, 'on'); grid(ax_graph, 'on');
set(ax_graph, 'Color', [0.05 0.07 0.11], 'GridColor', [0.2 0.3 0.4]);
colors = lines(${dof});
q_deg = rad2deg(q_tum);
for j = 1:${dof}
    plot(ax_graph, 1:toplam_adim, q_deg(j,:), 'LineWidth', 1.5, 'Color', colors(j,:));
end
h_cur = xline(ax_graph, 1, 'r-', 'LineWidth', 1.5);
xlabel(ax_graph, 'Adim', 'Color', 'w'); ylabel(ax_graph, 'Aci (°)', 'Color', 'w');
title(ax_graph, 'Eklem Acilari (q1...q${dof})', 'Color', 'w');

%% 6. ANİMASYON DÖNGÜSÜ
for i = 1:toplam_adim
    q_now = q_tum(:, i);
    P_links = ileri_kinematik(q_now, params);
    
    set(h_arm, 'XData', P_links(1,:), 'YData', P_links(2,:), 'ZData', P_links(3,:));
    tcp_trail(:, i) = P_links(:, end);
    set(h_trail, 'XData', tcp_trail(1,1:i), 'YData', tcp_trail(2,1:i), 'ZData', tcp_trail(3,1:i));
    set(h_cur, 'Value', i);
    drawnow;
    pause(0.025);
end

fprintf('=== SIMULASYON TAMAMLANDI ===\\n');

%% --- YARDIMCI FONKSIYONLAR ---
function P_links = ileri_kinematik(q, params)
    A1 = dh_mat(q(1), params.d1, 0, pi/2);
    A2 = dh_mat(q(2), 0, params.a2, 0);
    A3 = dh_mat(q(3), 0, 0, pi/2);
    A4 = dh_mat(q(4), params.d4, 0, -pi/2);
    A5 = dh_mat(q(5), 0, 0, pi/2);
    A6 = dh_mat(q(6), params.d6, 0, 0);
    
    T01 = A1; T02 = T01*A2; T03 = T02*A3;
    T04 = T03*A4; T05 = T04*A5; T06 = T05*A6;
    
    if params.dof == 3
        P_links = [[0;0;0], T01(1:3,4), T02(1:3,4), T04(1:3,4)];
    elseif params.dof == 4
        pitch = q(2) + q(3) + q(4);
        tcp_pos = T04(1:3,4) + [params.d6 * cos(pitch)*cos(q(1)); params.d6 * cos(pitch)*sin(q(1)); params.d6 * sin(pitch)];
        P_links = [[0;0;0], T01(1:3,4), T02(1:3,4), T04(1:3,4), tcp_pos];
    elseif params.dof == 5
        pitch = q(2) + q(3) + q(4);
        tcp_pos = T04(1:3,4) + [params.d6 * cos(pitch)*cos(q(1)); params.d6 * cos(pitch)*sin(q(1)); params.d6 * sin(pitch)];
        P_links = [[0;0;0], T01(1:3,4), T02(1:3,4), T04(1:3,4), 0.5*(T04(1:3,4)+tcp_pos), tcp_pos];
    else
        P_links = [[0;0;0], T01(1:3,4), T02(1:3,4), T03(1:3,4), T04(1:3,4), T05(1:3,4), T06(1:3,4)];
    end
end

function [q, ok] = ters_kinematik(P, R, params, q_ref)
    dof = params.dof;
    if dof == 3
        Pw = P;
    elseif dof == 4 || dof == 5
        pitch = -pi/2;
        r_target = sqrt(P(1)^2 + P(2)^2);
        q1_temp = atan2(P(2), P(1));
        r_w = r_target - params.d6 * cos(pitch);
        zw = P(3) - params.d6 * sin(pitch);
        Pw = [r_w * cos(q1_temp); r_w * sin(q1_temp); zw];
    else
        z_tool = R(:, 3);
        Pw = P - params.d6 * z_tool;
    end

    r = sqrt(Pw(1)^2 + Pw(2)^2);
    s = Pw(3) - params.d1;
    D2 = r^2 + s^2;
    D = sqrt(D2);

    if (D > params.a2 + params.d4 + 1e-4) || (D < abs(params.a2 - params.d4) - 1e-4)
        q = q_ref; ok = false; return;
    end
    ok = true;

    th1 = atan2(Pw(2), Pw(1));
    K = max(-1.0, min(1.0, (D2 - params.a2^2 - params.d4^2) / (2 * params.a2 * params.d4)));
    th3 = atan2(K, sqrt(max(0, 1 - K^2)));
    th2 = atan2(s, r) - atan2(-params.d4*cos(th3), params.a2 + params.d4*sin(th3));

    if dof == 3
        q = [th1; th2; th3; 0; 0; 0];
    elseif dof == 4
        pitch = -pi/2;
        th4 = pitch - (th2 + th3);
        q = [th1; th2; th3; th4; 0; 0];
    elseif dof == 5
        pitch = -pi/2;
        th4 = pitch - (th2 + th3);
        q = [th1; th2; th3; th4; 0; 0];
    else
        A1 = dh_mat(th1, params.d1, 0, pi/2);
        A2 = dh_mat(th2, 0, params.a2, 0);
        A3 = dh_mat(th3, 0, 0, pi/2);
        T03 = A1 * A2 * A3;
        R36 = T03(1:3, 1:3)' * R;

        cosTh5 = max(-1.0, min(1.0, R36(3, 3)));
        sinTh5 = sqrt(R36(1, 3)^2 + R36(2, 3)^2);
        th5 = atan2(sinTh5, cosTh5);

        if sinTh5 > 1e-4
            th4 = atan2(R36(2, 3), R36(1, 3));
            th6 = atan2(R36(3, 2), -R36(3, 1));
        else
            th4 = 0; th6 = atan2(-R36(1, 2), R36(1, 1));
        end
        q = [th1; th2; th3; th4; th5; th6];
    end
end

function A = dh_mat(theta, d, a, alpha)
    ct = cos(theta); st = sin(theta);
    ca = cos(alpha); sa = sin(alpha);
    A = [ ct, -st*ca,  st*sa, a*ct;
          st,  ct*ca, -ct*sa, a*st;
           0,     sa,     ca,    d;
           0,      0,      0,    1 ];
end

function q_traj = yorunge_olustur(q0, q1, N)
    q_traj = zeros(6, N);
    for k = 1:N
        s = 0.5 * (1 - cos(pi * (k - 1) / (N - 1)));
        q_traj(:, k) = q0 + s * (q1 - q0);
    end
end
`;
  }

  /**
   * Initialize UI and event listeners
   */
  init() {
    // Expose this instance globally for bulletproof inline click triggers
    window.codeGenerator = this;

    // Global helper fallbacks
    window.codeGenSwitchType = (type) => this.switchType(type);
    window.codeGenCopy = () => this.copyActiveCode();
    window.codeGenDownload = () => this.downloadActiveCode();
    window.codeGenRefresh = () => this.refreshCode();

    this.updateCodeDisplay();

    // Event listener attachments (redundant to inline handlers for 100% reliability)
    const btnArduino = document.getElementById('code-toggle-arduino');
    const btnMatlab = document.getElementById('code-toggle-matlab');
    const btnCopy = document.getElementById('btn-copy-code');
    const btnDownload = document.getElementById('btn-download-code');
    const btnRefresh = document.getElementById('btn-refresh-code');

    if (btnArduino) {
      btnArduino.addEventListener('click', (e) => {
        if (e) e.preventDefault();
        this.switchType('arduino');
      });
    }

    if (btnMatlab) {
      btnMatlab.addEventListener('click', (e) => {
        if (e) e.preventDefault();
        this.switchType('matlab');
      });
    }

    if (btnCopy) {
      btnCopy.addEventListener('click', (e) => {
        if (e) e.preventDefault();
        this.copyActiveCode();
      });
    }

    if (btnDownload) {
      btnDownload.addEventListener('click', (e) => {
        if (e) e.preventDefault();
        this.downloadActiveCode();
      });
    }

    if (btnRefresh) {
      btnRefresh.addEventListener('click', (e) => {
        if (e) e.preventDefault();
        this.refreshCode();
      });
    }
  }

  /**
   * Switch between Arduino (.ino) and MATLAB (.m)
   */
  switchType(type) {
    this.currentCodeType = type;
    const btnArduino = document.getElementById('code-toggle-arduino');
    const btnMatlab = document.getElementById('code-toggle-matlab');

    if (type === 'arduino') {
      if (btnArduino) btnArduino.classList.add('active');
      if (btnMatlab) btnMatlab.classList.remove('active');
    } else {
      if (btnMatlab) btnMatlab.classList.add('active');
      if (btnArduino) btnArduino.classList.remove('active');
    }

    this.updateCodeDisplay();

    // Scroll code box to top so student immediately sees the new file header
    const codeBlock = document.getElementById('code-display-block');
    if (codeBlock) codeBlock.scrollTop = 0;

    this.showToast(type === 'arduino' ? '🤖 Arduino C/C++ Kodu Aktif' : '📊 MATLAB Betiği Aktif');
  }

  /**
   * Refresh code with live values from simulator inputs
   */
  refreshCode() {
    if (this.app && typeof this.app.syncPosesFromInputs === 'function') {
      this.app.syncPosesFromInputs();
    }
    this.updateCodeDisplay();

    const btnRefresh = document.getElementById('btn-refresh-code');
    if (btnRefresh) {
      const origHtml = btnRefresh.innerHTML;
      btnRefresh.innerHTML = '✓ Güncellendi!';
      btnRefresh.classList.add('btn-emerald');
      setTimeout(() => {
        btnRefresh.innerHTML = origHtml;
        btnRefresh.classList.remove('btn-emerald');
      }, 1500);
    }
    this.showToast('⚡ Kod koordinatları simülatörle eşitlendi!');
  }

  /**
   * Universal Clipboard Copy with Fallback
   */
  async copyActiveCode() {
    const codeBlock = document.getElementById('code-display-block');
    if (!codeBlock) return;
    const codeText = codeBlock.textContent || '';

    const btnCopy = document.getElementById('btn-copy-code');
    const origHtml = btnCopy ? btnCopy.innerHTML : '';

    let success = false;
    // Method 1: Modern navigator.clipboard API
    if (navigator && navigator.clipboard && window.isSecureContext) {
      try {
        await navigator.clipboard.writeText(codeText);
        success = true;
      } catch (err) {
        console.warn('Clipboard writeText failed, using fallback:', err);
      }
    }

    // Method 2: Invisible textarea fallback
    if (!success) {
      try {
        const textArea = document.createElement('textarea');
        textArea.value = codeText;
        textArea.style.position = 'fixed';
        textArea.style.left = '-999999px';
        textArea.style.top = '-999999px';
        textArea.setAttribute('readonly', '');
        document.body.appendChild(textArea);
        textArea.focus();
        textArea.select();
        success = document.execCommand('copy');
        document.body.removeChild(textArea);
      } catch (err) {
        console.error('execCommand copy error:', err);
      }
    }

    if (btnCopy) {
      btnCopy.innerHTML = success ? '✓ Kod Kopyalandı!' : '⚠️ Kopyalanamadı';
      btnCopy.style.background = success ? '#10b981' : '#ef4444';
      setTimeout(() => {
        btnCopy.innerHTML = origHtml;
        btnCopy.style.background = '';
      }, 2000);
    }

    this.showToast(success ? '📋 Kod panoya başarıyla kopyalandı!' : '⚠️ Lütfen kodu el ile seçip kopyalayın.');
  }

  /**
   * Reliable File Download (.ino or .m)
   */
  downloadActiveCode() {
    const codeBlock = document.getElementById('code-display-block');
    if (!codeBlock) return;
    const content = codeBlock.textContent || '';

    const isArduino = (this.currentCodeType === 'arduino');
    const filename = isArduino ? 'robot_arm_controller.ino' : 'robot_arm_simulator.m';
    const mime = isArduino ? 'text/x-csrc;charset=utf-8' : 'text/plain;charset=utf-8';

    let downloadStarted = false;

    // Method 1: Blob URL with deferred revocation
    try {
      const blob = new Blob([content], { type: mime });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = filename;
      a.style.display = 'none';
      document.body.appendChild(a);
      a.click();
      downloadStarted = true;

      // Keep blob URL active for 3 seconds so OS download manager completes
      setTimeout(() => {
        if (a.parentNode) document.body.removeChild(a);
        URL.revokeObjectURL(url);
      }, 3000);
    } catch (e) {
      console.warn('Blob download failed, trying data URI:', e);
    }

    // Method 2: Data URI Fallback
    if (!downloadStarted) {
      try {
        const a = document.createElement('a');
        a.href = 'data:' + mime + ',' + encodeURIComponent(content);
        a.download = filename;
        a.style.display = 'none';
        document.body.appendChild(a);
        a.click();
        setTimeout(() => {
          if (a.parentNode) document.body.removeChild(a);
        }, 1000);
        downloadStarted = true;
      } catch (e2) {
        console.error('Data URI download failed:', e2);
      }
    }

    const btnDownload = document.getElementById('btn-download-code');
    if (btnDownload) {
      const origHtml = btnDownload.innerHTML;
      btnDownload.innerHTML = downloadStarted ? '✓ Dosya İndirildi!' : '⚠️ İndirilemedi';
      btnDownload.style.background = downloadStarted ? '#10b981' : '#ef4444';
      setTimeout(() => {
        btnDownload.innerHTML = origHtml;
        btnDownload.style.background = '';
      }, 2000);
    }

    this.showToast(downloadStarted ? `💾 ${filename} dosyası indirildi!` : '⚠️ Dosya indirilemedi.');
  }

  /**
   * Update the code pre element with freshly generated source
   */
  updateCodeDisplay() {
    const codeBlock = document.getElementById('code-display-block');
    const filenameLabel = document.getElementById('code-filename-label');
    if (!codeBlock) return;

    // Read current poses
    const poseA = (this.app && this.app.poseA) ? this.app.poseA : { x: 450, y: -250, z: 220, rollDeg: 0, pitchDeg: 90, yawDeg: 0 };
    const poseB = (this.app && this.app.poseB) ? this.app.poseB : { x: 420, y: 300, z: 380, rollDeg: 0, pitchDeg: 60, yawDeg: 45 };
    const dh = (this.app && this.app.kinematics) ? this.app.kinematics.dh : null;
    const dof = (this.app && this.app.kinematics) ? (this.app.kinematics.currentDOF || 6) : 6;

    let code = '';
    if (this.currentCodeType === 'arduino') {
      code = this.generateArduinoCode(poseA, poseB, dh, dof);
      if (filenameLabel) filenameLabel.textContent = `robot_arm_controller.ino (${dof}-DOF Arduino Sketch)`;
    } else {
      code = this.generateMatlabCode(poseA, poseB, dh, dof);
      if (filenameLabel) filenameLabel.textContent = `robot_arm_simulator.m (${dof}-DOF MATLAB Script)`;
    }

    codeBlock.textContent = code;
  }

  /**
   * Show notification toast inside the code generator section
   */
  showToast(message) {
    const toast = document.getElementById('code-status-toast');
    if (!toast || !toast.style) return;
    toast.textContent = message;
    toast.style.display = 'block';
    toast.style.opacity = '1';
    clearTimeout(this.toastTimer);
    this.toastTimer = setTimeout(() => {
      if (toast && toast.style) {
        toast.style.opacity = '0';
        setTimeout(() => { toast.style.display = 'none'; }, 300);
      }
    }, 2500);
  }
}

// Global export
if (typeof window !== 'undefined') {
  window.CodeGenerator = CodeGenerator;
}
if (typeof module !== 'undefined' && module.exports) {
  module.exports = CodeGenerator;
}

