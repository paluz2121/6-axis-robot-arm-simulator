%% =========================================================================
% 3-6 DOF ENDÜSTRİYEL ROBOT KOLU KİNEMATİK SİMÜLATÖRÜ VE YÖRÜNGE PLANLAYICI
% Üniversite Robotik ve Mekatronik Eğitimi İçin Özel Tasarlanmıştır.
% =========================================================================
% Senaryo:
% 1. Robot önce tüm eklem açıları 0° olan "HOME (Sıfır)" pozisyonuna gider.
% 2. Öğrencinin tanımladığı A Noktasına (X_A, Y_A, Z_A) pürüzsüz hareket eder.
% 3. A Noktasından B Noktasına (X_B, Y_B, Z_B) doğru hareket eder.
% 4. 3 Boyutlu robot kolu animasyonu, TCP yörünge izi ve eklem grafikleri çizilir.
% =========================================================================

clear; clc; close all;

%% 0. ROBOT KONFİGÜRASYONU (DOF SEÇİMİ)
% 3, 4, 5 veya 6 Serbestlik Derecesi seçilebilir (Varsayılan: 6)
dof = 6; 

fprintf('====================================================\n');
fprintf('   %d-DOF ROBOT KOLU MATLAB SİMÜLATÖRÜ BAŞLATILIYOR  \n', dof);
fprintf('====================================================\n\n');

%% 1. ROBOT GEOMETRİSİ VE DENAVIT-HARTENBERG (DH) PARAMETRELERİ
% Gerçek robot kolu ölçüleri (Metre cinsinden)
d1 = 0.350;   % Taban yükseltmesi (Link 1 offset)      [m] (350 mm)
a2 = 0.400;   % Omuz-Dirsek arası kol uzunluğu (Link 2) [m] (400 mm)
d4 = 0.380;   % Dirsek-Bilek arası önkol uzunluğu      [m] (380 mm)
d6 = 0.150;   % Bilek merkezi - Takım Flanşı (TCP)     [m] (150 mm)

% Yapı parametreleri struct'ı
robotParams.d1 = d1;
robotParams.a2 = a2;
robotParams.d4 = d4;
robotParams.d6 = d6;
robotParams.dof = dof;

fprintf('Robot Bağlantı Uzunlukları:\n');
fprintf('  d1 (Taban)   = %.3f m\n', d1);
fprintf('  a2 (Omuz)    = %.3f m\n', a2);
fprintf('  d4 (Ön Kol)  = %.3f m\n', d4);
if dof >= 4
    fprintf('  d6 (Flanş)   = %.3f m\n', d6);
end
fprintf('  Aktif Eksen Sayısı (DOF) = %d\n\n', dof);

%% 2. ÖĞRENCİ TARAFINDAN BELİRLENEN HEDEF NOKTALARI (A ve B)
% Uç nokta (TCP) kartezyen koordinatları [X, Y, Z] (Metre cinsinden)

% A NOKTASI (Örnek: Parça alma / Başlangıç kaynak noktası)
P_A = [0.420; -0.250; 0.220]; 

% B NOKTASI (Örnek: Parça bırakma / Bitiş kaynak noktası)
P_B = [0.380;  0.280; 0.350];

% Hedef Takım Oryantasyonu (Aşağıya bakan takım: Roll=0, Pitch=90°, Yaw=0°)
R_hedef = [ 1,  0,  0;
            0, -1,  0;
            0,  0, -1 ];

fprintf('Öğrenci Hedef Koordinatları:\n');
fprintf('  HOME (0 Pozisyonu) = Tüm eklemler 0 rad\n');
fprintf('  A Noktası = [X: %.3f, Y: %.3f, Z: %.3f] m\n', P_A(1), P_A(2), P_A(3));
fprintf('  B Noktası = [X: %.3f, Y: %.3f, Z: %.3f] m\n\n', P_B(1), P_B(2), P_B(3));

%% 3. TERS KİNEMATİK (IK) ÇÖZÜMLERİNİN HESAPLANMASI
% 1. HOME Pozisyonu:
q_home = zeros(6, 1); % [rad]

% 2. A Noktası için Ters Kinematik:
[q_A, ulasilabilir_A] = ters_kinematik(P_A, R_hedef, robotParams, q_home);
if ~ulasilabilir_A
    error('HATA: A Noktası robotun çalışma alanı (workspace) dışındadır!');
end

% 3. B Noktası için Ters Kinematik:
[q_B, ulasilabilir_B] = ters_kinematik(P_B, R_hedef, robotParams, q_A);
if ~ulasilabilir_B
    error('HATA: B Noktası robotun çalışma alanı (workspace) dışındadır!');
end

fprintf('Hesaplanan Eklem Açıları (Derece):\n');
fprintf('  A   : ');
for j = 1:dof
    fprintf('q%d: %.1f°  ', j, rad2deg(q_A(j)));
end
fprintf('\n  B   : ');
for j = 1:dof
    fprintf('q%d: %.1f°  ', j, rad2deg(q_B(j)));
end
fprintf('\n\n');

%% 4. YÖRÜNGE PLANLAMA (HOME -> A -> B)
% Pürüzsüz hareket için kosinüs yumuşatma (S-Curve) interpolasyonu
N_adima = 60;  % HOME -> A adım sayısı
N_adimb = 80;  % A -> B adım sayısı

% 1. Bölüm: HOME -> A
q_traj1 = yorunge_olustur(q_home, q_A, N_adima);

% Bekleme süresi (A noktasında 15 adım bekleme)
q_bekleA = repmat(q_A, 1, 15);

% 2. Bölüm: A -> B
q_traj2 = yorunge_olustur(q_A, q_B, N_adimb);

% Tüm eklem yörüngesini birleştir
q_tum = [q_traj1, q_bekleA, q_traj2];
toplam_adim = size(q_tum, 2);

fprintf('Yörünge hazırlandı: Toplam %d adım.\n', toplam_adim);
fprintf('3 Boyutlu Simülasyon başlatılıyor...\n');

%% 5. 3 BOYUTLU ROBOT ANİMASYONU VE GÖRSELLEŞTİRME
fig = figure('Name', sprintf('%d-DOF Robot Kolu Simülatörü - HOME -> A -> B', dof), ...
             'Color', [0.07 0.09 0.14], 'Position', [80, 80, 1100, 650]);

% Sol panel: 3D Görünüm
ax3d = subplot(1, 3, [1 2]);
hold(ax3d, 'on'); grid(ax3d, 'on');
set(ax3d, 'Color', [0.05 0.07 0.11], 'GridColor', [0.2 0.3 0.4], 'GridAlpha', 0.4);
view(ax3d, 125, 22);
axis(ax3d, 'equal');
xlim(ax3d, [-0.6 0.8]); ylim(ax3d, [-0.6 0.8]); zlim(ax3d, [-0.1 0.9]);
xlabel(ax3d, 'X (m)', 'Color', 'w');
ylabel(ax3d, 'Y (m)', 'Color', 'w');
zlabel(ax3d, 'Z (m)', 'Color', 'w');
title(ax3d, sprintf('%d-DOF Robot Kolu: HOME \\rightarrow A \\rightarrow B Hareketi', dof), ...
      'Color', [0.2 0.8 1.0], 'FontSize', 12, 'FontWeight', 'bold');

% Zemin ızgarası
[Xg, Yg] = meshgrid(-0.6:0.2:0.8, -0.6:0.2:0.8);
mesh(ax3d, Xg, Yg, zeros(size(Xg)), 'FaceAlpha', 0, 'EdgeColor', [0.12 0.18 0.25]);

% Hedef Noktaları Çiz
scatter3(ax3d, P_A(1), P_A(2), P_A(3), 120, [0.0 0.9 0.4], 'filled');
text(ax3d, P_A(1)+0.03, P_A(2)+0.03, P_A(3)+0.03, 'A Noktası', ...
     'Color', [0.0 0.9 0.4], 'FontWeight', 'bold', 'FontSize', 10);

scatter3(ax3d, P_B(1), P_B(2), P_B(3), 120, [1.0 0.4 0.2], 'filled');
text(ax3d, P_B(1)+0.03, P_B(2)+0.03, P_B(3)+0.03, 'B Noktası', ...
     'Color', [1.0 0.4 0.2], 'FontWeight', 'bold', 'FontSize', 10);

% Hedef çizgi (A -> B)
plot3(ax3d, [P_A(1) P_B(1)], [P_A(2) P_B(2)], [P_A(3) P_B(3)], 'w--', 'LineWidth', 1.2);

% Robot kolu gövdesi çizgisi ve eklem noktaları (Tam DOF + 1 adet nokta)
num_pts = dof + 1;
h_arm = plot3(ax3d, zeros(1, num_pts), zeros(1, num_pts), zeros(1, num_pts), '-o', ...
              'LineWidth', 4, 'Color', [0.2 0.7 1.0], ...
              'MarkerSize', 8, 'MarkerFaceColor', [1.0 0.8 0.2], 'MarkerEdgeColor', 'w');

% Uç nokta iz çizgisi (TCP Trail)
h_trail = plot3(ax3d, nan, nan, nan, 'Color', [0.0 1.0 0.8], 'LineWidth', 2);
tcp_trail = zeros(3, toplam_adim);

% Durum Bilgi Metni
h_status = text(ax3d, -0.5, -0.5, 0.8, 'Başlatılıyor...', ...
                'Color', 'w', 'FontSize', 10, 'BackgroundColor', [0.1 0.15 0.25]);

% Sağ panel: Eklem Açıları Grafiği
ax_graph = subplot(1, 3, 3);
hold(ax_graph, 'on'); grid(ax_graph, 'on');
set(ax_graph, 'Color', [0.05 0.07 0.11], 'GridColor', [0.2 0.3 0.4]);
colors = lines(dof);
joint_labels = {'q1 (Taban)', 'q2 (Omuz)', 'q3 (Dirsek)', 'q4 (Bilek R)', 'q5 (Bilek P)', 'q6 (Flanş)'};

q_deg_tum = rad2deg(q_tum);
for j = 1:dof
    plot(ax_graph, 1:toplam_adim, q_deg_tum(j,:), 'LineWidth', 1.5, 'Color', colors(j,:));
end
h_cursor = xline(ax_graph, 1, 'r-', 'LineWidth', 1.5);
xlabel(ax_graph, 'Adım No', 'Color', 'w');
ylabel(ax_graph, 'Açı (°)', 'Color', 'w');
title(ax_graph, sprintf('Eklem Açıları (1...%d)', dof), 'Color', 'w');
legend(ax_graph, joint_labels(1:dof), 'TextColor', 'w', ...
       'Color', [0.08 0.12 0.18], 'Location', 'best');

%% 6. ANİMASYON DÖNGÜSÜ
pause_time = 0.025; % Saniyede ~40 kare
for i = 1:toplam_adim
    q_suan = q_tum(:, i);
    [P_eklemler, ~] = ileri_kinematik(q_suan, robotParams);
    
    % Robot kolu pozisyonunu güncelle
    set(h_arm, 'XData', P_eklemler(1,:), ...
               'YData', P_eklemler(2,:), ...
               'ZData', P_eklemler(3,:));
    
    % TCP izini güncelle
    tcp_trail(:, i) = P_eklemler(:, end);
    set(h_trail, 'XData', tcp_trail(1, 1:i), ...
                 'YData', tcp_trail(2, 1:i), ...
                 'ZData', tcp_trail(3, 1:i));
    
    % Durum yazısı
    if i <= N_adima
        durum_str = sprintf('Aşama 1: HOME \\rightarrow A Noktası (Adım: %d/%d)', i, N_adima);
    elseif i <= N_adima + 15
        durum_str = 'Aşama 2: A Noktasında Bekleme / İşlem Yapılıyor';
    else
        durum_str = sprintf('Aşama 3: A \\rightarrow B Noktası (Adım: %d/%d)', i - N_adima - 15, N_adimb);
    end
    set(h_status, 'String', durum_str);
    set(h_cursor, 'Value', i);
    
    drawnow;
    pause(pause_time);
end

fprintf('\n=== SİMÜLASYON BAŞARIYLA TAMAMLANDI ===\n');


%% =========================================================================
% YARDIMCI KİNEMATİK FONKSİYONLARI (MODÜLER VE HIZLI ÇÖZÜM)
%% =========================================================================

%% A. İLERİ KİNEMATİK FONKSİYONU (FORWARD KINEMATICS)
function [P_eklemler, T06] = ileri_kinematik(q, params)
    d1 = params.d1; a2 = params.a2; d4 = params.d4; d6 = params.d6;
    dof = params.dof;
    
    % Standart DH Homojen Dönüşüm Matrisleri:
    A1 = dh_matris(q(1), d1,   0,   pi/2);
    A2 = dh_matris(q(2),  0,  a2,      0);
    A3 = dh_matris(q(3),  0,   0,   pi/2);
    A4 = dh_matris(q(4), d4,   0,  -pi/2);
    A5 = dh_matris(q(5),  0,   0,   pi/2);
    A6 = dh_matris(q(6), d6,   0,      0);
    
    % Kümülatif Dönüşümler
    T01 = A1;
    T02 = T01 * A2;
    T03 = T02 * A3;
    T04 = T03 * A4;
    T05 = T04 * A5;
    T06 = T05 * A6;
    
    % DOF'a tam uygun eklem noktaları
    if dof == 3
        % 3-DOF: Taban, Omuz, Ön Kol Ucu (3 link, 4 nokta)
        P_eklemler = [ [0; 0; 0], ...
                       T01(1:3, 4), ...
                       T02(1:3, 4), ...
                       T04(1:3, 4) ];
    elseif dof == 4
        % 4-DOF: Taban, Omuz, Dirsek, Takım Ucu (4 link, 5 nokta)
        pitch = q(2) + q(3) + q(4);
        tcp_pos = T04(1:3, 4) + [d6 * cos(pitch) * cos(q(1)); ...
                                 d6 * cos(pitch) * sin(q(1)); ...
                                 d6 * sin(pitch)];
        P_eklemler = [ [0; 0; 0], ...
                       T01(1:3, 4), ...
                       T02(1:3, 4), ...
                       T04(1:3, 4), ...
                       tcp_pos ];
    elseif dof == 5
        % 5-DOF: Taban, Omuz, Dirsek, Bilek Eklemi, Takım Ucu (5 link, 6 nokta)
        pitch = q(2) + q(3) + q(4);
        tcp_pos = T04(1:3, 4) + [d6 * cos(pitch) * cos(q(1)); ...
                                 d6 * cos(pitch) * sin(q(1)); ...
                                 d6 * sin(pitch)];
        mid_wrist = 0.5 * (T04(1:3, 4) + tcp_pos);
        P_eklemler = [ [0; 0; 0], ...
                       T01(1:3, 4), ...
                       T02(1:3, 4), ...
                       T04(1:3, 4), ...
                       mid_wrist, ...
                       tcp_pos ];
    else
        % 6-DOF: Standart Tam Küresel Bilek (6 link, 7 nokta)
        P_eklemler = [ [0; 0; 0], ...
                       T01(1:3, 4), ...
                       T02(1:3, 4), ...
                       T03(1:3, 4), ...
                       T04(1:3, 4), ...
                       T05(1:3, 4), ...
                       T06(1:3, 4) ];
    end
end

%% B. TERS KİNEMATİK FONKSİYONU (INVERSE KINEMATICS)
function [q, ulasilabilir] = ters_kinematik(P_hedef, R_hedef, params, q_referans)
    d1 = params.d1; a2 = params.a2; d4 = params.d4; d6 = params.d6;
    dof = params.dof;
    
    if dof == 3
        % 3-DOF: Takım ucu doğrudan ön kolun bittiği yerdir (d6 ofseti yoktur)
        Pw = P_hedef;
    elseif dof == 4 || dof == 5
        % 4/5-DOF: Düzlemsel eğimli bilek yaklaşımı
        pitch = -pi/2; % Aşağı yönlü takım
        r_target = sqrt(P_hedef(1)^2 + P_hedef(2)^2);
        q1_temp = atan2(P_hedef(2), P_hedef(1));
        r_w = r_target - d6 * cos(pitch);
        zw = P_hedef(3) - d6 * sin(pitch);
        Pw = [r_w * cos(q1_temp); r_w * sin(q1_temp); zw];
    else
        % 6-DOF: Pieper Küresel Bilek Ayrıştırması
        z_tool = R_hedef(:, 3);
        Pw = P_hedef - d6 * z_tool;
    end
    
    xw = Pw(1); yw = Pw(2); zw = Pw(3);
    
    % İlk 3 Eklem (q1, q2, q3) Geometrisi
    r = sqrt(xw^2 + yw^2);
    s = zw - d1;
    D2 = r^2 + s^2;
    D = sqrt(D2);
    
    % Çalışma Zarfı Sınır Kontrolü
    max_reach = a2 + d4;
    min_reach = abs(a2 - d4);
    
    if (D > max_reach + 1e-4) || (D < min_reach - 1e-4)
        q = q_referans;
        ulasilabilir = false;
        return;
    end
    ulasilabilir = true;
    
    % Kosinüs Teoremi ile Dirsek Açısı (q3)
    K = (D2 - a2^2 - d4^2) / (2 * a2 * d4);
    K = max(-1.0, min(1.0, K));
    
    th1 = atan2(yw, xw);
    th3 = atan2(K, sqrt(max(0, 1 - K^2))); % Elbow-up konfigürasyonu
    
    % Omuz Açısı (q2)
    X_prime = a2 + d4 * sin(th3);
    Y_prime = -d4 * cos(th3);
    th2 = atan2(s, r) - atan2(Y_prime, X_prime);
    
    if dof == 3
        q = [th1; th2; th3; 0; 0; 0];
    elseif dof == 4
        pitch = -pi/2;
        th4 = pitch - (th2 + th3);
        q = [th1; th2; th3; th4; 0; 0];
    elseif dof == 5
        pitch = -pi/2;
        th4 = pitch - (th2 + th3);
        th5 = 0;
        q = [th1; th2; th3; th4; th5; 0];
    else
        % 6-DOF: Bilek Oryantasyonu (q4, q5, q6)
        A1 = dh_matris(th1, d1,  0,  pi/2);
        A2 = dh_matris(th2,  0, a2,     0);
        A3 = dh_matris(th3,  0,  0,  pi/2);
        T03 = A1 * A2 * A3;
        R03 = T03(1:3, 1:3);
        
        R36 = R03' * R_hedef;
        
        cosTh5 = max(-1.0, min(1.0, R36(3, 3)));
        sinTh5 = sqrt(R36(1, 3)^2 + R36(2, 3)^2);
        th5 = atan2(sinTh5, cosTh5);
        
        if sinTh5 > 1e-4
            th4 = atan2(R36(2, 3), R36(1, 3));
            th6 = atan2(R36(3, 2), -R36(3, 1));
        else
            th4 = 0;
            th6 = atan2(-R36(1, 2), R36(1, 1));
        end
        q = [th1; th2; th3; th4; th5; th6];
    end
end

%% C. DH DÖNÜŞÜM MATRİSİ
function A = dh_matris(theta, d, a, alpha)
    ct = cos(theta); st = sin(theta);
    ca = cos(alpha); sa = sin(alpha);
    A = [ ct, -st*ca,  st*sa, a*ct;
          st,  ct*ca, -ct*sa, a*st;
           0,     sa,     ca,    d;
           0,      0,      0,    1 ];
end

%% D. S-CURVE / KOSİNÜS İNTERPOLASYON FONKSİYONU
function q_traj = yorunge_olustur(q_baslangic, q_bitis, N)
    q_traj = zeros(6, N);
    for k = 1:N
        t = (k - 1) / (N - 1);
        s = 0.5 * (1 - cos(pi * t)); 
        q_traj(:, k) = q_baslangic + s * (q_bitis - q_baslangic);
    end
end
