# ROBOKIN 6D: 6-Eksenli Robot Kinematiği ve Yörünge Simülatörü

Bu program, üniversite ve mühendislik düzeyindeki robotik, mekatronik ve otomasyon derslerinde öğrencilerin **6-eksenli endüstriyel robot kollarının hareket hesaplarını** (İleri Kinematik, Ters Kinematik, Jakoben Analizi, Tekillikler ve Yörünge Planlama) hem görsel hem de formülsel olarak derinlemesine öğrenmesi için geliştirilmiş kapsamlı bir eğitim simülatörüdür.

---

## 🚀 Hızlı Başlangıç

Programı çalıştırmak için iki basit yöntem bulunmaktadır:

### Yöntem 1: Doğrudan Tarayıcıda Açma (Kurulum Gerektirmez)
`index.html` dosyasını Chrome, Edge, Firefox veya Safari tarayıcınızda çift tıklayarak açabilirsiniz.

### Yöntem 2: Yerel Sunucu ile Çalıştırma
Klasördeki `start_simulator.bat` dosyasına çift tıklayabilir veya terminalde şu komutu çalıştırabilirsiniz:
```bash
python -m http.server 8080
```
Ardından tarayıcınızdan `http://localhost:8080` adresine gidin.

---

## 📚 Kapsanan Eğitim Parametreleri ve Konular

Simülatör, robotik mühendisliği müfredatında yer alan tüm kritik hesaplama adımlarını içerir:

### 1. Denavit-Hartenberg (DH) Parametreleri ve Çerçeve Atamaları
- $\theta_i$: Eklem açısı (Joint Angle) - Değişken açı
- $d_i$: Bağlantı ötelemesi (Link Offset)
- $a_i$: Bağlantı uzunluğu (Link Length)
- $\alpha_i$: Burulma açısı (Link Twist)
- $A_i(\theta_i, d_i, a_i, \alpha_i)$ tekil homojen dönüşüm matrisleri ($4 \times 4$)
- $T_0^6 = A_1 A_2 A_3 A_4 A_5 A_6$ uç nokta (TCP) dönüşüm matrisi

### 2. Kapalı Form Analitik Ters Kinematik (Pieper Teoremi)
- **Küresel Bilek Ayrıştırması (Spherical Wrist Decoupling):**
  Son 3 eklemin (Eklem 4, 5, 6) eksenlerinin tek bir noktada (Bilek Merkezi $P_w$) kesiştiği özelliği kullanılarak problem konum ve yönelim olarak iki bağımsız aşamaya ayrıştırılır:
  $$P_w = P_{TCP} - d_6 \cdot R \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}$$
- **8 Olası Çözüm Konfigürasyonu:**
  1. Omuz Konfigürasyonu: **Ön (Front)** vs **Arka (Back)** ($2$ çözüm)
  2. Dirsek Konfigürasyonu: **Yukarı (Elbow Up)** vs **Aşağı (Elbow Down)** ($2$ çözüm)
  3. Bilek Konfigürasyonu: **Normal (No-Flip)** vs **Ters (Flip)** ($2$ çözüm)
  Toplam $2 \times 2 \times 2 = 8$ analitik çözüm hesaplanır, eklem limitleri kontrol edilir ve öğrenci istediği dalı seçebilir.

### 3. Diferansiyel Kinematik ve Jakoben Analizi
- **Geometrik Jakoben Matrisi ($6 \times 6$):**
  $$\begin{bmatrix} v \\ \omega \end{bmatrix} = J(q) \dot{q}, \quad J_i = \begin{bmatrix} z_{i-1} \times (p_e - p_{i-1}) \\ z_{i-1} \end{bmatrix}$$
- **Yoshikawa Manipülatiflik İndeksi ($w$):**
  $$w(q) = \sqrt{\det(J(q) J^T(q))}$$
- **3 Temel Endüstriyel Robot Tekilliği Teşhisi:**
  - **Bilek Tekilliği (Wrist Singularity):** $\theta_5 \approx 0^\circ$ olduğunda eklem 4 ve 6 eksenleri çakışır.
  - **Omuz Tekilliği (Shoulder Singularity):** Bilek merkezi ana eksen ($Z_0$) üzerine geldiğinde ($r \approx 0$).
  - **Dirsek Tekilliği (Elbow / Boundary Singularity):** Kolun tam açık veya kapalı olduğu çalışma zarfı sınırları.

### 4. A Noktasından B Noktasına Yörünge Planlama
Program 4 farklı yörünge üretim profilini karşılaştırmalı olarak sunar:
1. **5. Derece Polinom (Quintic Spline - Minimum Jerk):**
   $s(\tau) = 10\tau^3 - 15\tau^4 + 6\tau^5$ ($\tau = t/T$).
   Başlangıç ve bitişte hem hızlar hem de ivmeler sıfırdır, mekanik darbeleri engeller.
2. **3. Derece Polinom (Cubic Spline):**
   $s(\tau) = 3\tau^2 - 2\tau^3$. Hızlar sıfırlanır ancak ivmede ani geçişler gözlemlenebilir.
3. **Yamuk Hız Profili (LSPB - Trapezoidal):**
   Doğrusal hızlanma, sabit seyir hızı ve doğrusal yavaşlama fazları.
4. **Kartezyen Doğrusal Yörünge (LIN):**
   Uç nokta 3D uzayda lazer gibi düz bir doğru üzerinde hareket ederken yönelim Quaternion SLERP ile hesaplanır. Her adımda anlık Ters Kinematik çözülür.

### 5. Dinamik Grafikler (Chart.js)
Zaman boyunca ($t = 0 \to T$) gerçek zamanlı çizilen eğriler:
- Eklem Konumları $\theta_1(t) \dots \theta_6(t)$ [Derece]
- Eklem Açısal Hızları $\dot{\theta}_1(t) \dots \dot{\theta}_6(t)$ [°/s]
- Eklem Açısal İvmeleri $\ddot{\theta}_1(t) \dots \ddot{\theta}_6(t)$ [°/s²]
- Kartezyen TCP Hızı $v(t)$ [mm/s] ve Yoshikawa Manipülatiflik Eğrisi

---

## 🎯 Hazır Eğitim Senaryoları (Presets)
Açılır menüden tek tıkla seçilebilecek laboratuvar senaryoları:
1. **Standart Taşıma (Pick & Place):** Masa üzerindeki A noktasından B kutusuna aktarma.
2. **Doğrusal Kaynak Yolu (Linear Seam):** Eklem uzayı (PTP) ile Kartezyen uzay (LIN) farkını gösteren düz çizgi kaynağı.
3. **Bilek Tekilliği Örneği:** $\theta_5 \to 0$ yapılarak robotun serbestlik derecesi kaybı ve hız patlaması incelenir.
4. **Omuz Tekilliği Örneği:** Kolun gövde merkezinden geçişi.
5. **Çalışma Alanı Sınırı:** Dirsek tekilliği ve maksimum uzanma zarfı testi.

---

## 🛠️ Dosya Yapısı
- `index.html`: Ana arayüz, 3D görünüm, kontrol panelleri, matematik sekmeleri.
- `css/style.css`: Modern karanlık cam efekti (glassmorphism), siber endüstriyel tema.
- `js/robot-kinematics.js`: Kinematik motor (DH, FK, 8 çözümlü analitik IK, Jakoben, Yörünge algoritmaları).
- `js/robot-3d.js`: Three.js 3D görselleştirici (robot kolu, koordinat eksenleri, hayalet modeller, yörünge izi).
- `js/charts.js`: Chart.js ile dinamik yörünge grafikleri ve senkronize zaman imleci.
- `js/math-explainer.js`: KaTeX ile canlı matris ve adım adım formül açıklamaları.
- `js/app.js`: Arayüz yöneticisi, klavye kısayolları ve simülasyon döngüsü.
