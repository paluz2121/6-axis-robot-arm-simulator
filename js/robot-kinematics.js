/**
 * Robot Kinematics Engine (6-DOF Anthropomorphic Spherical Wrist Manipulator)
 * Comprehensive Educational Kinematics & Trajectory Engine
 * 
 * Supports:
 * - Denavit-Hartenberg (DH) Forward Kinematics (FK)
 * - Individual Transformation Matrices A1...A6 & Cumulative T0...T6
 * - Analytical Closed-Form Inverse Kinematics (Up to 8 Solutions via Pieper's Decoupling)
 * - Geometric Jacobian (6x6), Determinant, Condition Number & Manipulability (Yoshikawa)
 * - Singularity Detection (Wrist, Shoulder, Elbow boundary)
 * - Trajectory Generators: Cubic Polynomial, Quintic Polynomial, Trapezoidal (LSPB), Cartesian Straight Line (LIN)
 */

class RobotKinematics {
  constructor(dhParams = null) {
    // Default DH Parameters (Industrial 6-Axis Robot, e.g. PUMA/KUKA style, in mm and radians)
    this.dh = dhParams || [
      { id: 1, name: "Base (Kök)",         theta: 0, d: 350.0, a: 0.0,   alpha: Math.PI / 2,  minDeg: -180, maxDeg: 180 },
      { id: 2, name: "Shoulder (Omuz)",   theta: 0, d: 0.0,   a: 400.0, alpha: 0.0,          minDeg: -120, maxDeg: 120 },
      { id: 3, name: "Elbow (Dirsek)",     theta: 0, d: 0.0,   a: 0.0,   alpha: Math.PI / 2,  minDeg: -150, maxDeg: 150 },
      { id: 4, name: "Wrist 1 (Bilek R)", theta: 0, d: 380.0, a: 0.0,   alpha: -Math.PI / 2, minDeg: -180, maxDeg: 180 },
      { id: 5, name: "Wrist 2 (Bilek P)", theta: 0, d: 0.0,   a: 0.0,   alpha: Math.PI / 2,  minDeg: -120, maxDeg: 120 },
      { id: 6, name: "Wrist 3 (Flanş/T)", theta: 0, d: 150.0, a: 0.0,   alpha: 0.0,          minDeg: -360, maxDeg: 360 }
    ];

    // Current joint state in radians
    this.q = [0, 0, 0, 0, 0, 0];

    // Multi-DOF Architecture (Supports 3-DOF, 4-DOF, 5-DOF, 6-DOF)
    this.currentDOF = 6;

    // Structural & Dynamic Physical Parameters (Mass, Inertia, Cross-section Geometry)
    this.material = {
      name: "Alüminyum 6061-T6 (Havacılık/Robotik Standardı)",
      yieldStressMPa: 276.0, // Akma Mukavemeti (MPa)
      densityKgM3: 2700.0,   // Yoğunluk (kg/m3)
      elasticModulusGPa: 68.9
    };

    // Link dynamic properties: mass (kg), CoM distance along link (m), outer diameter D (mm), wall thickness t (mm)
    this.linkDynamics = [
      { id: 1, name: "Taban / Kök (Base Turret)", mass: 18.0, rCoM: 0.175, D_outer: 160.0, t_wall: 12.0, Izz: 0.085 },
      { id: 2, name: "Omuz (Shoulder / Upper Arm)", mass: 14.5, rCoM: 0.200, D_outer: 120.0, t_wall: 10.0, Izz: 0.195 },
      { id: 3, name: "Dirsek / Ön Kol (Forearm)", mass: 9.0,  rCoM: 0.190, D_outer: 95.0,  t_wall: 7.0,  Izz: 0.108 },
      { id: 4, name: "Bilek 1 (Wrist Roll)", mass: 4.2,  rCoM: 0.080, D_outer: 75.0,  t_wall: 5.0,  Izz: 0.015 },
      { id: 5, name: "Bilek 2 (Wrist Pitch)", mass: 2.8,  rCoM: 0.050, D_outer: 60.0,  t_wall: 4.0,  Izz: 0.008 },
      { id: 6, name: "Bilek 3 / Flanş (Tool Flange)", mass: 1.6,  rCoM: 0.040, D_outer: 50.0,  t_wall: 3.5,  Izz: 0.003 }
    ];
  }

  /**
   * Set Active Degrees of Freedom (3, 4, 5, or 6)
   */
  setDOF(dof) {
    this.currentDOF = Math.max(3, Math.min(6, parseInt(dof, 10)));
    // Lock inactive joints to 0
    for (let i = this.currentDOF; i < 6; i++) {
      this.q[i] = 0.0;
    }
    return this.currentDOF;
  }

  /**
   * Clone current DH parameters
   */
  getDHParams() {
    return JSON.parse(JSON.stringify(this.dh));
  }

  /**
   * Update DH parameters (link lengths or offsets)
   */
  setDHParams(newDh) {
    this.dh = JSON.parse(JSON.stringify(newDh));
  }

  /**
   * Standard Denavit-Hartenberg Transformation Matrix A_i
   * A_i = Rot_z(theta_i) * Trans_z(d_i) * Trans_x(a_i) * Rot_x(alpha_i)
   */
  static dhMatrix(theta, d, a, alpha) {
    const ct = Math.cos(theta), st = Math.sin(theta);
    const ca = Math.cos(alpha), sa = Math.sin(alpha);
    return [
      [ct, -st * ca,  st * sa, a * ct],
      [st,  ct * ca, -ct * sa, a * st],
      [0,        sa,       ca,      d],
      [0,         0,        0,      1]
    ];
  }

  /**
   * 4x4 Matrix Multiplication
   */
  static matMul4(A, B) {
    const C = [
      [0, 0, 0, 0],
      [0, 0, 0, 0],
      [0, 0, 0, 0],
      [0, 0, 0, 0]
    ];
    for (let i = 0; i < 4; i++) {
      for (let j = 0; j < 4; j++) {
        let sum = 0;
        for (let k = 0; k < 4; k++) {
          sum += A[i][k] * B[k][j];
        }
        C[i][j] = sum;
      }
    }
    return C;
  }

  /**
   * Compute Forward Kinematics (FK)
   * Returns:
   * - individual: [A1, A2, A3, A4, A5, A6]
   * - cumulative: [T0, T1, T2, T3, T4, T5, T6] where T0 is Identity
   * - tcp: { x, y, z, roll, pitch, yaw, rotationMatrix, quaternion }
   */
  computeFK(q = this.q) {
    const individual = [];
    const cumulative = [];

    // T0 = Identity (Base frame)
    cumulative.push([
      [1, 0, 0, 0],
      [0, 1, 0, 0],
      [0, 0, 1, 0],
      [0, 0, 0, 1]
    ]);

    let T_current = cumulative[0];

    for (let i = 0; i < 6; i++) {
      const p = this.dh[i];
      const qi = (q && q[i] !== undefined) ? q[i] : (this.q[i] || 0.0);
      const thetaTotal = qi + (p.thetaOffset || 0);
      const A_i = RobotKinematics.dhMatrix(thetaTotal, p.d, p.a, p.alpha);
      individual.push(A_i);
      T_current = RobotKinematics.matMul4(T_current, A_i);
      cumulative.push(T_current);
    }

    const T6 = cumulative[6];
    const R = [
      [T6[0][0], T6[0][1], T6[0][2]],
      [T6[1][0], T6[1][1], T6[1][2]],
      [T6[2][0], T6[2][1], T6[2][2]]
    ];

    const pos = {
      x: T6[0][3],
      y: T6[1][3],
      z: T6[2][3]
    };

    const euler = RobotKinematics.rotationMatrixToEulerZYX(R);
    const quat = RobotKinematics.rotationMatrixToQuaternion(R);

    return {
      q: [...q],
      individual,
      cumulative,
      tcp: {
        ...pos,
        roll: euler.roll,
        pitch: euler.pitch,
        yaw: euler.yaw,
        rollDeg: (euler.roll * 180 / Math.PI),
        pitchDeg: (euler.pitch * 180 / Math.PI),
        yawDeg: (euler.yaw * 180 / Math.PI),
        rotationMatrix: R,
        transformationMatrix: T6,
        quaternion: quat
      }
    };
  }

  /**
   * Convert Euler ZYX (Roll-Pitch-Yaw: Rz(yaw)*Ry(pitch)*Rx(roll)) to 3x3 Rotation Matrix
   * angles in radians
   */
  static eulerZYXToRotationMatrix(roll, pitch, yaw) {
    const cr = Math.cos(roll),  sr = Math.sin(roll);
    const cp = Math.cos(pitch), sp = Math.sin(pitch);
    const cy = Math.cos(yaw),   sy = Math.sin(yaw);

    return [
      [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
      [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
      [-sp,     cp * sr,                cp * cr]
    ];
  }

  /**
   * Convert 3x3 Rotation Matrix to Euler ZYX angles (roll, pitch, yaw in radians)
   */
  static rotationMatrixToEulerZYX(R) {
    let pitch = -Math.asin(Math.max(-1, Math.min(1, R[2][0])));
    let roll, yaw;

    if (Math.abs(Math.cos(pitch)) > 1e-6) {
      roll = Math.atan2(R[2][1], R[2][2]);
      yaw = Math.atan2(R[1][0], R[0][0]);
    } else {
      // Gimbal lock
      roll = Math.atan2(-R[1][2], R[1][1]);
      yaw = 0;
    }

    return { roll, pitch, yaw };
  }

  /**
   * Convert 3x3 Rotation Matrix to Quaternion [x, y, z, w]
   */
  static rotationMatrixToQuaternion(R) {
    const tr = R[0][0] + R[1][1] + R[2][2];
    let qx, qy, qz, qw;

    if (tr > 0) {
      const S = Math.sqrt(tr + 1.0) * 2;
      qw = 0.25 * S;
      qx = (R[2][1] - R[1][2]) / S;
      qy = (R[0][2] - R[2][0]) / S;
      qz = (R[1][0] - R[0][1]) / S;
    } else if ((R[0][0] > R[1][1]) && (R[0][0] > R[2][2])) {
      const S = Math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2;
      qw = (R[2][1] - R[1][2]) / S;
      qx = 0.25 * S;
      qy = (R[0][1] + R[1][0]) / S;
      qz = (R[0][2] + R[2][0]) / S;
    } else if (R[1][1] > R[2][2]) {
      const S = Math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2;
      qw = (R[0][2] - R[2][0]) / S;
      qx = (R[0][1] + R[1][0]) / S;
      qy = 0.25 * S;
      qz = (R[1][2] + R[2][1]) / S;
    } else {
      const S = Math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2;
      qw = (R[1][0] - R[0][1]) / S;
      qx = (R[0][2] + R[2][0]) / S;
      qy = (R[1][2] + R[2][1]) / S;
      qz = 0.25 * S;
    }

    return { x: qx, y: qy, z: qz, w: qw };
  }

  /**
   * Quaternion SLERP (Spherical Linear Interpolation)
   */
  static slerpQuaternion(qa, qb, t) {
    let cosHalfTheta = qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z;

    let targetB = { ...qb };
    if (cosHalfTheta < 0) {
      targetB = { w: -qb.w, x: -qb.x, y: -qb.y, z: -qb.z };
      cosHalfTheta = -cosHalfTheta;
    }

    if (cosHalfTheta >= 0.9995) {
      // Linear interpolation when very close
      return {
        w: qa.w + t * (targetB.w - qa.w),
        x: qa.x + t * (targetB.x - qa.x),
        y: qa.y + t * (targetB.y - qa.y),
        z: qa.z + t * (targetB.z - qa.z)
      };
    }

    const halfTheta = Math.acos(cosHalfTheta);
    const sinHalfTheta = Math.sqrt(1.0 - cosHalfTheta * cosHalfTheta);

    const ratioA = Math.sin((1 - t) * halfTheta) / sinHalfTheta;
    const ratioB = Math.sin(t * halfTheta) / sinHalfTheta;

    return {
      w: qa.w * ratioA + targetB.w * ratioB,
      x: qa.x * ratioA + targetB.x * ratioB,
      y: qa.y * ratioA + targetB.y * ratioB,
      z: qa.z * ratioA + targetB.z * ratioB
    };
  }

  /**
   * Convert Quaternion to 3x3 Rotation Matrix
   */
  static quaternionToRotationMatrix(q) {
    const { x, y, z, w } = q;
    const xx = x * x, yy = y * y, zz = z * z;
    const xy = x * y, xz = x * z, yz = y * z;
    const wx = w * x, wy = w * y, wz = w * z;

    return [
      [1 - 2 * (yy + zz),     2 * (xy - wz),         2 * (xz + wy)],
      [2 * (xy + wz),         1 - 2 * (xx + zz),     2 * (yz - wx)],
      [2 * (xz - wy),         2 * (yz + wx),         1 - 2 * (xx + yy)]
    ];
  }

  /**
   * Analytical Inverse Kinematics (Closed-Form Pieper Decoupling)
   * Solves all 8 branches:
   * - Shoulder: Front vs Back (2)
   * - Elbow: Up vs Down (2)
   * - Wrist: Non-Flip vs Flip (2)
   * Total = 2 * 2 * 2 = 8 solutions
   * 
   * @param {Object} target - { x, y, z, roll, pitch, yaw } (angles in radians) or { x, y, z, rotationMatrix }
   * @param {Array} qCurrent - Current joint angles in radians (used to select nearest branch)
   * @returns {Object} { solutions: Array(8), bestIndex: Number, isReachable: Boolean, explanationSteps: Array }
   */
  computeIK(target, qCurrent = this.q) {
    const d1 = this.dh[0].d;
    const a2 = this.dh[1].a;
    const d4 = this.dh[3].d;
    const d6 = this.dh[5].d;

    let R;
    if (target.rotationMatrix) {
      R = target.rotationMatrix;
    } else {
      R = RobotKinematics.eulerZYXToRotationMatrix(target.roll, target.pitch, target.yaw);
    }

    const P = [target.x, target.y, target.z];

    // Educational Breakdown Steps
    const steps = [];

    // Step 1: Wrist Center Calculation
    // Z-axis of tool is R * [0, 0, 1]^T (3rd column of R)
    const zTool = [R[0][2], R[1][2], R[2][2]];
    const Pw = [
      P[0] - d6 * zTool[0],
      P[1] - d6 * zTool[1],
      P[2] - d6 * zTool[2]
    ];

    steps.push({
      step: 1,
      title: "Bilek Merkezi (Wrist Center) Ayrıştırması",
      description: "Pieper teoremine göre son 3 eklem ekseni bilek merkezinde kesişir. Uç nokta pozisyonundan (P) takım uzunluğu (d6) yönelim vektörü boyunca geri çekilerek bilek merkezi P_w bulunur.",
      formula: "P_w = P_{TCP} - d_6 \\cdot R \\begin{bmatrix} 0 \\\\ 0 \\\\ 1 \\end{bmatrix}",
      values: {
        "P_{TCP}": `[${P[0].toFixed(1)}, ${P[1].toFixed(1)}, ${P[2].toFixed(1)}] mm`,
        "d_6": `${d6.toFixed(1)} mm`,
        "z_{tool}": `[${zTool[0].toFixed(3)}, ${zTool[1].toFixed(3)}, ${zTool[2].toFixed(3)}]`,
        "P_w": `[${Pw[0].toFixed(1)}, ${Pw[1].toFixed(1)}, ${Pw[2].toFixed(1)}] mm`
      }
    });

    // Step 2 & 3: Arms 1, 2, 3 Geometry
    const xw = Pw[0], yw = Pw[1], zw = Pw[2];
    const rFront = Math.sqrt(xw * xw + yw * yw);
    const s = zw - d1;
    const D2 = rFront * rFront + s * s;
    const D = Math.sqrt(D2);

    const maxReach = a2 + d4;
    const minReach = Math.abs(a2 - d4);
    const isReachable = (D <= maxReach + 1e-4) && (D >= minReach - 1e-4);

    const K = (D2 - a2 * a2 - d4 * d4) / (2 * a2 * d4);
    const clampedK = Math.max(-1, Math.min(1, K));

    steps.push({
      step: 2,
      title: "İlk 3 Eklemin Düzlemsel Geometrisi ve Erişim Kontrolü",
      description: "Omuz ile bilek merkezi arasındaki mesafe (D) hesaplanır. Kosinüs Teoremi uygulanarak dirsek açısı (θ3) için K parametresi bulunur.",
      formula: "D = \\sqrt{x_w^2 + y_w^2 + (z_w - d_1)^2}, \\quad K = \\frac{D^2 - a_2^2 - d_4^2}{2 a_2 d_4}",
      values: {
        "r (Yatay Erişim)": `${rFront.toFixed(1)} mm`,
        "s (Dikey Mesafe)": `${s.toFixed(1)} mm`,
        "D (Bilek Mesafesi)": `${D.toFixed(1)} mm`,
        "Maksimum Erişim (a2+d4)": `${maxReach.toFixed(1)} mm`,
        "K Parametresi": clampedK.toFixed(4),
        "Erişilebilirlik": isReachable ? "ERİŞİLEBİLİR (Hedef çalışma zarfında)" : "ERİŞİLEMEZ (Çalışma alanı dışında!)"
      }
    });

    const solutions = [];

    // Shoulder configurations: Front (0) vs Back (1)
    const shoulderConfigs = [
      { name: "Omuz Ön (Front)", th1: Math.atan2(yw, xw), r: rFront, id: "Front" },
      { name: "Omuz Arka (Back)", th1: Math.atan2(-yw, -xw), r: -rFront, id: "Back" }
    ];

    // Elbow configurations: Up (0) vs Down (1)
    const elbowConfigs = [
      { name: "Dirsek Yukarı (Elbow Up)", sign: 1, id: "Up" },
      { name: "Dirsek Aşağı (Elbow Down)", sign: -1, id: "Down" }
    ];

    // Wrist configurations: Non-Flip (0) vs Flip (1)
    const wristConfigs = [
      { name: "Bilek Normal (No-Flip)", sign: 1, id: "NoFlip" },
      { name: "Bilek Ters (Flip)", sign: -1, id: "Flip" }
    ];

    for (const sc of shoulderConfigs) {
      const th1 = sc.th1;
      const r = sc.r;

      for (const ec of elbowConfigs) {
        // cos(th3) can be positive or negative
        const cosTh3 = ec.sign * Math.sqrt(Math.max(0, 1 - clampedK * clampedK));
        const th3 = Math.atan2(clampedK, cosTh3);

        const X_prime = a2 + d4 * Math.sin(th3);
        const Y_prime = -d4 * Math.cos(th3);
        const th2 = Math.atan2(s, r) - Math.atan2(Y_prime, X_prime);

        // Transformation T03
        const A1 = RobotKinematics.dhMatrix(th1, d1, 0, Math.PI / 2);
        const A2 = RobotKinematics.dhMatrix(th2, 0, a2, 0);
        const A3 = RobotKinematics.dhMatrix(th3, 0, 0, Math.PI / 2);
        const T03 = RobotKinematics.matMul4(RobotKinematics.matMul4(A1, A2), A3);

        // R36 = (R03)^T * R
        const R36 = [
          [0, 0, 0],
          [0, 0, 0],
          [0, 0, 0]
        ];
        for (let i = 0; i < 3; i++) {
          for (let j = 0; j < 3; j++) {
            let sum = 0;
            for (let k = 0; k < 3; k++) {
              sum += T03[k][i] * R[k][j];
            }
            R36[i][j] = sum;
          }
        }

        const sinTh5Mag = Math.sqrt(R36[0][2] * R36[0][2] + R36[1][2] * R36[1][2]);
        const cosTh5 = R36[2][2];

        for (const wc of wristConfigs) {
          const sinTh5 = wc.sign * sinTh5Mag;
          const th5 = Math.atan2(sinTh5, cosTh5);

          let th4, th6;
          if (sinTh5Mag > 1e-4) {
            if (wc.sign > 0) {
              th4 = Math.atan2(R36[1][2], R36[0][2]);
              th6 = Math.atan2(R36[2][1], -R36[2][0]);
            } else {
              th4 = Math.atan2(-R36[1][2], -R36[0][2]);
              th6 = Math.atan2(-R36[2][1], R36[2][0]);
            }
          } else {
            // Wrist singularity (theta5 = 0 or 180)
            th4 = qCurrent[3] || 0;
            th6 = Math.atan2(R36[1][0], R36[0][0]) - th4;
          }

          // Normalize angles to [-pi, pi]
          const qNorm = [
            RobotKinematics.normalizeAngle(th1),
            RobotKinematics.normalizeAngle(th2),
            RobotKinematics.normalizeAngle(th3),
            RobotKinematics.normalizeAngle(th4),
            RobotKinematics.normalizeAngle(th5),
            RobotKinematics.normalizeAngle(th6)
          ];

          // Check joint limits
          let withinLimits = true;
          for (let i = 0; i < 6; i++) {
            const deg = qNorm[i] * 180 / Math.PI;
            if (deg < this.dh[i].minDeg - 1e-2 || deg > this.dh[i].maxDeg + 1e-2) {
              withinLimits = false;
              break;
            }
          }

          // Angular distance to current pose (weighted norm)
          let distance = 0;
          for (let i = 0; i < 6; i++) {
            const diff = RobotKinematics.normalizeAngle(qNorm[i] - qCurrent[i]);
            distance += diff * diff;
          }
          distance = Math.sqrt(distance);

          solutions.push({
            id: `${sc.id}_${ec.id}_${wc.id}`,
            label: `${sc.name} | ${ec.name} | ${wc.name}`,
            q: qNorm,
            qDeg: qNorm.map(val => (val * 180 / Math.PI)),
            withinLimits: withinLimits && isReachable,
            distance,
            shoulder: sc.name,
            elbow: ec.name,
            wrist: wc.name
          });
        }
      }
    }

    // Step 4: Wrist Euler Angles Breakdown
    steps.push({
      step: 3,
      title: "Bilek Eksenlerinin (θ4, θ5, θ6) Euler ZYZ Ayrıştırması",
      description: "İlk 3 eklemin oluşturduğu rotasyon matrisi (R03) hedeften çıkarılır: R36 = (R03)^T * R. Elde edilen R36 matrisinden küresel bilek açıları trigonometrik olarak çözülür.",
      formula: "R_3^6 = (R_0^3)^T \\cdot R_{Hedef}, \\quad \\theta_5 = \\text{atan2}(\\pm \\sqrt{R_{13}^2 + R_{23}^2}, R_{33})",
      values: {
        "θ4 Formülü": "\\text{atan2}(R_{23}, R_{13})",
        "θ5 Formülü": "\\text{atan2}(\\sqrt{R_{13}^2 + R_{23}^2}, R_{33})",
        "θ6 Formülü": "\\text{atan2}(R_{32}, -R_{31})"
      }
    });

    // Select the best solution: closest reachable solution within limits
    let bestIndex = 0;
    let minDistance = Infinity;

    for (let i = 0; i < solutions.length; i++) {
      if (solutions[i].withinLimits && solutions[i].distance < minDistance) {
        minDistance = solutions[i].distance;
        bestIndex = i;
      }
    }

    if (minDistance === Infinity) {
      // If none within limits, pick smallest distance
      bestIndex = 0;
      let minD = Infinity;
      for (let i = 0; i < solutions.length; i++) {
        if (solutions[i].distance < minD) {
          minD = solutions[i].distance;
          bestIndex = i;
        }
      }
    }

    return {
      solutions,
      bestIndex,
      bestSolution: solutions[bestIndex],
      isReachable,
      steps
    };
  }

  /**
   * Angle normalization to [-PI, PI]
   */
  static normalizeAngle(angle) {
    let a = angle % (2 * Math.PI);
    if (a > Math.PI) a -= 2 * Math.PI;
    if (a < -Math.PI) a += 2 * Math.PI;
    return a;
  }

  /**
   * Vector cross product
   */
  static cross(a, b) {
    return [
      a[1] * b[2] - a[2] * b[1],
      a[2] * b[0] - a[0] * b[2],
      a[0] * b[1] - a[1] * b[0]
    ];
  }

  /**
   * Vector dot product
   */
  static dot(a, b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  }

  /**
   * Compute Geometric Jacobian Matrix (6x6)
   * J = [J_1, J_2, J_3, J_4, J_5, J_6]
   * For revolute joint i:
   * J_v,i = z_{i-1} x (p_e - p_{i-1})
   * J_w,i = z_{i-1}
   * 
   * Returns:
   * - J: 6x6 matrix
   * - det: determinant
   * - manipulability: Yoshikawa index w = sqrt(det(J*J^T))
   * - singularities: { wrist: Boolean, shoulder: Boolean, elbow: Boolean, isSingular: Boolean }
   */
  /**
   * Compute Geometric Jacobian Matrix (Adapted for 3, 4, 5, or 6 DOF)
   * J = [J_1 ... J_n]
   * 
   * Returns:
   * - J: Active n x n matrix for the selected DOF
   * - fullJ: Complete 6x6 spatial Jacobian
   * - det: determinant of active matrix
   * - manipulability: Yoshikawa index w
   * - dof: Active DOF count
   * - singularities: diagnostic checks
   */
  computeJacobian(q = this.q, dof = this.currentDOF) {
    const n = dof || this.currentDOF || 6;
    const fk = this.computeFK(q);
    const T = fk.cumulative;
    const pe = [T[6][0][3], T[6][1][3], T[6][2][3]]; // Tool Center Point (TCP)

    const fullJ = [
      [0, 0, 0, 0, 0, 0], // v_x
      [0, 0, 0, 0, 0, 0], // v_y
      [0, 0, 0, 0, 0, 0], // v_z
      [0, 0, 0, 0, 0, 0], // w_x
      [0, 0, 0, 0, 0, 0], // w_y
      [0, 0, 0, 0, 0, 0]  // w_z
    ];

    for (let i = 0; i < 6; i++) {
      const Ti = T[i];
      const pi_prev = [Ti[0][3], Ti[1][3], Ti[2][3]];
      const zi_prev = [Ti[0][2], Ti[1][2], Ti[2][2]];

      const r = [
        (pe[0] - pi_prev[0]) / 1000.0,
        (pe[1] - pi_prev[1]) / 1000.0,
        (pe[2] - pi_prev[2]) / 1000.0
      ];

      const Jv = RobotKinematics.cross(zi_prev, r);
      const Jw = zi_prev;

      fullJ[0][i] = Jv[0];
      fullJ[1][i] = Jv[1];
      fullJ[2][i] = Jv[2];
      fullJ[3][i] = Jw[0];
      fullJ[4][i] = Jw[1];
      fullJ[5][i] = Jw[2];
    }

    // Extract active Jacobian submatrix based on DOF
    let J = [];
    let rowLabels = [];

    if (n === 3) {
      // 3-DOF: Linear position velocities only [v_x, v_y, v_z] vs [q1, q2, q3]
      J = [
        [fullJ[0][0], fullJ[0][1], fullJ[0][2]],
        [fullJ[1][0], fullJ[1][1], fullJ[1][2]],
        [fullJ[2][0], fullJ[2][1], fullJ[2][2]]
      ];
      rowLabels = ['v_x', 'v_y', 'v_z'];
    } else if (n === 4) {
      // 4-DOF SCARA / Palletizer: [v_x, v_y, v_z, w_z] vs [q1...q4]
      J = [
        [fullJ[0][0], fullJ[0][1], fullJ[0][2], fullJ[0][3]],
        [fullJ[1][0], fullJ[1][1], fullJ[1][2], fullJ[1][3]],
        [fullJ[2][0], fullJ[2][1], fullJ[2][2], fullJ[2][3]],
        [fullJ[5][0], fullJ[5][1], fullJ[5][2], fullJ[5][3]]
      ];
      rowLabels = ['v_x', 'v_y', 'v_z', 'ω_z'];
    } else if (n === 5) {
      // 5-DOF: [v_x, v_y, v_z, w_x, w_y] vs [q1...q5]
      J = [
        [fullJ[0][0], fullJ[0][1], fullJ[0][2], fullJ[0][3], fullJ[0][4]],
        [fullJ[1][0], fullJ[1][1], fullJ[1][2], fullJ[1][3], fullJ[1][4]],
        [fullJ[2][0], fullJ[2][1], fullJ[2][2], fullJ[2][3], fullJ[2][4]],
        [fullJ[3][0], fullJ[3][1], fullJ[3][2], fullJ[3][3], fullJ[3][4]],
        [fullJ[4][0], fullJ[4][1], fullJ[4][2], fullJ[4][3], fullJ[4][4]]
      ];
      rowLabels = ['v_x', 'v_y', 'v_z', 'ω_x', 'ω_y'];
    } else {
      // 6-DOF Full Spatial Jacobian
      J = fullJ.map(row => [...row]);
      rowLabels = ['v_x', 'v_y', 'v_z', 'ω_x', 'ω_y', 'ω_z'];
    }

    // Determinant calculation of active square matrix
    const det = RobotKinematics.matrixDetN(J);
    const manipulability = Math.abs(det);

    // Singularity diagnostic checks:
    const wristSingular = (n >= 5) && (Math.abs(Math.sin(q[4])) < 0.05);

    const T4 = T[Math.min(4, n - 1)];
    const pw = [T4[0][3], T4[1][3], T4[2][3]];
    const rw = Math.sqrt(pw[0] * pw[0] + pw[1] * pw[1]);
    const shoulderSingular = rw < 30.0;

    const a2 = this.dh[1].a;
    const d4 = this.dh[3].d;
    const sShoulder = pw[2] - this.dh[0].d;
    const distArm = Math.sqrt(rw * rw + sShoulder * sShoulder);
    const elbowSingular = Math.abs(distArm - (a2 + d4)) < 25.0 || Math.abs(distArm - Math.abs(a2 - d4)) < 25.0;

    return {
      dof: n,
      J,
      fullJ,
      rowLabels,
      det,
      manipulability,
      singularities: {
        wrist: wristSingular,
        shoulder: shoulderSingular,
        elbow: elbowSingular,
        isNearSingular: wristSingular || shoulderSingular || elbowSingular || manipulability < 0.005
      }
    };
  }

  /**
   * Determinant of NxN Matrix via Gaussian Elimination with partial pivoting
   */
  static matrixDetN(M) {
    const A = M.map(row => [...row]);
    let det = 1.0;
    const n = A.length;

    for (let i = 0; i < n; i++) {
      let pivot = i;
      for (let j = i + 1; j < n; j++) {
        if (Math.abs(A[j][i]) > Math.abs(A[pivot][i])) {
          pivot = j;
        }
      }

      if (Math.abs(A[pivot][i]) < 1e-12) return 0.0;

      if (pivot !== i) {
        const temp = A[i];
        A[i] = A[pivot];
        A[pivot] = temp;
        det = -det;
      }

      det *= A[i][i];

      for (let j = i + 1; j < n; j++) {
        const factor = A[j][i] / A[i][i];
        for (let k = i + 1; k < n; k++) {
          A[j][k] -= factor * A[i][k];
        }
      }
    }

    return det;
  }

  /**
   * Determinant of 6x6 Matrix via LU / Gaussian Elimination with partial pivoting
   */
  static matrixDet6(M) {
    const A = M.map(row => [...row]);
    let det = 1.0;
    const n = 6;

    for (let i = 0; i < n; i++) {
      let pivot = i;
      for (let j = i + 1; j < n; j++) {
        if (Math.abs(A[j][i]) > Math.abs(A[pivot][i])) {
          pivot = j;
        }
      }

      if (Math.abs(A[pivot][i]) < 1e-12) return 0.0;

      if (pivot !== i) {
        const temp = A[i];
        A[i] = A[pivot];
        A[pivot] = temp;
        det = -det;
      }

      det *= A[i][i];

      for (let j = i + 1; j < n; j++) {
        const factor = A[j][i] / A[i][i];
        for (let k = i + 1; k < n; k++) {
          A[j][k] -= factor * A[i][k];
        }
      }
    }

    return det;
  }

  /**
   * Trajectory Generator: Point A to Point B
   * 
   * Modes:
   * 1. 'joint-quintic': 5th order polynomial in joint space (continuous pos, vel, acc)
   * 2. 'joint-cubic': 3rd order polynomial in joint space (continuous pos, vel)
   * 3. 'joint-trapezoidal': Trapezoidal velocity profile (LSPB) in joint space
   * 4. 'cartesian-linear': Cartesian straight line (LIN) with SLERP orientation + continuous IK
   * 
   * @param {Array} qA - Start joint angles (6 rad)
   * @param {Array} qB - Target joint angles (6 rad)
   * @param {Object} poseA - Start Cartesian pose { x, y, z, roll, pitch, yaw }
   * @param {Object} poseB - Target Cartesian pose { x, y, z, roll, pitch, yaw }
   * @param {Number} duration - Total motion duration in seconds
   * @param {Number} stepsCount - Number of discrete trajectory samples
   * @param {String} mode - Interpolation method
   * @returns {Object} Trajectory data with sample arrays for charts and animation
   */
  generateTrajectory(qA, qB, poseA, poseB, duration = 3.0, stepsCount = 120, mode = 'joint-quintic') {
    const timeSamples = [];
    const qSamples = [];
    const qDotSamples = [];
    const qDoubleDotSamples = [];
    const tcpSamples = [];
    const manipulabilitySamples = [];

    const dt = duration / (stepsCount - 1);

    const quatA = RobotKinematics.rotationMatrixToQuaternion(
      RobotKinematics.eulerZYXToRotationMatrix(poseA.roll, poseA.pitch, poseA.yaw)
    );
    const quatB = RobotKinematics.rotationMatrixToQuaternion(
      RobotKinematics.eulerZYXToRotationMatrix(poseB.roll, poseB.pitch, poseB.yaw)
    );

    let lastContinuousQ = [...qA];

    for (let step = 0; step < stepsCount; step++) {
      const t = step * dt;
      const tau = Math.min(1.0, Math.max(0.0, t / duration)); // normalized time [0, 1]

      let s = 0, sDot = 0, sDoubleDot = 0;

      if (mode === 'joint-cubic') {
        // Cubic: s(tau) = 3*tau^2 - 2*tau^3
        s = 3 * tau * tau - 2 * tau * tau * tau;
        sDot = (6 * tau - 6 * tau * tau) / duration;
        sDoubleDot = (6 - 12 * tau) / (duration * duration);
      } else if (mode === 'joint-trapezoidal') {
        // Trapezoidal LSPB with tb = duration / 4
        const tb = duration * 0.25;
        const aCruise = 1.0 / (tb * (duration - tb));
        const vCruise = aCruise * tb;

        if (t <= tb) {
          s = 0.5 * aCruise * t * t;
          sDot = aCruise * t;
          sDoubleDot = aCruise;
        } else if (t <= duration - tb) {
          s = 0.5 * aCruise * tb * tb + vCruise * (t - tb);
          sDot = vCruise;
          sDoubleDot = 0;
        } else {
          const tRem = duration - t;
          s = 1.0 - 0.5 * aCruise * tRem * tRem;
          sDot = aCruise * tRem;
          sDoubleDot = -aCruise;
        }
      } else {
        // Default: Quintic s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5 (Smooth jerk-free)
        s = 10 * Math.pow(tau, 3) - 15 * Math.pow(tau, 4) + 6 * Math.pow(tau, 5);
        sDot = (30 * Math.pow(tau, 2) - 60 * Math.pow(tau, 3) + 30 * Math.pow(tau, 4)) / duration;
        sDoubleDot = (60 * tau - 180 * Math.pow(tau, 2) + 120 * Math.pow(tau, 3)) / (duration * duration);
      }

      let qCurrent = [];
      let qDotCurrent = [];
      let qDoubleDotCurrent = [];
      let tcpPose = null;

      if (mode === 'cartesian-linear') {
        // Cartesian Space Linear Interpolation (LIN)
        const currentPos = {
          x: poseA.x + s * (poseB.x - poseA.x),
          y: poseA.y + s * (poseB.y - poseA.y),
          z: poseA.z + s * (poseB.z - poseA.z)
        };

        const currentQuat = RobotKinematics.slerpQuaternion(quatA, quatB, s);
        const currentR = RobotKinematics.quaternionToRotationMatrix(currentQuat);
        const currentEuler = RobotKinematics.rotationMatrixToEulerZYX(currentR);

        // Solve IK at this Cartesian waypoint
        const ikRes = this.computeIK({
          x: currentPos.x,
          y: currentPos.y,
          z: currentPos.z,
          rotationMatrix: currentR
        }, lastContinuousQ);

        qCurrent = [...ikRes.bestSolution.q];
        lastContinuousQ = [...qCurrent];

        // Numerical finite difference for joint velocities and accelerations
        if (qSamples.length > 0) {
          const prevQ = qSamples[qSamples.length - 1];
          qDotCurrent = qCurrent.map((qi, idx) => (qi - prevQ[idx]) / dt);

          if (qDotSamples.length > 0) {
            const prevQDot = qDotSamples[qDotSamples.length - 1];
            qDoubleDotCurrent = qDotCurrent.map((qdi, idx) => (qdi - prevQDot[idx]) / dt);
          } else {
            qDoubleDotCurrent = [0, 0, 0, 0, 0, 0];
          }
        } else {
          qDotCurrent = [0, 0, 0, 0, 0, 0];
          qDoubleDotCurrent = [0, 0, 0, 0, 0, 0];
        }

        const linSpeed = sDot * Math.sqrt(
          Math.pow(poseB.x - poseA.x, 2) +
          Math.pow(poseB.y - poseA.y, 2) +
          Math.pow(poseB.z - poseA.z, 2)
        );

        tcpPose = {
          ...currentPos,
          roll: currentEuler.roll,
          pitch: currentEuler.pitch,
          yaw: currentEuler.yaw,
          speed: linSpeed
        };
      } else {
        // Joint Space Interpolation (PTP)
        for (let i = 0; i < 6; i++) {
          const deltaQ = qB[i] - qA[i];
          const qi = qA[i] + deltaQ * s;
          const qd = deltaQ * sDot;
          const qdd = deltaQ * sDoubleDot;

          qCurrent.push(qi);
          qDotCurrent.push(qd);
          qDoubleDotCurrent.push(qdd);
        }

        // Forward Kinematics to find TCP position along path
        const fk = this.computeFK(qCurrent);
        tcpPose = {
          x: fk.tcp.x,
          y: fk.tcp.y,
          z: fk.tcp.z,
          roll: fk.tcp.roll,
          pitch: fk.tcp.pitch,
          yaw: fk.tcp.yaw,
          speed: 0 // Will compute below
        };
      }

      // Calculate speed for joint space motions via finite differences
      if (tcpSamples.length > 0 && mode !== 'cartesian-linear') {
        const prevTcp = tcpSamples[tcpSamples.length - 1];
        const dist = Math.sqrt(
          Math.pow(tcpPose.x - prevTcp.x, 2) +
          Math.pow(tcpPose.y - prevTcp.y, 2) +
          Math.pow(tcpPose.z - prevTcp.z, 2)
        );
        tcpPose.speed = dist / dt;
      }

      // Jacobian and manipulability measure
      const jac = this.computeJacobian(qCurrent);

      timeSamples.push(t);
      qSamples.push(qCurrent);
      qDotSamples.push(qDotCurrent);
      qDoubleDotSamples.push(qDoubleDotCurrent);
      tcpSamples.push(tcpPose);
      manipulabilitySamples.push(jac.manipulability);
    }

    return {
      mode,
      duration,
      dt,
      stepsCount,
      timeSamples,
      qSamples,
      qDotSamples,
      qDoubleDotSamples,
      tcpSamples,
      manipulabilitySamples,
      qA: [...qA],
      qB: [...qB],
      poseA: { ...poseA },
      poseB: { ...poseB }
    };
  }

  /**
   * Multi-DOF Theoretical Comparison & Kinematic Analysis
   * Compares 3-DOF, 4-DOF, 5-DOF, and 6-DOF architectures
   */
  getDOFComparison() {
    return {
      3: {
        dof: 3,
        name: "3-DOF Artiküle / Düzlemsel Konumlandırma Kolu (RRR)",
        taskSpace: "Yalnızca 3B Konum (X, Y, Z)",
        uncontrolled: "Tüm 3B Yönelim Serbesttir (Roll, Pitch, Yaw kontrol edilemez)",
        jacobianDim: "3x3 [v_x, v_y, v_z]^T = J · [dq1, dq2, dq3]^T",
        matrixSize: [3, 3],
        singularityCondition: "det(J_3x3) = 0. Dirsek tam açıldığında veya omuz eksenine yaklaşıldığında.",
        applications: "Basit transfer kolları, 3B yazıcı ekstrüder taşıyıcıları, kaba istifleme.",
        differenceFrom6DOF: "Uç noktanın oryantasyonu (takımın duruş açısı) bağımsız olarak ayarlanamaz. Takım parçaya yalnızca kolun geldiği açıyla temas edebilir."
      },
      4: {
        dof: 4,
        name: "4-DOF SCARA / Dikey Paletleme Robotu (RRRP / RRRR)",
        taskSpace: "3B Konum (X, Y, Z) + Düşey Dönme (Yaw / Rz)",
        uncontrolled: "Roll (Rx) ve Pitch (Ry) mekanik olarak zemine dik kilitlidir (0°).",
        jacobianDim: "4x4 [v_x, v_y, v_z, ω_z]^T = J · [dq1...dq4]^T",
        matrixSize: [4, 4],
        singularityCondition: "det(J_4x4) = 0. Yatay iki kolun tam doğrusal hale gelmesi (r = a1 + a2).",
        applications: "Elektronik PCB dizgi (Pick & place), kutulama, konveyörden alma.",
        differenceFrom6DOF: "Uç flanş her zaman zemine paraleldir. Eğimli yüzeylerde delme veya yan taraftan kaynak yapamaz; ancak dikey rijitliği ve hızı 6-eksenliye göre çok daha yüksektir."
      },
      5: {
        dof: 5,
        name: "5-DOF Artiküle Robot (Roll Eksensiz Manipülatör)",
        taskSpace: "3B Konum (X, Y, Z) + Yönelim Açısı (Pitch, Yaw)",
        uncontrolled: "Takım ekseni etrafında dönme (Roll / Takım dönüşü) eksiktir.",
        jacobianDim: "5x5 [v_x, v_y, v_z, ω_x, ω_y]^T = J · [dq1...dq5]^T",
        matrixSize: [5, 5],
        singularityCondition: "det(J_5x5) = 0. Bilek eğim açısı sıfır olduğunda veya kol sınırında.",
        applications: "Gazaltı / Lazer ark kaynağı, sprey boyama, püskürtme, yapıştırıcı sürme (Eksenel simetrik takımlar).",
        differenceFrom6DOF: "Torç veya püskürtme nozülü silindirik simetriye sahip olduğu için kendi ekseni etrafında dönmesine ihtiyaç yoktur. Maliyet ve ağırlık 6-DOF'a göre %25 daha düşüktür."
      },
      6: {
        dof: 6,
        name: "6-DOF Tam Uzamsal Endüstriyel Robot Kolu",
        taskSpace: "Tam Uzamsal Serbestlik: Konum (X, Y, Z) + Tam Yönelim (Roll, Pitch, Yaw)",
        uncontrolled: "Yok (Tamamen kontrol edilebilir uzamsal 6 serbestlik)",
        jacobianDim: "6x6 [v; ω]^T = J · [dq1...dq6]^T",
        matrixSize: [6, 6],
        singularityCondition: "det(J_6x6) = 0. Bilek tekilliği (θ5 = 0), Omuz tekilliği (r = 0), Dirsek sınırı.",
        applications: "CNC işleme, montaj, orbital kaynak, karmaşık parça işleme, cerrahi robotik.",
        differenceFrom6DOF: "Evrensel kinematik yapıya sahiptir. 3B uzayda erişilebilir her noktaya istenen herhangi bir açıyla yaklaşabilir."
      }
    };
  }

  /**
   * Recursive Newton-Euler (RNE) Dynamic Torque & Force Calculator
   * Calculates joint torques, link accelerations, inertial forces, and moments
   * 
   * @param {Array} q - Joint angles (rad)
   * @param {Array} qDot - Joint velocities (rad/s)
   * @param {Array} qDoubleDot - Joint accelerations (rad/s^2)
   * @param {Number} payloadKg - End-effector payload mass (kg)
   * @returns {Object} Dynamics results with torques, link forces, moments, and energies
   */
  computeNewtonEulerDynamics(q = this.q, qDot = [0,0,0,0,0,0], qDoubleDot = [0,0,0,0,0,0], payloadKg = 3.0) {
    const n = this.currentDOF || 6;
    const g0 = [0, 0, 9.81]; // Gravity vector in base frame (m/s^2)

    const fk = this.computeFK(q);
    const T = fk.cumulative;

    // Link relative rotation matrices R_{i-1}^i and displacement vectors P_i
    const R_rel = [];
    const P_vec = [];

    for (let i = 0; i < n; i++) {
      const Ai = fk.individual[i];
      // 3x3 rotation of Ai
      const R_i = [
        [Ai[0][0], Ai[0][1], Ai[0][2]],
        [Ai[1][0], Ai[1][1], Ai[1][2]],
        [Ai[2][0], Ai[2][1], Ai[2][2]]
      ];
      // displacement in meters
      const P_i = [Ai[0][3] / 1000.0, Ai[1][3] / 1000.0, Ai[2][3] / 1000.0];

      R_rel.push(R_i);
      P_vec.push(P_i);
    }

    // Forward Recursion: Kinematic variables (velocities and accelerations)
    const omega = [[0, 0, 0]]; // Base angular velocity
    const alpha = [[0, 0, 0]]; // Base angular acceleration
    const a_orig = [[g0[0], g0[1], g0[2]]]; // Base acceleration with gravity offset
    const a_com = [];
    const F_inertial = [];
    const N_inertial = [];

    const z0 = [0, 0, 1]; // Joint rotation axis in local frame

    for (let i = 0; i < n; i++) {
      const R = R_rel[i];
      const RT = [
        [R[0][0], R[1][0], R[2][0]],
        [R[0][1], R[1][1], R[2][1]],
        [R[0][2], R[1][2], R[2][2]]
      ];

      const qd = qDot[i] || 0.0;
      const qdd = qDoubleDot[i] || 0.0;

      // omega_i = R^T * omega_{i-1} + z0 * qd
      const w_prev_rot = RobotKinematics.matVecMul3(RT, omega[i]);
      const w_i = [w_prev_rot[0], w_prev_rot[1], w_prev_rot[2] + qd];
      omega.push(w_i);

      // alpha_i = R^T * alpha_{i-1} + z0 * qdd + omega_i x (z0 * qd)
      const a_prev_rot = RobotKinematics.matVecMul3(RT, alpha[i]);
      const cross_w_z = RobotKinematics.cross(w_i, [0, 0, qd]);
      const a_i = [
        a_prev_rot[0] + cross_w_z[0],
        a_prev_rot[1] + cross_w_z[1],
        a_prev_rot[2] + qdd + cross_w_z[2]
      ];
      alpha.push(a_i);

      // a_origin_i = R^T * a_{origin, i-1} + alpha_i x P_i + omega_i x (omega_i x P_i)
      const a_orig_rot = RobotKinematics.matVecMul3(RT, a_orig[i]);
      const P = P_vec[i];
      const alpha_x_P = RobotKinematics.cross(a_i, P);
      const w_x_P = RobotKinematics.cross(w_i, P);
      const w_x_w_x_P = RobotKinematics.cross(w_i, w_x_P);

      const a_orig_i = [
        a_orig_rot[0] + alpha_x_P[0] + w_x_w_x_P[0],
        a_orig_rot[1] + alpha_x_P[1] + w_x_w_x_P[1],
        a_orig_rot[2] + alpha_x_P[2] + w_x_w_x_P[2]
      ];
      a_orig.push(a_orig_i);

      // Center of Mass acceleration
      const dyn = this.linkDynamics[i];
      let m_i = dyn.mass;
      if (i === n - 1) m_i += payloadKg; // Add payload to end-effector

      const r_com = [0, 0, dyn.rCoM]; // CoM vector
      const alpha_x_rc = RobotKinematics.cross(a_i, r_com);
      const w_x_rc = RobotKinematics.cross(w_i, r_com);
      const w_x_w_x_rc = RobotKinematics.cross(w_i, w_x_rc);

      const a_com_i = [
        a_orig_i[0] + alpha_x_rc[0] + w_x_w_x_rc[0],
        a_orig_i[1] + alpha_x_rc[1] + w_x_w_x_rc[1],
        a_orig_i[2] + alpha_x_rc[2] + w_x_w_x_rc[2]
      ];
      a_com.push(a_com_i);

      // Inertial Forces & Torques: F_i = m_i * a_com_i, N_i = I_i * alpha_i + omega_i x (I_i * omega_i)
      const F_i = [m_i * a_com_i[0], m_i * a_com_i[1], m_i * a_com_i[2]];
      const Izz = dyn.Izz;
      const I_w = [Izz * w_i[0] * 0.5, Izz * w_i[1] * 0.5, Izz * w_i[2]];
      const w_x_Iw = RobotKinematics.cross(w_i, I_w);
      const N_i = [
        Izz * 0.5 * a_i[0] + w_x_Iw[0],
        Izz * 0.5 * a_i[1] + w_x_Iw[1],
        Izz * a_i[2] + w_x_Iw[2]
      ];

      F_inertial.push(F_i);
      N_inertial.push(N_i);
    }

    // Backward Recursion: Forces, Moments, and Joint Motor Torques
    const torques = new Array(n).fill(0);
    const linkForces = new Array(n).fill(null);
    const linkMoments = new Array(n).fill(null);

    let f_next = [0, 0, payloadKg * 9.81]; // External payload force at end-effector
    let n_next = [0, 0, 0];

    for (let i = n - 1; i >= 0; i--) {
      const R = R_rel[i];
      const P = P_vec[i];
      const F_i = F_inertial[i];
      const N_i = N_inertial[i];
      const r_com = [0, 0, this.linkDynamics[i].rCoM];

      // f_i = R_{i+1} * f_{i+1} + F_i
      const f_next_rot = (i === n - 1) ? f_next : RobotKinematics.matVecMul3(R_rel[i + 1], f_next);
      const f_i = [
        f_next_rot[0] + F_i[0],
        f_next_rot[1] + F_i[1],
        f_next_rot[2] + F_i[2]
      ];

      // n_i = N_i + R_{i+1} * n_{i+1} + P_i x (R_{i+1} * f_{i+1}) + (P_i + r_com) x F_i
      const n_next_rot = (i === n - 1) ? n_next : RobotKinematics.matVecMul3(R_rel[i + 1], n_next);
      const P_x_fnext = RobotKinematics.cross(P, f_next_rot);
      const P_plus_rc = [P[0] + r_com[0], P[1] + r_com[1], P[2] + r_com[2]];
      const Prc_x_F = RobotKinematics.cross(P_plus_rc, F_i);

      const n_i = [
        N_i[0] + n_next_rot[0] + P_x_fnext[0] + Prc_x_F[0],
        N_i[1] + n_next_rot[1] + P_x_fnext[1] + Prc_x_F[1],
        N_i[2] + n_next_rot[2] + P_x_fnext[2] + Prc_x_F[2]
      ];

      // Motor torque tau_i is the projection along the z-axis (joint axis)
      torques[i] = n_i[2];
      linkForces[i] = f_i;
      linkMoments[i] = n_i;

      f_next = f_i;
      n_next = n_i;
    }

    // Structural Stress & Strength Calculation
    const structuralAnalysis = this.computeStructuralStrength(torques, linkForces, linkMoments);

    return {
      dof: n,
      torques,
      linkForces,
      linkMoments,
      accelerations: { linear: a_com, angular: alpha.slice(1) },
      structuralAnalysis
    };
  }

  /**
   * Structural Strength & Stress Sizing Analysis (Mukavemet Analizi)
   * Calculates bending moment, shear, Von Mises stress, and Factor of Safety (FOS)
   */
  computeStructuralStrength(torques, linkForces = null, linkMoments = null) {
    if (torques && typeof torques === 'object' && !Array.isArray(torques)) {
      if (torques.structuralAnalysis) return torques.structuralAnalysis;
      linkMoments = torques.linkMoments;
      linkForces = torques.linkForces;
      torques = torques.torques;
    }
    const n = (torques && torques.length) || 0;
    const results = [];
    const sigmaYield = this.material.yieldStressMPa; // e.g. 276 MPa for Al 6061-T6

    for (let i = 0; i < n; i++) {
      const dyn = this.linkDynamics[i];
      const M = linkMoments[i] || [0, 0, 0];
      const F = linkForces[i] || [0, 0, 0];

      // Bending moment magnitude (orthogonal to axis): M_b = sqrt(Mx^2 + My^2)
      const Mb = Math.sqrt(M[0] * M[0] + M[1] * M[1]); // in Nm
      const Torsion = Math.abs(torques[i]); // in Nm
      const ShearForce = Math.sqrt(F[0] * F[0] + F[1] * F[1]); // in N
      const AxialForce = Math.abs(F[2]); // in N

      // Hollow circular cross-section geometry: D_outer, t_wall in meters
      const D = dyn.D_outer / 1000.0;
      const t = dyn.t_wall / 1000.0;
      const d = Math.max(0.001, D - 2 * t);

      // Cross-sectional Area: A = pi * (D^2 - d^2) / 4
      const Area = (Math.PI * (D * D - d * d)) / 4.0; // m^2

      // Section Modulus for bending: W = pi * (D^4 - d^4) / (32 * D)
      const W_bending = (Math.PI * (Math.pow(D, 4) - Math.pow(d, 4))) / (32.0 * D); // m^3
      // Polar section modulus for torsion: W_p = 2 * W_bending
      const W_polar = 2.0 * W_bending;

      // Normal Bending Stress: sigma_b = M_b / W_bending (in Pa -> convert to MPa)
      const sigma_bending = (Mb / W_bending) / 1e6;
      // Axial Stress: sigma_axial = F_axial / Area
      const sigma_axial = (AxialForce / Area) / 1e6;
      const sigma_normal_total = sigma_bending + sigma_axial;

      // Torsional Shear Stress: tau_torsion = T / W_polar
      const tau_torsion = (Torsion / W_polar) / 1e6;
      // Direct Shear Stress: tau_direct = 2 * V / Area (approx for thin tube)
      const tau_shear = (2.0 * ShearForce / Area) / 1e6;
      const tau_shear_total = tau_torsion + tau_shear;

      // Von Mises Equivalent Stress: sigma_vm = sqrt(sigma^2 + 3 * tau^2)
      const sigma_von_mises = Math.sqrt(
        Math.pow(sigma_normal_total, 2) + 3.0 * Math.pow(tau_shear_total, 2)
      );

      // Factor of Safety (Emniyet Katsayısı): FOS = sigma_yield / sigma_von_mises
      const fos = sigma_von_mises > 0.05 ? (sigmaYield / sigma_von_mises) : 999.0;

      // Required minimum wall thickness for target FOS >= 2.5
      // W_req = (sqrt(Mb^2 + 0.75 * T^2)) / (sigma_yield / 2.5)
      const sigma_allowable = (sigmaYield * 1e6) / 2.5;
      const equivalentMoment = Math.sqrt(Mb * Mb + 0.75 * Torsion * Torsion);
      const W_required = equivalentMoment / sigma_allowable;
      // Estimate minimum thickness: W ~ (pi/8) * D^2 * t -> t_min = W / ((pi/8) * D^2)
      const t_min_calc = Math.max(1.5, (W_required / ((Math.PI / 8.0) * D * D)) * 1000.0);

      results.push({
        linkId: i + 1,
        name: dyn.name,
        massKg: dyn.mass,
        torqueNm: Torsion.toFixed(1),
        bendingMomentNm: Mb.toFixed(1),
        shearForceN: ShearForce.toFixed(1),
        outerDiameterMm: dyn.D_outer.toFixed(1),
        wallThicknessMm: dyn.t_wall.toFixed(1),
        bendingStressMPa: sigma_bending.toFixed(2),
        vonMisesStressMPa: sigma_von_mises.toFixed(2),
        factorOfSafety: Math.min(999, fos).toFixed(1),
        isSafe: fos >= 2.0,
        recommendedMinThicknessMm: t_min_calc.toFixed(1),
        materialName: this.material.name,
        materialYieldMPa: sigmaYield
      });
    }

    return results;
  }

  /**
   * Euler-Lagrange Dynamic Formulation Model
   * M(q)*qdd + C(q, qd)*qd + G(q) = tau
   */
  computeEulerLagrange(q = this.q, qDot = [0,0,0,0,0,0], qDoubleDot = [0,0,0,0,0,0]) {
    const n = this.currentDOF || 6;

    // 1. Gravity Vector G(q): Torques when qDot = 0, qDoubleDot = 0
    const dynStatic = this.computeNewtonEulerDynamics(q, new Array(n).fill(0), new Array(n).fill(0), 3.0);
    const G_vector = [...dynStatic.torques];

    // 2. Full Torque tau
    const dynFull = this.computeNewtonEulerDynamics(q, qDot, qDoubleDot, 3.0);
    const tau_vector = [...dynFull.torques];

    // 3. Kinetic Energy K = 0.5 * sum(m_i * v_i^2 + I_i * w_i^2)
    let kineticEnergy = 0.0;
    for (let i = 0; i < n; i++) {
      const qd = qDot[i] || 0.0;
      kineticEnergy += 0.5 * this.linkDynamics[i].Izz * qd * qd;
    }

    // 4. Potential Energy P = sum(m_i * g * h_i)
    let potentialEnergy = 0.0;
    const fk = this.computeFK(q);
    for (let i = 0; i < n; i++) {
      const z_m = fk.cumulative[i + 1][2][3] / 1000.0;
      potentialEnergy += this.linkDynamics[i].mass * 9.81 * z_m;
    }

    const lagrangian = kineticEnergy - potentialEnergy;

    return {
      G_vector: G_vector.map(g => g.toFixed(2)),
      tau_vector: tau_vector.map(t => t.toFixed(2)),
      kineticEnergyJ: kineticEnergy.toFixed(2),
      potentialEnergyJ: potentialEnergy.toFixed(2),
      lagrangianJ: lagrangian.toFixed(2)
    };
  }

  /**
   * 3x3 Matrix-Vector Multiplication
   */
  static matVecMul3(M, v) {
    return [
      M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2],
      M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2],
      M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2]
    ];
  }
}

// Export for browser global or module environment
if (typeof window !== 'undefined') {
  window.RobotKinematics = RobotKinematics;
}
if (typeof module !== 'undefined' && module.exports) {
  module.exports = RobotKinematics;
}

