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
      const thetaTotal = q[i] + (p.thetaOffset || 0);
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
  computeJacobian(q = this.q) {
    const fk = this.computeFK(q);
    const T = fk.cumulative;
    const pe = [T[6][0][3], T[6][1][3], T[6][2][3]]; // Tool Center Point (TCP)

    const J = [
      [0, 0, 0, 0, 0, 0], // v_x
      [0, 0, 0, 0, 0, 0], // v_y
      [0, 0, 0, 0, 0, 0], // v_z
      [0, 0, 0, 0, 0, 0], // w_x
      [0, 0, 0, 0, 0, 0], // w_y
      [0, 0, 0, 0, 0, 0]  // w_z
    ];

    for (let i = 0; i < 6; i++) {
      const Ti = T[i]; // T_{i-1} in 0-indexed terms
      const pi_prev = [Ti[0][3], Ti[1][3], Ti[2][3]];
      const zi_prev = [Ti[0][2], Ti[1][2], Ti[2][2]]; // 3rd column = z-axis

      // r = pe - pi_prev (convert mm to meters for conditioning if needed, or keep in mm)
      // Note: To have standard physical units, convert positions to meters:
      const r = [
        (pe[0] - pi_prev[0]) / 1000.0,
        (pe[1] - pi_prev[1]) / 1000.0,
        (pe[2] - pi_prev[2]) / 1000.0
      ];

      const Jv = RobotKinematics.cross(zi_prev, r);
      const Jw = zi_prev;

      J[0][i] = Jv[0];
      J[1][i] = Jv[1];
      J[2][i] = Jv[2];
      J[3][i] = Jw[0];
      J[4][i] = Jw[1];
      J[5][i] = Jw[2];
    }

    // Determinant calculation via Gaussian Elimination
    const det = RobotKinematics.matrixDet6(J);
    const manipulability = Math.abs(det);

    // Singularity diagnostic checks:
    // 1. Wrist singularity: sin(theta5) ~ 0
    const wristSingular = Math.abs(Math.sin(q[4])) < 0.05;

    // 2. Shoulder singularity: Wrist center lies on base z-axis (r ~ 0)
    const T4 = T[4];
    const pw = [T4[0][3], T4[1][3], T4[2][3]];
    const rw = Math.sqrt(pw[0] * pw[0] + pw[1] * pw[1]);
    const shoulderSingular = rw < 30.0; // within 30 mm of base axis

    // 3. Elbow singularity: Arm fully stretched or folded
    const a2 = this.dh[1].a;
    const d4 = this.dh[3].d;
    const sShoulder = pw[2] - this.dh[0].d;
    const distArm = Math.sqrt(rw * rw + sShoulder * sShoulder);
    const elbowSingular = Math.abs(distArm - (a2 + d4)) < 25.0 || Math.abs(distArm - Math.abs(a2 - d4)) < 25.0;

    return {
      J,
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
}

// Export for browser global or module environment
if (typeof window !== 'undefined') {
  window.RobotKinematics = RobotKinematics;
}
if (typeof module !== 'undefined' && module.exports) {
  module.exports = RobotKinematics;
}
