/**
 * Math Explainer & Educational Inspector using KaTeX
 * 
 * Features:
 * - Live KaTeX rendering of DH Parameter table & Individual Homogeneous Transformation Matrices (A1...A6, T0^6)
 * - 8 Inverse Kinematics Solutions Inspector with configuration switching
 * - Step-by-step mathematical derivation walkthrough for students
 * - Geometric Jacobian 6x6 matrix and Singularity diagnostic indicators
 * - Copyable Lab Assignment / Calculation Report generator
 */

class MathExplainer {
  constructor(kinematicsEngine, onSelectSolutionCallback) {
    this.kinematics = kinematicsEngine;
    this.onSelectSolution = onSelectSolutionCallback;
    this.activeMatrixTab = 'all'; // 'all' or 'T6' or 'A1'...'A6'
  }

  /**
   * Render KaTeX formula safely into a DOM element
   */
  renderLatex(elementId, latexCode, displayMode = true) {
    const el = document.getElementById(elementId);
    if (!el) return;
    if (typeof katex !== 'undefined') {
      try {
        katex.render(latexCode, el, {
          displayMode: displayMode,
          throwOnError: false
        });
      } catch (err) {
        el.textContent = latexCode;
      }
    } else {
      el.textContent = latexCode;
    }
  }

  /**
   * Format a 4x4 matrix into LaTeX bmatrix string
   */
  formatMatrixLatex(matrix, decimals = 3) {
    const rows = matrix.map(row => 
      row.map(val => {
        const rounded = Math.abs(val) < 1e-4 ? 0 : val;
        return rounded.toFixed(decimals);
      }).join(' & ')
    );
    return `\\begin{bmatrix} ${rows.join(' \\\\ ')} \\end{bmatrix}`;
  }

  /**
   * Format a 6x6 Jacobian matrix into compact LaTeX bmatrix string
   */
  formatJacobianLatex(J, decimals = 2) {
    const rows = J.map(row => 
      row.map(val => {
        const rounded = Math.abs(val) < 1e-4 ? 0 : val;
        return rounded.toFixed(decimals);
      }).join(' & ')
    );
    return `\\begin{bmatrix} ${rows.join(' \\\\ ')} \\end{bmatrix}`;
  }

  /**
   * Update the DH Parameters Table in UI
   */
  updateDHTable(q = this.kinematics.q) {
    const tbody = document.getElementById('dh-table-body');
    if (!tbody) return;

    const dh = this.kinematics.dh;
    let html = '';

    for (let i = 0; i < 6; i++) {
      const p = dh[i];
      const qRad = q[i];
      const qDeg = (qRad * 180 / Math.PI).toFixed(1);
      const alphaDeg = (p.alpha * 180 / Math.PI).toFixed(0);

      html += `
        <tr>
          <td><span class="joint-badge j${i+1}">${i+1}</span></td>
          <td><strong>${p.name}</strong></td>
          <td class="math-num text-cyan">${qDeg}° <span class="sub-rad">(${qRad.toFixed(2)} rad)</span></td>
          <td class="math-num">${p.d.toFixed(1)}</td>
          <td class="math-num">${p.a.toFixed(1)}</td>
          <td class="math-num">${alphaDeg}°</td>
          <td class="text-muted text-xs">[${p.minDeg}°, ${p.maxDeg}°]</td>
        </tr>
      `;
    }

    tbody.innerHTML = html;
  }

  /**
   * Update Transformation Matrices Display
   */
  updateTransformationMatrices(fkResult) {
    const container = document.getElementById('matrices-container');
    if (!container) return;

    const ind = fkResult.individual;
    const T6 = fkResult.cumulative[6];

    // Master T0^6 formula
    const t6Latex = `T_0^6 = A_1 A_2 A_3 A_4 A_5 A_6 = ${this.formatMatrixLatex(T6, 3)}`;
    this.renderLatex('matrix-t06-katex', t6Latex, true);

    // Individual Matrices Accordion / Cards
    let html = '';
    for (let i = 0; i < 6; i++) {
      const Ai = ind[i];
      const name = this.kinematics.dh[i].name;
      html += `
        <div class="matrix-card">
          <div class="matrix-card-header">
            <span class="matrix-card-title">A_{${i+1}} (${name})</span>
            <span class="matrix-card-meta">Çerçeve ${i} → ${i+1}</span>
          </div>
          <div class="matrix-card-body" id="matrix-a${i+1}-katex"></div>
        </div>
      `;
    }
    container.innerHTML = html;

    // Render KaTeX for each individual matrix
    for (let i = 0; i < 6; i++) {
      const AiLatex = `A_${i+1} = ${this.formatMatrixLatex(ind[i], 3)}`;
      this.renderLatex(`matrix-a${i+1}-katex`, AiLatex, true);
    }
  }

  /**
   * Update Inverse Kinematics 8-Solutions Table
   */
  updateIKSolutionsTable(ikResult, selectedIndex = 0) {
    const tbody = document.getElementById('ik-solutions-body');
    if (!tbody) return;

    const solutions = ikResult.solutions;
    let html = '';

    solutions.forEach((sol, idx) => {
      const isSelected = (idx === selectedIndex);
      const isWithin = sol.withinLimits;
      const statusBadge = isWithin 
        ? `<span class="badge badge-success">✓ Geçerli</span>` 
        : `<span class="badge badge-danger">✗ Limit Aşıldı</span>`;

      const qDegFormatted = sol.qDeg.map(d => `${d >= 0 ? '+' : ''}${d.toFixed(1)}°`).join(', ');

      html += `
        <tr class="sol-row ${isSelected ? 'selected-sol' : ''} ${!isWithin ? 'row-invalid' : ''}" data-sol-index="${idx}">
          <td>
            <input type="radio" name="ik-sol-radio" value="${idx}" ${isSelected ? 'checked' : ''} class="sol-radio">
            <span class="sol-id">#${idx + 1}</span>
          </td>
          <td>
            <div class="sol-branch-title">${sol.label}</div>
            <div class="sol-branch-sub">Açısal Mesafe: ${sol.distance.toFixed(2)} rad</div>
          </td>
          <td>${statusBadge}</td>
          <td class="font-mono text-xs">${qDegFormatted}</td>
          <td>
            <button class="btn-micro ${isSelected ? 'btn-active' : ''}" onclick="window.app.applyIKSolution(${idx})">
              ${isSelected ? 'Aktif' : 'Seç'}
            </button>
          </td>
        </tr>
      `;
    });

    tbody.innerHTML = html;

    // Add row click listeners
    const rows = tbody.querySelectorAll('.sol-row');
    rows.forEach(r => {
      r.addEventListener('click', (e) => {
        if (e.target.tagName !== 'BUTTON' && e.target.tagName !== 'INPUT') {
          const idx = parseInt(r.getAttribute('data-sol-index'), 10);
          if (this.onSelectSolution) this.onSelectSolution(idx);
        }
      });
    });
  }

  /**
   * Update Step-by-Step Educational Kinematics Breakdown
   */
  updateEducationalSteps(ikResult) {
    const container = document.getElementById('educational-steps-container');
    if (!container) return;

    let html = '';
    ikResult.steps.forEach(st => {
      html += `
        <div class="edu-step-card">
          <div class="edu-step-num">${st.step}</div>
          <div class="edu-step-content">
            <h4 class="edu-step-title">${st.title}</h4>
            <p class="edu-step-desc">${st.description}</p>
            <div class="edu-step-formula" id="step-formula-${st.step}"></div>
            <div class="edu-step-values">
              ${Object.entries(st.values).map(([k, v]) => `
                <div class="val-item">
                  <span class="val-label">${k}:</span>
                  <span class="val-num">${v}</span>
                </div>
              `).join('')}
            </div>
          </div>
        </div>
      `;
    });

    container.innerHTML = html;

    // Render KaTeX for formulas
    ikResult.steps.forEach(st => {
      this.renderLatex(`step-formula-${st.step}`, st.formula, true);
    });
  }

  /**
   * Update Jacobian & Singularity Dashboard
   */
  updateJacobianAndSingularities(jacobianResult) {
    const { J, det, manipulability, singularities, dof, rowLabels } = jacobianResult;
    const n = dof || 6;

    // Render Active Jacobian Matrix with dimension label
    const dimText = `${n} \\times ${n}`;
    const jLatex = `J_{${n} \\times ${n}}(q) = ${this.formatJacobianLatex(J, 2)}`;
    this.renderLatex('jacobian-matrix-katex', jLatex, true);

    const titleEl = document.getElementById('jacobian-title-dim');
    if (titleEl) titleEl.textContent = `[${dimText} Matris - ${n} Serbestlik Derecesi]`;

    // Determinant & Manipulability values
    const detEl = document.getElementById('jac-det-val');
    const manipEl = document.getElementById('jac-manip-val');
    const manipProgress = document.getElementById('manip-progress-bar');

    if (detEl) detEl.textContent = det.toFixed(5);
    if (manipEl) manipEl.textContent = manipulability.toFixed(5);

    if (manipProgress) {
      // Manipulability scale (0 to 0.1 standard range)
      const percent = Math.min(100, Math.max(0, (manipulability / 0.08) * 100));
      manipProgress.style.width = `${percent}%`;

      if (manipulability < 0.005) {
        manipProgress.className = 'progress-bar bg-danger';
      } else if (manipulability < 0.02) {
        manipProgress.className = 'progress-bar bg-warning';
      } else {
        manipProgress.className = 'progress-bar bg-success';
      }
    }

    // Singularity Diagnostic Badges
    const badgeWrist = document.getElementById('sing-badge-wrist');
    const badgeShoulder = document.getElementById('sing-badge-shoulder');
    const badgeElbow = document.getElementById('sing-badge-elbow');
    const overallBanner = document.getElementById('singularity-alert-banner');

    if (badgeWrist) {
      badgeWrist.className = `diag-badge ${singularities.wrist ? 'diag-alert' : 'diag-ok'}`;
      badgeWrist.innerHTML = singularities.wrist 
        ? '<strong>UYARI:</strong> Bilek Tekilliği Aktif (θ5 ≈ 0°). Eksen 4 ve 6 çakıştı!'
        : '✓ Bilek Normal (θ5 eksenleri bağımsız)';
    }

    if (badgeShoulder) {
      badgeShoulder.className = `diag-badge ${singularities.shoulder ? 'diag-alert' : 'diag-ok'}`;
      badgeShoulder.innerHTML = singularities.shoulder 
        ? '<strong>UYARI:</strong> Omuz Tekilliği Aktif (Bilek merkezi ana eksen üzerinde)!'
        : '✓ Omuz Normal (Bilek merkezi ofsetli)';
    }

    if (badgeElbow) {
      badgeElbow.className = `diag-badge ${singularities.elbow ? 'diag-alert' : 'diag-ok'}`;
      badgeElbow.innerHTML = singularities.elbow 
        ? '<strong>UYARI:</strong> Dirsek Tekilliği / Sınır (Kol tam açık veya kapalı)!'
        : '✓ Dirsek Normal (Çalışma zarfı içinde)';
    }

    if (overallBanner) {
      if (singularities.isNearSingular) {
        overallBanner.style.display = 'flex';
      } else {
        overallBanner.style.display = 'none';
      }
    }
  }

  /**
   * Generate Full Markdown / Text Calculation Report for Homework/Assignments
   */
  generateCalculationReport(qA, qB, poseA, poseB, trajectoryData) {
    const dh = this.kinematics.dh;
    const fkA = this.kinematics.computeFK(qA);
    const fkB = this.kinematics.computeFK(qB);
    const jacA = this.kinematics.computeJacobian(qA);
    const jacB = this.kinematics.computeJacobian(qB);

    const report = `# 6-EKSENLİ ROBOT KOLU KİNEMATİK VE YÖRÜNGE HESAPLAMA RAPORU
Tarih: ${new Date().toLocaleString('tr-TR')}
Öğrenci / Eğitim Amaçlı Analiz Çıktısı

=======================================================
1. ROBOT GEOMETRİSİ VE DENAVIT-HARTENBERG (DH) PARAMETRELERİ
=======================================================
Link i | Eklem Adı         | d_i (mm) | a_i (mm) | alpha_i (rad) | Eklem Limitleri
-----------------------------------------------------------------------------------
1      | Base (Kök)        | ${dh[0].d.toFixed(1).padEnd(8)} | ${dh[0].a.toFixed(1).padEnd(8)} | ${(dh[0].alpha*180/Math.PI).toFixed(0)}° (${dh[0].alpha.toFixed(2)})   | [${dh[0].minDeg}°, ${dh[0].maxDeg}°]
2      | Shoulder (Omuz)   | ${dh[1].d.toFixed(1).padEnd(8)} | ${dh[1].a.toFixed(1).padEnd(8)} | ${(dh[1].alpha*180/Math.PI).toFixed(0)}° (${dh[1].alpha.toFixed(2)})   | [${dh[1].minDeg}°, ${dh[1].maxDeg}°]
3      | Elbow (Dirsek)     | ${dh[2].d.toFixed(1).padEnd(8)} | ${dh[2].a.toFixed(1).padEnd(8)} | ${(dh[2].alpha*180/Math.PI).toFixed(0)}° (${dh[2].alpha.toFixed(2)})   | [${dh[2].minDeg}°, ${dh[2].maxDeg}°]
4      | Wrist 1 (Bilek R) | ${dh[3].d.toFixed(1).padEnd(8)} | ${dh[3].a.toFixed(1).padEnd(8)} | ${(dh[3].alpha*180/Math.PI).toFixed(0)}° (${dh[3].alpha.toFixed(2)})  | [${dh[3].minDeg}°, ${dh[3].maxDeg}°]
5      | Wrist 2 (Bilek P) | ${dh[4].d.toFixed(1).padEnd(8)} | ${dh[4].a.toFixed(1).padEnd(8)} | ${(dh[4].alpha*180/Math.PI).toFixed(0)}° (${dh[4].alpha.toFixed(2)})   | [${dh[4].minDeg}°, ${dh[4].maxDeg}°]
6      | Wrist 3 (Flanş/T) | ${dh[5].d.toFixed(1).padEnd(8)} | ${dh[5].a.toFixed(1).padEnd(8)} | ${(dh[5].alpha*180/Math.PI).toFixed(0)}° (${dh[5].alpha.toFixed(2)})   | [${dh[5].minDeg}°, ${dh[5].maxDeg}°]

=======================================================
2. BAŞLANGIÇ KONUMU: NOKTA A
=======================================================
Kartezyen Koordinatlar:
X = ${poseA.x.toFixed(2)} mm
Y = ${poseA.y.toFixed(2)} mm
Z = ${poseA.z.toFixed(2)} mm
Oryantasyon (Roll, Pitch, Yaw):
Roll  (Rx) = ${poseA.rollDeg.toFixed(2)}°
Pitch (Ry) = ${poseA.pitchDeg.toFixed(2)}°
Yaw   (Rz) = ${poseA.yawDeg.toFixed(2)}°

Çözülen Eklem Açıları qA:
q1 = ${(qA[0]*180/Math.PI).toFixed(2)}° (${qA[0].toFixed(4)} rad)
q2 = ${(qA[1]*180/Math.PI).toFixed(2)}° (${qA[1].toFixed(4)} rad)
q3 = ${(qA[2]*180/Math.PI).toFixed(2)}° (${qA[2].toFixed(4)} rad)
q4 = ${(qA[3]*180/Math.PI).toFixed(2)}° (${qA[3].toFixed(4)} rad)
q5 = ${(qA[4]*180/Math.PI).toFixed(2)}° (${qA[4].toFixed(4)} rad)
q6 = ${(qA[5]*180/Math.PI).toFixed(2)}° (${qA[5].toFixed(4)} rad)

Jakoben Determinantı det(J_A) = ${jacA.det.toFixed(6)}
Yoshikawa Manipülatiflik w_A   = ${jacA.manipulability.toFixed(6)}

=======================================================
3. HEDEF KONUM: NOKTA B
=======================================================
Kartezyen Koordinatlar:
X = ${poseB.x.toFixed(2)} mm
Y = ${poseB.y.toFixed(2)} mm
Z = ${poseB.z.toFixed(2)} mm
Oryantasyon (Roll, Pitch, Yaw):
Roll  (Rx) = ${poseB.rollDeg.toFixed(2)}°
Pitch (Ry) = ${poseB.pitchDeg.toFixed(2)}°
Yaw   (Rz) = ${poseB.yawDeg.toFixed(2)}°

Çözülen Eklem Açıları qB:
q1 = ${(qB[0]*180/Math.PI).toFixed(2)}° (${qB[0].toFixed(4)} rad)
q2 = ${(qB[1]*180/Math.PI).toFixed(2)}° (${qB[1].toFixed(4)} rad)
q3 = ${(qB[2]*180/Math.PI).toFixed(2)}° (${qB[2].toFixed(4)} rad)
q4 = ${(qB[3]*180/Math.PI).toFixed(2)}° (${qB[3].toFixed(4)} rad)
q5 = ${(qB[4]*180/Math.PI).toFixed(2)}° (${qB[4].toFixed(4)} rad)
q6 = ${(qB[5]*180/Math.PI).toFixed(2)}° (${qB[5].toFixed(4)} rad)

Jakoben Determinantı det(J_B) = ${jacB.det.toFixed(6)}
Yoshikawa Manipülatiflik w_B   = ${jacB.manipulability.toFixed(6)}

=======================================================
4. YÖRÜNGE PLANLAMA PARAMETRELERİ (A -> B HAREKETİ)
=======================================================
İnterpolasyon Yöntemi: ${trajectoryData.mode}
Toplam Hareket Süresi (T): ${trajectoryData.duration.toFixed(2)} saniye
Örnekleme Adım Sayısı: ${trajectoryData.stepsCount}

Polinom Denklemi (Eklem Başına):
s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5  (tau = t / T)
q_i(t) = q_i,A + (q_i,B - q_i,A) * s(tau)

Sonuç: Hareket sarsıntısız (jerk-free), başlangıç ve bitiş ivmeleri 0 olacak şekilde başarıyla hesaplanmıştır.
`;

    return report;
  }

  /**
   * Update Dynamics & Structural Strength Analysis (Newton-Euler, Euler-Lagrange, Mukavemet)
   */
  updateDynamicsAndStrength(dynResult, eulerLagrangeResult) {
    if (!dynResult) return;

    // 1. Update Torques Table
    const torqueBody = document.getElementById('dynamics-torque-body');
    if (torqueBody) {
      const torques = dynResult.torques;
      let html = '';
      for (let i = 0; i < torques.length; i++) {
        const tau = torques[i];
        const absTau = Math.abs(tau);
        const percent = Math.min(100, (absTau / 250.0) * 100);
        const barColor = absTau > 180 ? 'bg-danger' : absTau > 90 ? 'bg-warning' : 'bg-success';

        html += `
          <tr>
            <td><span class="joint-badge j${i+1}">${i+1}</span></td>
            <td><strong>Eklem ${i+1} Torku (τ${i+1})</strong></td>
            <td class="font-mono ${tau >= 0 ? 'text-cyan' : 'text-amber'}">${tau >= 0 ? '+' : ''}${tau.toFixed(1)} Nm</td>
            <td>
              <div class="progress-container" style="width: 110px; margin: 0;">
                <div class="progress-bar ${barColor}" style="width: ${percent}%;"></div>
              </div>
            </td>
          </tr>
        `;
      }
      torqueBody.innerHTML = html;
    }

    // 2. Render Euler-Lagrange Formulas
    if (eulerLagrangeResult) {
      const elLatex = `M(q)\\ddot{q} + C(q, \\dot{q})\\dot{q} + G(q) = \\tau`;
      this.renderLatex('lagrange-eq-katex', elLatex, true);

      const kineticEl = document.getElementById('val-kinetic-energy');
      const potentialEl = document.getElementById('val-potential-energy');
      const lagrangianEl = document.getElementById('val-lagrangian');

      if (kineticEl) kineticEl.textContent = `${eulerLagrangeResult.kineticEnergyJ} J`;
      if (potentialEl) potentialEl.textContent = `${eulerLagrangeResult.potentialEnergyJ} J`;
      if (lagrangianEl) lagrangianEl.textContent = `${eulerLagrangeResult.lagrangianJ} J`;
    }

    // 3. Update Structural Stress & Strength (Mukavemet) Table
    const stressBody = document.getElementById('structural-stress-body');
    if (stressBody && dynResult.structuralAnalysis) {
      let html = '';
      dynResult.structuralAnalysis.forEach(item => {
        const isSafe = item.isSafe;
        const statusBadge = isSafe
          ? `<span class="badge badge-success">✓ Emniyetli (FOS: ${item.factorOfSafety})</span>`
          : `<span class="badge badge-danger">⚠️ Kritik (FOS: ${item.factorOfSafety})</span>`;

        html += `
          <tr>
            <td><strong>${item.name}</strong></td>
            <td class="font-mono">${item.bendingMomentNm} Nm</td>
            <td class="font-mono">Ø${item.outerDiameterMm} / t=${item.wallThicknessMm} mm</td>
            <td class="font-mono text-amber">${item.vonMisesStressMPa} MPa</td>
            <td>${statusBadge}</td>
            <td class="font-mono text-cyan">≥ ${item.recommendedMinThicknessMm} mm</td>
          </tr>
        `;
      });
      stressBody.innerHTML = html;
    }
  }

  /**
   * Update Multi-DOF Comparison Section (3, 4, 5, 6 DOF)
   */
  updateDOFComparison(dofComparisonData, activeDOF = 6) {
    const container = document.getElementById('dof-comparison-container');
    if (!container) return;

    let html = '';
    const dofs = [3, 4, 5, 6];

    dofs.forEach(d => {
      const item = dofComparisonData[d];
      const isActive = (d === parseInt(activeDOF, 10));

      html += `
        <div class="glass-card ${isActive ? 'dof-card-active' : ''}" style="margin-bottom: 10px; border-left: 4px solid ${isActive ? 'var(--accent-cyan)' : 'var(--border-subtle)'};">
          <div class="card-title-bar">
            <span class="card-title ${isActive ? 'cyan' : ''}">
              ${isActive ? '★ [AKTİF ROBOT] ' : ''}${item.name}
            </span>
            <span class="badge ${isActive ? 'badge-success' : 'badge-secondary'}">${d} Eksen</span>
          </div>
          <div style="font-size: 11.5px; display: flex; flex-direction: column; gap: 6px;">
            <div><strong style="color: var(--text-dim);">Kontrol Edilen Uzay:</strong> <span class="text-cyan">${item.taskSpace}</span></div>
            <div><strong style="color: var(--text-dim);">Kilitli / Eksik Yönler:</strong> <span class="text-amber">${item.uncontrolled}</span></div>
            <div><strong style="color: var(--text-dim);">Jakoben Matrisi:</strong> <code>${item.jacobianDim}</code></div>
            <div><strong style="color: var(--text-dim);">6-DOF'tan Farkı:</strong> ${item.differenceFrom6DOF}</div>
            <div><strong style="color: var(--text-dim);">Tipik Endüstriyel Görevler:</strong> ${item.applications}</div>
          </div>
        </div>
      `;
    });

    container.innerHTML = html;
  }
}

// Global export
if (typeof window !== 'undefined') {
  window.MathExplainer = MathExplainer;
}
