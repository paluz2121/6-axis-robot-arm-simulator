/**
 * Main Application Coordinator & UI Controller
 */

class RobotEducationalApp {
  constructor() {
    this.kinematics = new RobotKinematics();
    this.visualizer = null;
    this.chartsManager = null;
    this.mathExplainer = null;

    // Simulation State
    this.sim = {
      isPlaying: false,
      currentTime: 0.0,
      duration: 3.0,
      speedMultiplier: 1.0,
      mode: 'joint-quintic', // 'joint-quintic', 'joint-cubic', 'joint-trapezoidal', 'cartesian-linear'
      trajectory: null,
      lastFrameTime: performance.now()
    };

    // Waypoint A (Start) Defaults
    this.poseA = {
      x: 450.0,
      y: -250.0,
      z: 220.0,
      roll: 0.0,
      pitch: Math.PI / 2, // 90 deg down
      yaw: 0.0,
      rollDeg: 0.0,
      pitchDeg: 90.0,
      yawDeg: 0.0
    };

    // Waypoint B (Target) Defaults
    this.poseB = {
      x: 420.0,
      y: 300.0,
      z: 380.0,
      roll: 0.0,
      pitch: Math.PI / 3, // 60 deg
      yaw: Math.PI / 4,   // 45 deg
      rollDeg: 0.0,
      pitchDeg: 60.0,
      yawDeg: 45.0
    };

    this.qA = [0, 0, 0, 0, 0, 0];
    this.qB = [0, 0, 0, 0, 0, 0];
    this.currentQ = [0, 0, 0, 0, 0, 0];

    // Presets
    this.presets = {
      'pick-place': {
        name: 'Standart Parça Taşıma (Pick & Place)',
        desc: 'Masa üzerinden parça alıp kutuya aktarma senaryosu.',
        poseA: { x: 420, y: -260, z: 180, rollDeg: 0, pitchDeg: 90, yawDeg: 0 },
        poseB: { x: 380, y: 320, z: 320, rollDeg: 0, pitchDeg: 60, yawDeg: 45 },
        mode: 'joint-quintic',
        duration: 3.0
      },
      'linear-weld': {
        name: 'Doğrusal Kaynak Yolu (Linear Seam)',
        desc: 'İki nokta arasında kesin düz çizgi takibi. Kartezyen ve Eklem uzayı farkını gösterir.',
        poseA: { x: 350, y: -250, z: 250, rollDeg: 0, pitchDeg: 90, yawDeg: 0 },
        poseB: { x: 350, y: 250, z: 250, rollDeg: 0, pitchDeg: 90, yawDeg: 0 },
        mode: 'cartesian-linear',
        duration: 4.0
      },
      'wrist-singularity': {
        name: 'Bilek Tekilliği Örneği (Wrist Singularity: θ5 ≈ 0°)',
        desc: 'Eklem 4 ve Eklem 6 eksenlerinin çakıştığı tekillik bölgesi. Manipülatiflik sıfıra yaklaşır.',
        poseA: { x: 480, y: 0, z: 420, rollDeg: 0, pitchDeg: 0, yawDeg: 0 },
        poseB: { x: 480, y: 0, z: 580, rollDeg: 0, pitchDeg: 0, yawDeg: 0 },
        mode: 'joint-quintic',
        duration: 3.5
      },
      'shoulder-singularity': {
        name: 'Omuz Tekilliği Örneği (Shoulder Singularity: r ≈ 0)',
        desc: 'Bilek merkezinin robotun ana dik ekseni (Z0) üzerinden geçtiği durum.',
        poseA: { x: 80, y: -150, z: 550, rollDeg: 0, pitchDeg: 45, yawDeg: 0 },
        poseB: { x: 80, y: 150, z: 550, rollDeg: 0, pitchDeg: 45, yawDeg: 0 },
        mode: 'joint-cubic',
        duration: 3.0
      },
      'workspace-boundary': {
        name: 'Erişim Sınırı (Dirsek Tekilliği)',
        desc: 'Robot kolunun tam açıldığı çalışma zarfı sınırı.',
        poseA: { x: 400, y: 0, z: 400, rollDeg: 0, pitchDeg: 45, yawDeg: 0 },
        poseB: { x: 740, y: 0, z: 350, rollDeg: 0, pitchDeg: 0, yawDeg: 0 },
        mode: 'joint-quintic',
        duration: 3.5
      }
    };
  }

  init() {
    // 1. Initialize Visualizer
    const canvasContainer = document.getElementById('robot-3d-viewport');
    this.visualizer = new Robot3DVisualizer(canvasContainer, this.kinematics);

    // 2. Initialize Charts
    this.chartsManager = new TrajectoryChartsManager((seekTime) => {
      this.seekToTime(seekTime);
    });

    // 3. Initialize Math Explainer
    this.mathExplainer = new MathExplainer(this.kinematics, (solIndex) => {
      this.applyIKSolution(solIndex);
    });

    // 3.5. Initialize Code Generator (Arduino C++ & MATLAB)
    if (typeof CodeGenerator !== 'undefined') {
      this.codeGenerator = new CodeGenerator(this);
      this.codeGenerator.init();
    }

    // 4. Initial calculations for A and B
    this.syncPosesFromInputs();
    this.solveInitialPoses();
    this.recalculateTrajectory();

    // 5. Setup UI Listeners & Presets
    this.setupUIEventListeners();
    this.setupTabNavigation();

    // 6. Start animation loop
    this.simLoop = this.simLoop.bind(this);
    requestAnimationFrame(this.simLoop);
  }

  syncPosesFromInputs() {
    // Read Pose A
    this.poseA.x = parseFloat(document.getElementById('input-ax').value) || 450;
    this.poseA.y = parseFloat(document.getElementById('input-ay').value) || -250;
    this.poseA.z = parseFloat(document.getElementById('input-az').value) || 220;
    this.poseA.rollDeg = parseFloat(document.getElementById('input-aroll').value) || 0;
    this.poseA.pitchDeg = parseFloat(document.getElementById('input-apitch').value) || 90;
    this.poseA.yawDeg = parseFloat(document.getElementById('input-ayaw').value) || 0;
    this.poseA.roll = this.poseA.rollDeg * Math.PI / 180;
    this.poseA.pitch = this.poseA.pitchDeg * Math.PI / 180;
    this.poseA.yaw = this.poseA.yawDeg * Math.PI / 180;

    // Read Pose B
    this.poseB.x = parseFloat(document.getElementById('input-bx').value) || 420;
    this.poseB.y = parseFloat(document.getElementById('input-by').value) || 300;
    this.poseB.z = parseFloat(document.getElementById('input-bz').value) || 380;
    this.poseB.rollDeg = parseFloat(document.getElementById('input-broll').value) || 0;
    this.poseB.pitchDeg = parseFloat(document.getElementById('input-bpitch').value) || 60;
    this.poseB.yawDeg = parseFloat(document.getElementById('input-byaw').value) || 45;
    this.poseB.roll = this.poseB.rollDeg * Math.PI / 180;
    this.poseB.pitch = this.poseB.pitchDeg * Math.PI / 180;
    this.poseB.yaw = this.poseB.yawDeg * Math.PI / 180;

    // Simulation params
    this.sim.duration = parseFloat(document.getElementById('input-duration').value) || 3.0;
    this.sim.mode = document.getElementById('select-trajectory-mode').value || 'joint-quintic';
  }

  solveInitialPoses() {
    // Solve IK for A
    const ikA = this.kinematics.computeIK(this.poseA, this.currentQ);
    this.qA = [...ikA.bestSolution.q];
    this.visualizer.setWaypointAPose(this.poseA, this.qA);

    // Solve IK for B
    const ikB = this.kinematics.computeIK(this.poseB, this.qA);
    this.qB = [...ikB.bestSolution.q];
    this.visualizer.setWaypointBPose(this.poseB, this.qB);

    // Start at Point A
    this.currentQ = [...this.qA];
    this.updateRobotState(this.currentQ);

    // Update Educational UI with Point A details
    this.mathExplainer.updateIKSolutionsTable(ikA, ikA.bestIndex);
    this.mathExplainer.updateEducationalSteps(ikA);
  }

  recalculateTrajectory() {
    this.syncPosesFromInputs();

    // Re-solve IK for A and B
    const ikA = this.kinematics.computeIK(this.poseA, this.currentQ);
    this.qA = [...ikA.bestSolution.q];
    this.visualizer.setWaypointAPose(this.poseA, this.qA);

    const ikB = this.kinematics.computeIK(this.poseB, this.qA);
    this.qB = [...ikB.bestSolution.q];
    this.visualizer.setWaypointBPose(this.poseB, this.qB);

    // Generate full trajectory
    this.sim.trajectory = this.kinematics.generateTrajectory(
      this.qA,
      this.qB,
      this.poseA,
      this.poseB,
      this.sim.duration,
      120,
      this.sim.mode
    );

    // Render 3D Trajectory Trail
    this.visualizer.renderTrajectoryCurve(this.sim.trajectory);

    // Update charts
    this.chartsManager.updateTrajectory(this.sim.trajectory);

    // Reset simulation time to start
    this.seekToTime(0.0);

    // Update live Arduino & MATLAB code export
    if (this.codeGenerator) {
      this.codeGenerator.updateCodeDisplay();
    }
  }

  updateRobotState(q) {
    this.currentQ = [...q];
    this.kinematics.q = [...q];

    // 1. Update 3D scene
    const fk = this.visualizer.updateRobotPose(q);

    // 2. Update Joint Sliders & angle readouts
    for (let i = 0; i < 6; i++) {
      const deg = q[i] * 180 / Math.PI;
      const slider = document.getElementById(`slider-q${i+1}`);
      const valText = document.getElementById(`val-q${i+1}`);
      if (slider) slider.value = deg.toFixed(1);
      if (valText) valText.textContent = `${deg >= 0 ? '+' : ''}${deg.toFixed(1)}°`;
    }

    // 3. Update HUD overlays
    const hudX = document.getElementById('hud-tcp-x');
    const hudY = document.getElementById('hud-tcp-y');
    const hudZ = document.getElementById('hud-tcp-z');
    const hudManip = document.getElementById('hud-manip-val');

    if (hudX) hudX.textContent = fk.tcp.x.toFixed(1);
    if (hudY) hudY.textContent = fk.tcp.y.toFixed(1);
    if (hudZ) hudZ.textContent = fk.tcp.z.toFixed(1);

    // 4. Compute Jacobian
    const jac = this.kinematics.computeJacobian(q);
    if (hudManip) hudManip.textContent = jac.manipulability.toFixed(4);

    // 5. Update Math inspector
    this.mathExplainer.updateDHTable(q);
    this.mathExplainer.updateTransformationMatrices(fk);
    this.mathExplainer.updateJacobianAndSingularities(jac);

    // 6. Compute Dynamics (Recursive Newton-Euler) & Structural Strength (Mukavemet)
    const dyn = this.kinematics.computeNewtonEulerDynamics(q, [0,0,0,0,0,0], [0,0,0,0,0,0], 3.0);
    const el = this.kinematics.computeEulerLagrange(q, [0,0,0,0,0,0], [0,0,0,0,0,0]);
    this.mathExplainer.updateDynamicsAndStrength(dyn, el);
    this.mathExplainer.updateDOFComparison(this.kinematics.getDOFComparison(), this.kinematics.currentDOF);
  }

  /**
   * Reset Robot to Home Pose (All joint angles 0.0°)
   */
  resetToHomePose() {
    this.pause();
    this.currentQ = [0, 0, 0, 0, 0, 0];
    this.kinematics.q = [0, 0, 0, 0, 0, 0];
    this.updateRobotState(this.currentQ);

    const fk = this.kinematics.computeFK(this.currentQ);
    // Set Waypoint A inputs to Home Pose
    document.getElementById('input-ax').value = fk.tcp.x.toFixed(1);
    document.getElementById('input-ay').value = fk.tcp.y.toFixed(1);
    document.getElementById('input-az').value = fk.tcp.z.toFixed(1);
    document.getElementById('input-aroll').value = fk.tcp.rollDeg.toFixed(1);
    document.getElementById('input-apitch').value = fk.tcp.pitchDeg.toFixed(1);
    document.getElementById('input-ayaw').value = fk.tcp.yawDeg.toFixed(1);

    this.recalculateTrajectory();

    const banner = document.getElementById('preset-info-banner');
    if (banner) {
      banner.textContent = "🏠 Robot Sıfırlandı: Tüm Eklem Açıları 0.0° (Home Pose)";
      banner.style.display = 'block';
      setTimeout(() => { banner.style.display = 'none'; }, 3000);
    }
  }

  /**
   * Set Active Degrees of Freedom (3, 4, 5, or 6 DOF)
   */
  setDOF(dof) {
    const activeDOF = this.kinematics.setDOF(dof);
    this.visualizer.setDOF(activeDOF);

    // Adapt joint sliders UI
    for (let i = 1; i <= 6; i++) {
      const slider = document.getElementById(`slider-q${i}`);
      const row = slider ? slider.closest('.joint-slider-row') : null;
      if (row) {
        if (i <= activeDOF) {
          row.style.opacity = '1.0';
          slider.disabled = false;
        } else {
          row.style.opacity = '0.35';
          slider.disabled = true;
          slider.value = 0;
          const valText = document.getElementById(`val-q${i}`);
          if (valText) valText.textContent = '0.0° (Kilitli)';
        }
      }
    }

    this.updateRobotState(this.currentQ);
    this.recalculateTrajectory();

    // Auto-switch to DOF Compare tab so the user immediately sees the explanations and Jacobian differences
    this.switchTab('dof-compare');

    const banner = document.getElementById('preset-info-banner');
    if (banner) {
      banner.textContent = `Robot Modeli Değiştirildi: ${activeDOF}-DOF Konfigürasyonu`;
      banner.style.display = 'block';
      setTimeout(() => { banner.style.display = 'none'; }, 3000);
    }
  }

  applyIKSolution(solIndex) {
    const ikResult = this.kinematics.computeIK(this.poseA, this.currentQ);
    if (ikResult.solutions[solIndex]) {
      this.qA = [...ikResult.solutions[solIndex].q];
      this.mathExplainer.updateIKSolutionsTable(ikResult, solIndex);
      this.recalculateTrajectory();
    }
  }

  seekToTime(timeSec) {
    if (!this.sim.trajectory) return;

    this.sim.currentTime = Math.max(0.0, Math.min(this.sim.duration, timeSec));

    // Find sample closest to currentTime
    const steps = this.sim.trajectory.stepsCount;
    const ratio = this.sim.currentTime / this.sim.duration;
    const sampleIdx = Math.min(steps - 1, Math.max(0, Math.round(ratio * (steps - 1))));

    const targetQ = this.sim.trajectory.qSamples[sampleIdx];
    this.updateRobotState(targetQ);

    // Update timeline controls
    const timeSlider = document.getElementById('timeline-slider');
    const timeDisplay = document.getElementById('timeline-current-time');
    const tcpSpeedHud = document.getElementById('hud-speed-val');

    if (timeSlider) {
      timeSlider.value = (ratio * 1000).toFixed(0);
    }
    if (timeDisplay) {
      timeDisplay.textContent = `${this.sim.currentTime.toFixed(2)}s / ${this.sim.duration.toFixed(1)}s`;
    }
    if (tcpSpeedHud && this.sim.trajectory.tcpSamples[sampleIdx]) {
      tcpSpeedHud.textContent = this.sim.trajectory.tcpSamples[sampleIdx].speed.toFixed(1);
    }

    this.chartsManager.setPlayheadTime(this.sim.currentTime);
  }

  play() {
    if (this.sim.currentTime >= this.sim.duration - 0.01) {
      this.sim.currentTime = 0.0;
    }
    this.sim.isPlaying = true;
    this.sim.lastFrameTime = performance.now();
    const playBtn = document.getElementById('btn-play-pause');
    if (playBtn) {
      playBtn.innerHTML = '<span class="icon">⏸</span> Duraklat';
      playBtn.classList.add('btn-playing');
    }
  }

  pause() {
    this.sim.isPlaying = false;
    const playBtn = document.getElementById('btn-play-pause');
    if (playBtn) {
      playBtn.innerHTML = '<span class="icon">▶</span> Oynat';
      playBtn.classList.remove('btn-playing');
    }
  }

  togglePlayPause() {
    if (this.sim.isPlaying) {
      this.pause();
    } else {
      this.play();
    }
  }

  resetSimulation() {
    this.pause();
    this.seekToTime(0.0);
  }

  stepTime(deltaSec) {
    this.pause();
    this.seekToTime(this.sim.currentTime + deltaSec);
  }

  simLoop(now) {
    requestAnimationFrame(this.simLoop);

    if (this.sim.isPlaying && this.sim.trajectory) {
      const dtMs = now - this.sim.lastFrameTime;
      this.sim.lastFrameTime = now;

      const dtSec = (dtMs / 1000.0) * this.sim.speedMultiplier;
      const nextTime = this.sim.currentTime + dtSec;

      if (nextTime >= this.sim.duration) {
        this.seekToTime(this.sim.duration);
        this.pause();
      } else {
        this.seekToTime(nextTime);
      }
    } else {
      this.sim.lastFrameTime = now;
    }
  }

  loadPreset(presetKey) {
    const p = this.presets[presetKey];
    if (!p) return;

    // Apply values to inputs
    document.getElementById('input-ax').value = p.poseA.x;
    document.getElementById('input-ay').value = p.poseA.y;
    document.getElementById('input-az').value = p.poseA.z;
    document.getElementById('input-aroll').value = p.poseA.rollDeg;
    document.getElementById('input-apitch').value = p.poseA.pitchDeg;
    document.getElementById('input-ayaw').value = p.poseA.yawDeg;

    document.getElementById('input-bx').value = p.poseB.x;
    document.getElementById('input-by').value = p.poseB.y;
    document.getElementById('input-bz').value = p.poseB.z;
    document.getElementById('input-broll').value = p.poseB.rollDeg;
    document.getElementById('input-bpitch').value = p.poseB.pitchDeg;
    document.getElementById('input-byaw').value = p.poseB.yawDeg;

    document.getElementById('select-trajectory-mode').value = p.mode;
    document.getElementById('input-duration').value = p.duration;

    this.recalculateTrajectory();

    // Show toast or info
    const banner = document.getElementById('preset-info-banner');
    if (banner) {
      banner.textContent = `Senaryo: ${p.name} - ${p.desc}`;
      banner.style.display = 'block';
    }
  }

  setupUIEventListeners() {
    // Play / Pause / Reset / Steps
    const playBtn = document.getElementById('btn-play-pause');
    if (playBtn) playBtn.addEventListener('click', () => this.togglePlayPause());

    const resetBtn = document.getElementById('btn-reset');
    if (resetBtn) resetBtn.addEventListener('click', () => this.resetSimulation());

    const stepForwardBtn = document.getElementById('btn-step-fwd');
    if (stepForwardBtn) stepForwardBtn.addEventListener('click', () => this.stepTime(0.05));

    const stepBackBtn = document.getElementById('btn-step-back');
    if (stepBackBtn) stepBackBtn.addEventListener('click', () => this.stepTime(-0.05));

    // Timeline Scrubber
    const timeSlider = document.getElementById('timeline-slider');
    if (timeSlider) {
      timeSlider.addEventListener('input', (e) => {
        this.pause();
        const ratio = parseFloat(e.target.value) / 1000.0;
        this.seekToTime(ratio * this.sim.duration);
      });
    }

    // Speed multiplier buttons
    const speedSelect = document.getElementById('select-speed');
    if (speedSelect) {
      speedSelect.addEventListener('change', (e) => {
        this.sim.speedMultiplier = parseFloat(e.target.value) || 1.0;
      });
    }

    // Go to Point A / Go to Point B
    const gotoABtn = document.getElementById('btn-goto-a');
    if (gotoABtn) gotoABtn.addEventListener('click', () => { this.pause(); this.seekToTime(0.0); });

    const gotoBBtn = document.getElementById('btn-goto-b');
    if (gotoBBtn) gotoBBtn.addEventListener('click', () => { this.pause(); this.seekToTime(this.sim.duration); });

    // Recalculate button
    const recalcBtn = document.getElementById('btn-recalculate');
    if (recalcBtn) recalcBtn.addEventListener('click', () => this.recalculateTrajectory());

    // Input coordinate changes
    const coordInputs = document.querySelectorAll('.coord-input');
    coordInputs.forEach(input => {
      input.addEventListener('change', () => this.recalculateTrajectory());
    });

    // Preset selector
    const presetSelect = document.getElementById('select-preset');
    if (presetSelect) {
      presetSelect.addEventListener('change', (e) => {
        this.loadPreset(e.target.value);
      });
    }

    // Manual Joint Sliders (Forward Kinematics Jogging)
    for (let i = 0; i < 6; i++) {
      const slider = document.getElementById(`slider-q${i+1}`);
      if (slider) {
        slider.addEventListener('input', (e) => {
          this.pause();
          const deg = parseFloat(e.target.value);
          this.currentQ[i] = deg * Math.PI / 180;
          this.updateRobotState(this.currentQ);
        });
      }
    }

    // "Mevcut Konumu A Noktası Yap" & "B Noktası Yap" buttons
    const setCurAsABtn = document.getElementById('btn-set-current-as-a');
    if (setCurAsABtn) {
      setCurAsABtn.addEventListener('click', () => {
        const fk = this.kinematics.computeFK(this.currentQ);
        document.getElementById('input-ax').value = fk.tcp.x.toFixed(1);
        document.getElementById('input-ay').value = fk.tcp.y.toFixed(1);
        document.getElementById('input-az').value = fk.tcp.z.toFixed(1);
        document.getElementById('input-aroll').value = fk.tcp.rollDeg.toFixed(1);
        document.getElementById('input-apitch').value = fk.tcp.pitchDeg.toFixed(1);
        document.getElementById('input-ayaw').value = fk.tcp.yawDeg.toFixed(1);
        this.recalculateTrajectory();
      });
    }

    const setCurAsBBtn = document.getElementById('btn-set-current-as-b');
    if (setCurAsBBtn) {
      setCurAsBBtn.addEventListener('click', () => {
        const fk = this.kinematics.computeFK(this.currentQ);
        document.getElementById('input-bx').value = fk.tcp.x.toFixed(1);
        document.getElementById('input-by').value = fk.tcp.y.toFixed(1);
        document.getElementById('input-bz').value = fk.tcp.z.toFixed(1);
        document.getElementById('input-broll').value = fk.tcp.rollDeg.toFixed(1);
        document.getElementById('input-bpitch').value = fk.tcp.pitchDeg.toFixed(1);
        document.getElementById('input-byaw').value = fk.tcp.yawDeg.toFixed(1);
        this.recalculateTrajectory();
      });
    }

    // Home / Zero Pose Reset Button
    const resetHomeBtn = document.getElementById('btn-reset-home');
    if (resetHomeBtn) {
      resetHomeBtn.addEventListener('click', () => this.resetToHomePose());
    }

    const resetHomeTimelineBtn = document.getElementById('btn-reset-home-timeline');
    if (resetHomeTimelineBtn) {
      resetHomeTimelineBtn.addEventListener('click', () => this.resetToHomePose());
    }

    // Multi-DOF Selector
    const dofSelect = document.getElementById('select-dof');
    if (dofSelect) {
      dofSelect.addEventListener('change', (e) => {
        this.setDOF(parseInt(e.target.value, 10));
      });
    }

    // 3D Visual toggles
    const toggleFrames = document.getElementById('toggle-frames');
    if (toggleFrames) toggleFrames.addEventListener('change', (e) => this.visualizer.toggleFrames(e.target.checked));

    const toggleGhosts = document.getElementById('toggle-ghosts');
    if (toggleGhosts) toggleGhosts.addEventListener('change', (e) => this.visualizer.toggleGhosts(e.target.checked));

    const toggleTrajectory = document.getElementById('toggle-trajectory');
    if (toggleTrajectory) toggleTrajectory.addEventListener('change', (e) => this.visualizer.toggleTrajectory(e.target.checked));

    const toggleWorkspace = document.getElementById('toggle-workspace');
    if (toggleWorkspace) toggleWorkspace.addEventListener('change', (e) => this.visualizer.toggleWorkspace(e.target.checked));

    const toggleGrid = document.getElementById('toggle-grid');
    if (toggleGrid) toggleGrid.addEventListener('change', (e) => this.visualizer.toggleGrid(e.target.checked));

    // Camera preset buttons
    const camPresets = document.querySelectorAll('.cam-btn');
    camPresets.forEach(btn => {
      btn.addEventListener('click', (e) => {
        e.preventDefault();
        e.stopPropagation();
        camPresets.forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        const camMode = btn.getAttribute('data-cam');
        if (this.visualizer) {
          this.visualizer.setCameraPreset(camMode);
        }
      });
    });

    // Copy Calculation Report
    const copyReportBtn = document.getElementById('btn-copy-report');
    if (copyReportBtn) {
      copyReportBtn.addEventListener('click', () => {
        const report = this.mathExplainer.generateCalculationReport(
          this.qA,
          this.qB,
          this.poseA,
          this.poseB,
          this.sim.trajectory
        );
        navigator.clipboard.writeText(report).then(() => {
          copyReportBtn.innerHTML = '✓ Rapor Kopyalandı!';
          setTimeout(() => {
            copyReportBtn.innerHTML = '📋 Hesaplama Raporunu Kopyala';
          }, 2500);
        });
      });
    }

    // Keyboard Shortcuts (Space to play/pause, Left/Right arrow to step)
    window.addEventListener('keydown', (e) => {
      if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
      if (e.code === 'Space') {
        e.preventDefault();
        this.togglePlayPause();
      } else if (e.code === 'ArrowRight') {
        e.preventDefault();
        this.stepTime(0.05);
      } else if (e.code === 'ArrowLeft') {
        e.preventDefault();
        this.stepTime(-0.05);
      }
    });
  }

  switchTab(tabId) {
    const tabButtons = document.querySelectorAll('.edu-tab-btn');
    const tabPanels = document.querySelectorAll('.edu-tab-panel');

    tabButtons.forEach(b => {
      if (b.getAttribute('data-tab') === tabId) {
        b.classList.add('active');
        b.scrollIntoView({ behavior: 'smooth', block: 'nearest', inline: 'center' });
      } else {
        b.classList.remove('active');
      }
    });

    tabPanels.forEach(p => {
      if (p.id === `tab-${tabId}`) {
        p.classList.add('active');
      } else {
        p.classList.remove('active');
      }
    });

    if (tabId === 'code-export' && this.codeGenerator) {
      this.codeGenerator.updateCodeDisplay();
    }
  }

  setupTabNavigation() {
    const tabButtons = document.querySelectorAll('.edu-tab-btn');
    tabButtons.forEach(btn => {
      btn.addEventListener('click', () => {
        const targetTab = btn.getAttribute('data-tab');
        this.switchTab(targetTab);
      });
    });
  }
}

// Global bootstrap
window.addEventListener('DOMContentLoaded', () => {
  window.app = new RobotEducationalApp();
  window.app.init();
});
