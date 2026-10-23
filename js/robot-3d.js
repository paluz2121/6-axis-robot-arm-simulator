/**
 * Robot 3D Visualizer & Scene Manager using Three.js
 * 
 * Features:
 * - High-fidelity 6-DOF industrial robot mesh with realistic metallic materials
 * - Forward kinematics bone/joint linkage hierarchy
 * - Interactive Coordinate Frames (X-Red, Y-Green, Z-Blue) at each joint
 * - Point A & Point B 3D Target Waypoint Markers with interactive gizmos
 * - Ghost / Shadow Robot Previews for Point A (cyan) and Point B (emerald)
 * - 3D Trajectory Ribbon / Trail showing TCP path
 * - Reachable Workspace Envelope (translucent boundary sphere)
 * - Studio lighting, shadow floor grid, OrbitControls with view presets
 */

class Robot3DVisualizer {
  constructor(containerElement, kinematicsEngine) {
    this.container = containerElement;
    this.kinematics = kinematicsEngine;

    this.showFrames = true;
    this.showGhosts = true;
    this.showTrajectory = true;
    this.showWorkspace = false;
    this.showGrid = true;

    this.jointObjects = [];
    this.frameHelpers = [];
    this.ghostA = null;
    this.ghostB = null;

    // Camera follow & transition animation state
    this.isFollowingTCP = false;
    this.currentTCPPos = new THREE.Vector3(450, 220, 250);
    this.camTransition = {
      isAnimating: false,
      startPos: new THREE.Vector3(),
      targetPos: new THREE.Vector3(),
      startTarget: new THREE.Vector3(),
      targetTarget: new THREE.Vector3(),
      startTime: 0,
      duration: 650
    };

    this.initScene();
    this.buildRobot();
    this.buildGhosts();
    this.buildWaypoints();
    this.buildWorkspaceEnvelope();
    this.setupEventListeners();

    this.animate = this.animate.bind(this);
    requestAnimationFrame(this.animate);
  }

  initScene() {
    // 1. Scene setup
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0c101d);
    this.scene.fog = new THREE.FogExp2(0x0c101d, 0.0006);

    // 2. Camera setup
    const aspect = this.container.clientWidth / this.container.clientHeight;
    this.camera = new THREE.PerspectiveCamera(45, aspect, 10, 5000);
    this.camera.position.set(1100, 950, 1200);

    // 3. Renderer setup
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.container.appendChild(this.renderer.domElement);

    // 4. OrbitControls
    if (typeof THREE.OrbitControls !== 'undefined') {
      this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
      this.controls.enableDamping = true;
      this.controls.dampingFactor = 0.08;
      this.controls.maxPolarAngle = Math.PI / 2 + 0.05; // Don't go below floor
      this.controls.minDistance = 150;
      this.controls.maxDistance = 4000;
      this.controls.target.set(0, 350, 0);

      // Disable follow / animation if user manually rotates
      this.controls.addEventListener('start', () => {
        this.camTransition.isAnimating = false;
        if (this.isFollowingTCP) {
          this.isFollowingTCP = false;
          const tcpBtn = document.querySelector('.cam-btn[data-cam="tcp"]');
          if (tcpBtn) tcpBtn.classList.remove('active');
        }
      });
    }

    // 5. Lighting
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.55);
    this.scene.add(ambientLight);

    const mainLight = new THREE.DirectionalLight(0xffffff, 0.9);
    mainLight.position.set(800, 1500, 800);
    mainLight.castShadow = true;
    mainLight.shadow.mapSize.width = 2048;
    mainLight.shadow.mapSize.height = 2048;
    mainLight.shadow.camera.near = 100;
    mainLight.shadow.camera.far = 4000;
    const d = 1200;
    mainLight.shadow.camera.left = -d;
    mainLight.shadow.camera.right = d;
    mainLight.shadow.camera.top = d;
    mainLight.shadow.camera.bottom = -d;
    mainLight.shadow.bias = -0.0005;
    this.scene.add(mainLight);

    const rimLight = new THREE.DirectionalLight(0x38bdf8, 0.6); // Cyan rim light
    rimLight.position.set(-1000, 800, -1000);
    this.scene.add(rimLight);

    const underGlow = new THREE.PointLight(0x6366f1, 0.5, 1200); // Indigo under-glow
    underGlow.position.set(0, 50, 0);
    this.scene.add(underGlow);

    // 6. Floor Grid & Reflection Plane
    this.gridGroup = new THREE.Group();
    const gridHelper = new THREE.GridHelper(2400, 48, 0x38bdf8, 0x1e293b);
    gridHelper.position.y = 0.5;
    this.gridGroup.add(gridHelper);

    const floorGeo = new THREE.PlaneGeometry(3500, 3500);
    const floorMat = new THREE.MeshStandardMaterial({
      color: 0x090d16,
      roughness: 0.85,
      metalness: 0.2
    });
    const floorMesh = new THREE.Mesh(floorGeo, floorMat);
    floorMesh.rotation.x = -Math.PI / 2;
    floorMesh.receiveShadow = true;
    this.gridGroup.add(floorMesh);

    // Outer boundary ring on floor
    const ringGeo = new THREE.RingGeometry(900, 908, 64);
    const ringMat = new THREE.MeshBasicMaterial({ color: 0x1e3a8a, side: THREE.DoubleSide, transparent: true, opacity: 0.4 });
    const ringMesh = new THREE.Mesh(ringGeo, ringMat);
    ringMesh.rotation.x = -Math.PI / 2;
    ringMesh.position.y = 1;
    this.gridGroup.add(ringMesh);

    this.scene.add(this.gridGroup);

    // 7. Trajectory line container
    this.trajectoryGroup = new THREE.Group();
    this.scene.add(this.trajectoryGroup);
  }

  /**
   * Helper to create styled coordinate axes triad (RGB)
   */
  createAxesHelper(size = 80, labelText = "") {
    const group = new THREE.Group();

    // X Axis: Red
    const xGeo = new THREE.CylinderGeometry(2, 2, size, 8);
    const xMat = new THREE.MeshBasicMaterial({ color: 0xef4444 });
    const xCyl = new THREE.Mesh(xGeo, xMat);
    xCyl.rotation.z = -Math.PI / 2;
    xCyl.position.x = size / 2;
    group.add(xCyl);

    const xCone = new THREE.Mesh(new THREE.ConeGeometry(5, 14, 8), xMat);
    xCone.rotation.z = -Math.PI / 2;
    xCone.position.x = size + 7;
    group.add(xCone);

    // Y Axis: Green
    const yMat = new THREE.MeshBasicMaterial({ color: 0x10b981 });
    const yCyl = new THREE.Mesh(xGeo, yMat);
    yCyl.position.y = size / 2;
    group.add(yCyl);

    const yCone = new THREE.Mesh(new THREE.ConeGeometry(5, 14, 8), yMat);
    yCone.position.y = size + 7;
    group.add(yCone);

    // Z Axis: Blue
    const zMat = new THREE.MeshBasicMaterial({ color: 0x3b82f6 });
    const zCyl = new THREE.Mesh(xGeo, zMat);
    zCyl.rotation.x = Math.PI / 2;
    zCyl.position.z = size / 2;
    group.add(zCyl);

    const zCone = new THREE.Mesh(new THREE.ConeGeometry(5, 14, 8), zMat);
    zCone.rotation.x = Math.PI / 2;
    zCone.position.z = size + 7;
    group.add(zCone);

    // Center Origin Sphere
    const originMesh = new THREE.Mesh(
      new THREE.SphereGeometry(3.5, 12, 12),
      new THREE.MeshBasicMaterial({ color: 0xffffff })
    );
    group.add(originMesh);

    return group;
  }

  /**
   * Construct 3D Visual Robot Arm Model
   */
  buildRobot(isGhost = false, ghostColor = 0x38bdf8) {
    const robotRoot = new THREE.Group();

    // Color theme
    let matBase, matLink, matJoint, matAccent, matTool;

    if (isGhost) {
      const ghostMat = new THREE.MeshStandardMaterial({
        color: ghostColor,
        roughness: 0.3,
        metalness: 0.1,
        transparent: true,
        opacity: 0.28,
        wireframe: false
      });
      matBase = matLink = matJoint = matAccent = matTool = ghostMat;
    } else {
      matBase = new THREE.MeshStandardMaterial({
        color: 0x1e293b,
        roughness: 0.4,
        metalness: 0.8
      });
      matLink = new THREE.MeshStandardMaterial({
        color: 0xe2e8f0, // Sleek industrial off-white/titanium
        roughness: 0.35,
        metalness: 0.4
      });
      matJoint = new THREE.MeshStandardMaterial({
        color: 0x0f172a, // Dark gunmetal joint housings
        roughness: 0.5,
        metalness: 0.85
      });
      matAccent = new THREE.MeshStandardMaterial({
        color: 0x0ea5e9, // Glowing sky blue accents
        roughness: 0.2,
        metalness: 0.6,
        emissive: 0x0284c7,
        emissiveIntensity: 0.3
      });
      matTool = new THREE.MeshStandardMaterial({
        color: 0xf59e0b, // Amber industrial tool flange
        roughness: 0.3,
        metalness: 0.7
      });
    }

    const dh = this.kinematics.dh;
    const d1 = dh[0].d;
    const a2 = dh[1].a;
    const d4 = dh[3].d;
    const d6 = dh[5].d;

    // --- Base Stand (Static) ---
    const baseStand = new THREE.Group();

    const pedestal = new THREE.Mesh(
      new THREE.CylinderGeometry(150, 190, 80, 32),
      matBase
    );
    pedestal.position.y = 40;
    pedestal.castShadow = !isGhost;
    pedestal.receiveShadow = !isGhost;
    baseStand.add(pedestal);

    const baseRings = new THREE.Mesh(
      new THREE.TorusGeometry(152, 6, 16, 32),
      matAccent
    );
    baseRings.rotation.x = Math.PI / 2;
    baseRings.position.y = 65;
    baseStand.add(baseRings);

    // Base Pillar up to Joint 1
    const basePillar = new THREE.Mesh(
      new THREE.CylinderGeometry(110, 130, d1 - 80, 24),
      matBase
    );
    basePillar.position.y = 40 + (d1 - 80) / 2;
    basePillar.castShadow = !isGhost;
    baseStand.add(basePillar);

    robotRoot.add(baseStand);

    // --- Joint 1: Base Yaw Turret (Rotates around Y in Three.js coords) ---
    const j1Group = new THREE.Group();
    j1Group.position.set(0, d1, 0);

    const turret = new THREE.Mesh(
      new THREE.CylinderGeometry(115, 115, 100, 24),
      matJoint
    );
    turret.castShadow = !isGhost;
    j1Group.add(turret);

    const turretRing = new THREE.Mesh(
      new THREE.TorusGeometry(118, 4, 16, 32),
      matAccent
    );
    turretRing.rotation.x = Math.PI / 2;
    j1Group.add(turretRing);

    robotRoot.add(j1Group);

    // --- Joint 2: Shoulder Pitch (Rotates around X/Z horizontal axis) ---
    const j2Group = new THREE.Group();
    j2Group.position.set(0, 0, 0); // At joint 1 origin

    // Shoulder motor casing
    const shoulderMotor = new THREE.Mesh(
      new THREE.CylinderGeometry(95, 95, 160, 24),
      matJoint
    );
    shoulderMotor.rotation.z = Math.PI / 2;
    shoulderMotor.castShadow = !isGhost;
    j2Group.add(shoulderMotor);

    // Link 2: Upper Arm (extends length a2)
    const upperArm = new THREE.Group();
    const armBeam = new THREE.Mesh(
      new THREE.BoxGeometry(85, a2, 75),
      matLink
    );
    armBeam.position.set(0, a2 / 2, 0);
    armBeam.castShadow = !isGhost;
    upperArm.add(armBeam);

    // Side accent strips
    const stripL = new THREE.Mesh(new THREE.BoxGeometry(90, a2 * 0.7, 8), matAccent);
    stripL.position.set(0, a2 / 2, 40);
    upperArm.add(stripL);

    j2Group.add(upperArm);
    j1Group.add(j2Group);

    // --- Joint 3: Elbow Pitch ---
    const j3Group = new THREE.Group();
    j3Group.position.set(0, a2, 0); // At top of upper arm

    const elbowMotor = new THREE.Mesh(
      new THREE.CylinderGeometry(80, 80, 140, 24),
      matJoint
    );
    elbowMotor.rotation.z = Math.PI / 2;
    elbowMotor.castShadow = !isGhost;
    j3Group.add(elbowMotor);

    // Forearm link extends d4
    const forearm = new THREE.Group();
    const forearmBeam = new THREE.Mesh(
      new THREE.CylinderGeometry(55, 65, d4, 20),
      matLink
    );
    forearmBeam.position.set(0, d4 / 2, 0);
    forearmBeam.castShadow = !isGhost;
    forearm.add(forearmBeam);

    j3Group.add(forearm);
    j2Group.add(j3Group);

    // --- Joint 4: Forearm Roll ---
    const j4Group = new THREE.Group();
    j4Group.position.set(0, d4, 0); // At wrist center

    const wristRollHub = new THREE.Mesh(
      new THREE.CylinderGeometry(55, 55, 60, 20),
      matJoint
    );
    wristRollHub.castShadow = !isGhost;
    j4Group.add(wristRollHub);

    // --- Joint 5: Wrist Pitch ---
    const j5Group = new THREE.Group();
    const wristPitchHub = new THREE.Mesh(
      new THREE.CylinderGeometry(45, 45, 80, 20),
      matJoint
    );
    wristPitchHub.rotation.x = Math.PI / 2;
    wristPitchHub.castShadow = !isGhost;
    j5Group.add(wristPitchHub);
    j4Group.add(j5Group);

    // --- Joint 6: Wrist Roll / Tool Flange ---
    const j6Group = new THREE.Group();
    const flange = new THREE.Mesh(
      new THREE.CylinderGeometry(40, 48, 25, 20),
      matTool
    );
    flange.position.set(0, 12, 0);
    flange.castShadow = !isGhost;
    j6Group.add(flange);

    // Tool tip pointer & laser
    const pointer = new THREE.Mesh(
      new THREE.ConeGeometry(12, d6 - 25, 16),
      matBase
    );
    pointer.position.set(0, 25 + (d6 - 25) / 2, 0);
    pointer.castShadow = !isGhost;
    j6Group.add(pointer);

    // TCP Tip Indicator (Sphere)
    const tcpTip = new THREE.Mesh(
      new THREE.SphereGeometry(6, 12, 12),
      new THREE.MeshBasicMaterial({ color: isGhost ? ghostColor : 0xef4444 })
    );
    tcpTip.position.set(0, d6, 0);
    j6Group.add(tcpTip);

    j5Group.add(j6Group);
    j3Group.add(j4Group);

    // Store references
    const model = {
      root: robotRoot,
      joints: [j1Group, j2Group, j3Group, j4Group, j5Group, j6Group],
      parts: { baseStand, turret, upperArm, forearm, j4Group, j5Group, j6Group, tcpTip }
    };

    if (!isGhost) {
      this.robotModel = model;
      this.scene.add(robotRoot);
      this.buildCoordinateFrames();
    }

    return model;
  }

  /**
   * Build coordinate frame visual helpers for all joints (0 to 6)
   */
  buildCoordinateFrames() {
    this.frameGroup = new THREE.Group();

    // 7 Frames: F0 (Base) + F1...F6
    this.jointFrames = [];
    for (let i = 0; i <= 6; i++) {
      const helper = this.createAxesHelper(i === 6 ? 90 : 65);
      helper.visible = this.showFrames;
      this.jointFrames.push(helper);
      this.frameGroup.add(helper);
    }

    this.scene.add(this.frameGroup);
  }

  /**
   * Build Ghost / Semi-transparent Preview Robots for Waypoint A and Waypoint B
   */
  buildGhosts() {
    this.ghostA = this.buildRobot(true, 0x06b6d4); // Cyan Ghost
    this.ghostB = this.buildRobot(true, 0x10b981); // Emerald Ghost

    this.ghostA.root.visible = this.showGhosts;
    this.ghostB.root.visible = this.showGhosts;

    this.scene.add(this.ghostA.root);
    this.scene.add(this.ghostB.root);
  }

  /**
   * 3D Markers for Point A and Point B
   */
  buildWaypoints() {
    this.waypointGroup = new THREE.Group();

    // Marker A (Start)
    this.markerA = this.createWaypointMarker(0x06b6d4, "A (Başlangıç)");
    this.markerB = this.createWaypointMarker(0x10b981, "B (Hedef)");

    this.waypointGroup.add(this.markerA);
    this.waypointGroup.add(this.markerB);

    this.scene.add(this.waypointGroup);
  }

  createWaypointMarker(colorHex, label) {
    const group = new THREE.Group();

    // Pulsing core sphere
    const sphere = new THREE.Mesh(
      new THREE.SphereGeometry(14, 20, 20),
      new THREE.MeshBasicMaterial({ color: colorHex, wireframe: false })
    );
    group.add(sphere);

    // Glowing outer halo
    const halo = new THREE.Mesh(
      new THREE.SphereGeometry(24, 16, 16),
      new THREE.MeshBasicMaterial({ color: colorHex, transparent: true, opacity: 0.25, wireframe: true })
    );
    group.add(halo);

    // Orientation Triad at target
    const triad = this.createAxesHelper(70);
    group.add(triad);

    // Vertical guide line down to floor
    const lineGeo = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(0, -500, 0)
    ]);
    const lineMat = new THREE.LineDashedMaterial({
      color: colorHex,
      dashSize: 12,
      gapSize: 8,
      transparent: true,
      opacity: 0.5
    });
    const guideLine = new THREE.Line(lineGeo, lineMat);
    guideLine.computeLineDistances();
    group.add(guideLine);

    // Ground target circle
    const groundCircle = new THREE.Mesh(
      new THREE.RingGeometry(20, 24, 32),
      new THREE.MeshBasicMaterial({ color: colorHex, side: THREE.DoubleSide, transparent: true, opacity: 0.6 })
    );
    groundCircle.rotation.x = -Math.PI / 2;
    group.add(groundCircle);

    group.userData = { sphere, halo, guideLine, groundCircle };

    return group;
  }

  /**
   * Spherical Reachable Workspace Envelope
   */
  buildWorkspaceEnvelope() {
    const dh = this.kinematics.dh;
    const maxReach = dh[1].a + dh[3].d + dh[5].d; // a2 + d4 + d6

    const sphereGeo = new THREE.SphereGeometry(maxReach, 36, 18, 0, Math.PI * 2, 0, Math.PI / 2);
    const sphereMat = new THREE.MeshBasicMaterial({
      color: 0x38bdf8,
      transparent: true,
      opacity: 0.08,
      wireframe: true
    });
    this.workspaceSphere = new THREE.Mesh(sphereGeo, sphereMat);
    this.workspaceSphere.position.set(0, dh[0].d, 0); // Centered at shoulder
    this.workspaceSphere.visible = this.showWorkspace;
    this.scene.add(this.workspaceSphere);
  }

  /**
   * Update 3D Robot Pose using Forward Kinematics Transformation Matrices
   * 
   * Transforms robot links matching DH homogeneous matrices T0...T6
   */
  updateRobotPose(q, targetModel = this.robotModel) {
    const fk = this.kinematics.computeFK(q);
    const T = fk.cumulative;

    // Convert 4x4 DH matrices into Three.js Matrix4
    // DH standard: Z is along joint axis, X is normal.
    // Three.js world: Y is up, X is right, Z is front.
    // Note: We can directly place frames and link meshes using Three.js Matrix4 decomposition!
    if (targetModel === this.robotModel && this.jointFrames) {
      for (let i = 0; i <= 6; i++) {
        const Ti = T[i];
        const m = new THREE.Matrix4();
        // Set matrix elements (column-major in Three.js)
        m.set(
          Ti[0][0], Ti[0][1], Ti[0][2], Ti[0][3],
          Ti[2][0], Ti[2][1], Ti[2][2], Ti[2][3], // Swapping Y and Z for robotics (Z-up) to Three.js (Y-up)
          -Ti[1][0], -Ti[1][1], -Ti[1][2], -Ti[1][3],
          Ti[3][0], Ti[3][1], Ti[3][2], Ti[3][3]
        );

        // Or direct mapping since robotics convention is Z up:
        // In our scene: Floor is Y=0, so Base is along Y axis!
        // Let's map DH coords:
        // DH X -> Three.js X
        // DH Y -> Three.js -Z
        // DH Z -> Three.js Y
        this.applyDhMatrixToThreeObject(this.jointFrames[i], Ti);
      }
    }

    // Apply joint angles to robot visual hierarchy
    // Joint 1: Base Yaw around Y
    targetModel.joints[0].rotation.y = -q[0];

    // Joint 2: Shoulder Pitch around X
    targetModel.joints[1].rotation.x = q[1];

    // Joint 3: Elbow Pitch around X
    targetModel.joints[2].rotation.x = q[2];

    // Joint 4: Forearm Roll around Y
    targetModel.joints[3].rotation.y = q[3];

    // Joint 5: Wrist Pitch around X
    targetModel.joints[4].rotation.x = q[4];

    // Joint 6: Tool Roll around Y
    targetModel.joints[5].rotation.y = q[5];

    if (targetModel === this.robotModel) {
      // Store current TCP position in Three.js world coordinates
      // DH (X, Y, Z) -> Three.js (X, Z, -Y)
      this.currentTCPPos.set(fk.tcp.x, fk.tcp.z, -fk.tcp.y);
    }

    return fk;
  }

  /**
   * Map standard robotics DH coordinates (X-East, Y-North, Z-Up) to Three.js (X-Right, Y-Up, Z-Front)
   */
  applyDhMatrixToThreeObject(object, dhMatrix) {
    // DH matrix:
    // [ r11 r12 r13 px ] (DH X)
    // [ r21 r22 r23 py ] (DH Y)
    // [ r31 r32 r33 pz ] (DH Z)
    // [  0   0   0   1 ]
    // Three.js world:
    // Three.X =  DH X
    // Three.Y =  DH Z
    // Three.Z = -DH Y

    const px = dhMatrix[0][3];
    const py = dhMatrix[1][3];
    const pz = dhMatrix[2][3];

    object.position.set(px, pz, -py);

    const m = new THREE.Matrix4();
    m.set(
      dhMatrix[0][0],  dhMatrix[0][2], -dhMatrix[0][1], px,
      dhMatrix[2][0],  dhMatrix[2][2], -dhMatrix[2][1], pz,
     -dhMatrix[1][0], -dhMatrix[1][2],  dhMatrix[1][1], -py,
      0, 0, 0, 1
    );

    object.setRotationFromMatrix(m);
  }

  /**
   * Set Waypoint A Position & Orientation
   */
  setWaypointAPose(pose, qPose = null) {
    this.markerA.position.set(pose.x, pose.z, -pose.y);
    const R = RobotKinematics.eulerZYXToRotationMatrix(pose.roll, pose.pitch, pose.yaw);
    const m = new THREE.Matrix4();
    m.set(
      R[0][0],  R[0][2], -R[0][1], pose.x,
      R[2][0],  R[2][2], -R[2][1], pose.z,
     -R[1][0], -R[1][2],  R[1][1], -pose.y,
      0, 0, 0, 1
    );
    this.markerA.setRotationFromMatrix(m);

    // Update ground guide line height
    const guideLine = this.markerA.userData.guideLine;
    guideLine.geometry.setFromPoints([
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(0, -pose.z, 0)
    ]);
    guideLine.computeLineDistances();
    this.markerA.userData.groundCircle.position.set(0, -pose.z + 1, 0);

    if (qPose && this.ghostA) {
      this.updateRobotPose(qPose, this.ghostA);
    }
  }

  /**
   * Set Waypoint B Position & Orientation
   */
  setWaypointBPose(pose, qPose = null) {
    this.markerB.position.set(pose.x, pose.z, -pose.y);
    const R = RobotKinematics.eulerZYXToRotationMatrix(pose.roll, pose.pitch, pose.yaw);
    const m = new THREE.Matrix4();
    m.set(
      R[0][0],  R[0][2], -R[0][1], pose.x,
      R[2][0],  R[2][2], -R[2][1], pose.z,
     -R[1][0], -R[1][2],  R[1][1], -pose.y,
      0, 0, 0, 1
    );
    this.markerB.setRotationFromMatrix(m);

    // Update ground guide line height
    const guideLine = this.markerB.userData.guideLine;
    guideLine.geometry.setFromPoints([
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(0, -pose.z, 0)
    ]);
    guideLine.computeLineDistances();
    this.markerB.userData.groundCircle.position.set(0, -pose.z + 1, 0);

    if (qPose && this.ghostB) {
      this.updateRobotPose(qPose, this.ghostB);
    }
  }

  /**
   * Render 3D Trajectory Ribbon / Curve along waypoints
   */
  renderTrajectoryCurve(trajectoryData) {
    // Clear old trajectory meshes
    while (this.trajectoryGroup.children.length > 0) {
      const obj = this.trajectoryGroup.children[0];
      if (obj.geometry) obj.geometry.dispose();
      if (obj.material) obj.material.dispose();
      this.trajectoryGroup.remove(obj);
    }

    if (!this.showTrajectory || !trajectoryData || !trajectoryData.tcpSamples) {
      return;
    }

    const samples = trajectoryData.tcpSamples;
    const points = samples.map(tcp => new THREE.Vector3(tcp.x, tcp.z, -tcp.y));

    // 1. Core glowing curve line
    const curve = new THREE.CatmullRomCurve3(points);
    const tubeGeo = new THREE.TubeGeometry(curve, points.length * 2, 3.5, 8, false);

    // Gradient vertex colors or cyan-amber highlight based on velocity
    const isCartesian = trajectoryData.mode === 'cartesian-linear';
    const tubeMat = new THREE.MeshBasicMaterial({
      color: isCartesian ? 0x06b6d4 : 0xf59e0b, // Cyan for Cartesian LIN, Amber for Joint PTP
      transparent: true,
      opacity: 0.85
    });

    const tubeMesh = new THREE.Mesh(tubeGeo, tubeMat);
    this.trajectoryGroup.add(tubeMesh);

    // 2. Trajectory sample tick marks
    const tickGeo = new THREE.BufferGeometry();
    const tickPositions = [];
    for (let i = 0; i < points.length; i += 4) {
      tickPositions.push(points[i].x, points[i].y, points[i].z);
    }
    tickGeo.setAttribute('position', new THREE.Float32BufferAttribute(tickPositions, 3));
    const tickMat = new THREE.PointsMaterial({
      color: 0xffffff,
      size: 5,
      transparent: true,
      opacity: 0.75
    });
    const tickPoints = new THREE.Points(tickGeo, tickMat);
    this.trajectoryGroup.add(tickPoints);
  }

  /**
   * Smoothly animate camera to new position and target
   */
  animateCameraTo(targetPos, targetTarget, duration = 650) {
    if (!this.controls || !this.camera) return;
    this.camTransition.isAnimating = true;
    this.camTransition.startPos.copy(this.camera.position);
    this.camTransition.targetPos.copy(targetPos);
    this.camTransition.startTarget.copy(this.controls.target);
    this.camTransition.targetTarget.copy(targetTarget);
    this.camTransition.startTime = performance.now();
    this.camTransition.duration = duration;
  }

  /**
   * Camera View Presets
   */
  setCameraPreset(preset) {
    if (!this.controls || !this.camera) return;

    this.isFollowingTCP = (preset === 'tcp');

    const newPos = new THREE.Vector3();
    const newTarget = new THREE.Vector3();

    switch (preset) {
      case 'iso':
        newPos.set(1100, 950, 1200);
        newTarget.set(0, 350, 0);
        break;
      case 'top':
        // Top view: directly above, with tiny 5mm Z offset to maintain stable Y-up orientation
        newPos.set(0, 1900, 5);
        newTarget.set(0, 350, 0);
        break;
      case 'front':
        // Front view: looking at X-Z plane
        newPos.set(0, 450, 1700);
        newTarget.set(0, 450, 0);
        break;
      case 'side':
        // Side view: looking along X axis
        newPos.set(1700, 450, 0);
        newTarget.set(0, 450, 0);
        break;
      case 'tcp':
        // Focus on current Tool Center Point
        newTarget.copy(this.currentTCPPos);
        newPos.set(
          this.currentTCPPos.x + 350,
          this.currentTCPPos.y + 250,
          this.currentTCPPos.z + 400
        );
        break;
    }

    this.animateCameraTo(newPos, newTarget);
  }

  /**
   * Visibility Toggles
   */
  toggleFrames(visible) {
    this.showFrames = visible;
    if (this.jointFrames) {
      this.jointFrames.forEach(f => f.visible = visible);
    }
  }

  toggleGhosts(visible) {
    this.showGhosts = visible;
    if (this.ghostA) this.ghostA.root.visible = visible;
    if (this.ghostB) this.ghostB.root.visible = visible;
  }

  toggleTrajectory(visible) {
    this.showTrajectory = visible;
    this.trajectoryGroup.visible = visible;
  }

  toggleWorkspace(visible) {
    this.showWorkspace = visible;
    if (this.workspaceSphere) this.workspaceSphere.visible = visible;
  }

  toggleGrid(visible) {
    this.showGrid = visible;
    if (this.gridGroup) this.gridGroup.visible = visible;
  }

  setupEventListeners() {
    window.addEventListener('resize', () => {
      if (!this.container || !this.renderer || !this.camera) return;
      const width = this.container.clientWidth;
      const height = this.container.clientHeight;
      this.camera.aspect = width / height;
      this.camera.updateProjectionMatrix();
      this.renderer.setSize(width, height);
    });
  }

  animate() {
    requestAnimationFrame(this.animate);

    // Smooth camera transition animation
    if (this.camTransition.isAnimating) {
      const elapsed = performance.now() - this.camTransition.startTime;
      const progress = Math.min(1.0, elapsed / this.camTransition.duration);
      // Smooth cubic ease-out
      const ease = 1 - Math.pow(1 - progress, 3);

      this.camera.position.lerpVectors(this.camTransition.startPos, this.camTransition.targetPos, ease);
      this.controls.target.lerpVectors(this.camTransition.startTarget, this.camTransition.targetTarget, ease);
      this.controls.update();

      if (progress >= 1.0) {
        this.camTransition.isAnimating = false;
      }
    } else if (this.isFollowingTCP) {
      // Dynamic TCP tracking during motion
      this.controls.target.lerp(this.currentTCPPos, 0.1);
      const desiredPos = new THREE.Vector3(
        this.currentTCPPos.x + 350,
        this.currentTCPPos.y + 250,
        this.currentTCPPos.z + 400
      );
      this.camera.position.lerp(desiredPos, 0.08);
      this.controls.update();
    } else if (this.controls) {
      this.controls.update();
    }

    // Gentle pulsing animation on Waypoint halos
    const time = Date.now() * 0.003;
    if (this.markerA && this.markerA.userData.halo) {
      const scaleA = 1.0 + 0.12 * Math.sin(time);
      this.markerA.userData.halo.scale.set(scaleA, scaleA, scaleA);
    }
    if (this.markerB && this.markerB.userData.halo) {
      const scaleB = 1.0 + 0.12 * Math.sin(time + Math.PI);
      this.markerB.userData.halo.scale.set(scaleB, scaleB, scaleB);
    }

    this.renderer.render(this.scene, this.camera);
  }
}

// Global export
if (typeof window !== 'undefined') {
  window.Robot3DVisualizer = Robot3DVisualizer;
}
