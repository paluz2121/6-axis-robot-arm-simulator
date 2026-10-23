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
   * Helper to create canvas label billboard sprite
   */
  createLabelSprite(text, textColor = '#38bdf8') {
    const canvas = document.createElement('canvas');
    canvas.width = 180;
    canvas.height = 48;
    const ctx = canvas.getContext('2d');

    // Pill background
    ctx.fillStyle = 'rgba(13, 19, 34, 0.88)';
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.5)';
    ctx.lineWidth = 3;
    if (ctx.roundRect) {
      ctx.beginPath();
      ctx.roundRect(4, 4, 172, 40, 8);
      ctx.fill();
      ctx.stroke();
    } else {
      ctx.fillRect(4, 4, 172, 40);
      ctx.strokeRect(4, 4, 172, 40);
    }

    // Text
    ctx.fillStyle = textColor;
    ctx.font = 'bold 20px Inter, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, 90, 24);

    const texture = new THREE.CanvasTexture(canvas);
    const spriteMat = new THREE.SpriteMaterial({ map: texture, depthTest: false, transparent: true });
    const sprite = new THREE.Sprite(spriteMat);
    sprite.scale.set(55, 15, 1);
    return sprite;
  }

  /**
   * Helper to create styled coordinate axes triad (RGB) with billboard label
   */
  createAxesHelper(size = 80, labelText = "") {
    const group = new THREE.Group();

    // X Axis: Red
    const xGeo = new THREE.CylinderGeometry(2, 2, size, 8);
    const xMat = new THREE.MeshBasicMaterial({ color: 0xef4444, depthTest: false });
    const xCyl = new THREE.Mesh(xGeo, xMat);
    xCyl.rotation.z = -Math.PI / 2;
    xCyl.position.x = size / 2;
    group.add(xCyl);

    const xCone = new THREE.Mesh(new THREE.ConeGeometry(5, 14, 8), xMat);
    xCone.rotation.z = -Math.PI / 2;
    xCone.position.x = size + 7;
    group.add(xCone);

    // Y Axis: Green
    const yMat = new THREE.MeshBasicMaterial({ color: 0x10b981, depthTest: false });
    const yCyl = new THREE.Mesh(xGeo, yMat);
    yCyl.position.y = size / 2;
    group.add(yCyl);

    const yCone = new THREE.Mesh(new THREE.ConeGeometry(5, 14, 8), yMat);
    yCone.position.y = size + 7;
    group.add(yCone);

    // Z Axis: Blue
    const zMat = new THREE.MeshBasicMaterial({ color: 0x38bdf8, depthTest: false });
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
      new THREE.MeshBasicMaterial({ color: 0xffffff, depthTest: false })
    );
    group.add(originMesh);

    // Billboard Text Label
    if (labelText) {
      const label = this.createLabelSprite(labelText);
      label.position.set(0, size + 16, 0);
      group.add(label);
    }

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

    // Forearm link body extends d4 - 110mm to make room for clearly articulated wrist stages
    const forearm = new THREE.Group();
    const forearmBodyLength = d4 - 110;
    const forearmBeam = new THREE.Mesh(
      new THREE.CylinderGeometry(52, 64, forearmBodyLength, 24),
      matLink
    );
    forearmBeam.position.set(0, forearmBodyLength / 2, 0);
    forearmBeam.castShadow = !isGhost;
    forearm.add(forearmBeam);

    // Long accent channels on forearm
    const conduit1 = new THREE.Mesh(new THREE.BoxGeometry(10, forearmBodyLength * 0.85, 12), matAccent);
    conduit1.position.set(32, forearmBodyLength / 2, 0);
    forearm.add(conduit1);
    const conduit2 = new THREE.Mesh(new THREE.BoxGeometry(10, forearmBodyLength * 0.85, 12), matAccent);
    conduit2.position.set(-32, forearmBodyLength / 2, 0);
    forearm.add(conduit2);

    j3Group.add(forearm);
    j2Group.add(j3Group);

    // =========================================================================
    // --- Joint 4: Forearm Roll (Rotates around Y forearm longitudinal axis) ---
    // =========================================================================
    const j4Group = new THREE.Group();
    j4Group.position.set(0, forearmBodyLength, 0); // Positioned cleanly at top of forearm

    // Joint 4 Rotary Bearing Collar
    const j4Collar = new THREE.Mesh(
      new THREE.CylinderGeometry(56, 56, 36, 24),
      matJoint
    );
    j4Collar.position.set(0, 18, 0);
    j4Collar.castShadow = !isGhost;
    j4Group.add(j4Collar);

    // Circular graduation dial ring (visually emphasizes roll rotation)
    const j4Dial = new THREE.Mesh(
      new THREE.TorusGeometry(58, 3.5, 16, 32),
      matAccent
    );
    j4Dial.rotation.x = Math.PI / 2;
    j4Dial.position.set(0, 20, 0);
    j4Group.add(j4Dial);

    // Clevis / Yoke fork extending forward 65mm to Joint 5 pivot
    const j5Offset = 65;
    const forkLeft = new THREE.Mesh(new THREE.BoxGeometry(14, j5Offset, 28), matJoint);
    forkLeft.position.set(-30, 18 + j5Offset / 2, 0);
    forkLeft.castShadow = !isGhost;
    j4Group.add(forkLeft);

    const forkRight = new THREE.Mesh(new THREE.BoxGeometry(14, j5Offset, 28), matJoint);
    forkRight.position.set(30, 18 + j5Offset / 2, 0);
    forkRight.castShadow = !isGhost;
    j4Group.add(forkRight);

    // =========================================================================
    // --- Joint 5: Wrist Pitch (Rotates around X transverse pivot axis) ---
    // =========================================================================
    const j5Group = new THREE.Group();
    j5Group.position.set(0, 18 + j5Offset, 0); // Distinct 65mm forward clearance from Joint 4!

    // Transverse Pitch Pivot Motor Cylinder (along X axis)
    const pitchPivot = new THREE.Mesh(
      new THREE.CylinderGeometry(32, 32, 74, 24),
      matJoint
    );
    pitchPivot.rotation.z = Math.PI / 2;
    pitchPivot.castShadow = !isGhost;
    j5Group.add(pitchPivot);

    // Side Pivot Bearing Caps (Anodized accent rings showing pitch angle)
    const capLeft = new THREE.Mesh(new THREE.CylinderGeometry(18, 18, 78, 16), matAccent);
    capLeft.rotation.z = Math.PI / 2;
    j5Group.add(capLeft);

    // Forward Neck Link extending 55mm to Joint 6 Tool Flange
    const j6Offset = 55;
    const j5Neck = new THREE.Mesh(
      new THREE.CylinderGeometry(36, 42, j6Offset, 20),
      matLink
    );
    j5Neck.position.set(0, j6Offset / 2, 0);
    j5Neck.castShadow = !isGhost;
    j5Group.add(j5Neck);

    j4Group.add(j5Group);

    // =========================================================================
    // --- Joint 6: Wrist Roll / Tool Flange (Rotates around tool roll axis) ---
    // =========================================================================
    const j6Group = new THREE.Group();
    j6Group.position.set(0, j6Offset, 0); // Distinct 55mm forward clearance from Joint 5!

    // Tool Flange Rotary Face Plate (Gold/Amber Industrial Alloy)
    const flangeDisk = new THREE.Mesh(
      new THREE.CylinderGeometry(46, 46, 14, 24),
      matTool
    );
    flangeDisk.position.set(0, 7, 0);
    flangeDisk.castShadow = !isGhost;
    j6Group.add(flangeDisk);

    // 6 Radial Fastener Bolts (visually show tool roll spinning)
    for (let b = 0; b < 6; b++) {
      const angle = (b * Math.PI) / 3;
      const bolt = new THREE.Mesh(new THREE.CylinderGeometry(3, 3, 16, 8), matJoint);
      bolt.position.set(32 * Math.cos(angle), 7, 32 * Math.sin(angle));
      j6Group.add(bolt);
    }

    // Radial Orientation Marker / Pointer needle on rim
    const rollPointer = new THREE.Mesh(new THREE.BoxGeometry(5, 3, 18), matAccent);
    rollPointer.position.set(0, 14, 32);
    j6Group.add(rollPointer);

    // Tool Chuck & Pointer Body
    const toolChuck = new THREE.Mesh(
      new THREE.CylinderGeometry(24, 34, 32, 18),
      matBase
    );
    toolChuck.position.set(0, 23, 0);
    toolChuck.castShadow = !isGhost;
    j6Group.add(toolChuck);

    const pointerLen = Math.max(35, d6 - 45);
    const pointerCone = new THREE.Mesh(
      new THREE.ConeGeometry(9, pointerLen, 16),
      matBase
    );
    pointerCone.position.set(0, 39 + pointerLen / 2, 0);
    pointerCone.castShadow = !isGhost;
    j6Group.add(pointerCone);

    // Glowing TCP Tip Indicator
    const tcpTip = new THREE.Mesh(
      new THREE.SphereGeometry(6, 14, 14),
      new THREE.MeshBasicMaterial({ color: isGhost ? ghostColor : 0xef4444 })
    );
    tcpTip.position.set(0, 39 + pointerLen, 0);
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
   * Build coordinate frame visual helpers for all joints (0 to 6) with distinct labels
   */
  buildCoordinateFrames() {
    this.frameGroup = new THREE.Group();

    const frameNames = [
      "F0: Taban",
      "F1: Kök Yaw",
      "F2: Omuz Pitch",
      "F3: Dirsek Pitch",
      "F4: Bilek Roll",
      "F5: Bilek Pitch",
      "F6: Flanş / TCP"
    ];

    // 7 Frames: F0 (Base) + F1...F6
    this.jointFrames = [];
    for (let i = 0; i <= 6; i++) {
      const helper = this.createAxesHelper(i === 6 ? 85 : 60, frameNames[i]);
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
   * Set Active DOF in 3D Visualizer (3, 4, 5, or 6)
   */
  setDOF(dof) {
    this.currentDOF = dof;
    if (this.jointFrames) {
      for (let i = 0; i <= 6; i++) {
        this.jointFrames[i].visible = this.showFrames && (i <= dof);
      }
    }
  }

  /**
   * Update 3D Robot Pose and Link Articulation
   */
  updateRobotPose(q, targetModel = this.robotModel) {
    const fk = this.kinematics.computeFK(q);
    const n = this.currentDOF || this.kinematics.currentDOF || 6;

    // Apply joint angles to robot visual hierarchy
    // Joint 1: Base Yaw around Y
    targetModel.joints[0].rotation.y = -q[0];

    // Joint 2: Shoulder Pitch around X
    targetModel.joints[1].rotation.x = q[1];

    // Joint 3: Elbow Pitch around X
    targetModel.joints[2].rotation.x = q[2];

    // Joint 4: Forearm Roll around Y (Active if DOF >= 4)
    targetModel.joints[3].rotation.y = (n >= 4) ? q[3] : 0.0;

    // Joint 5: Wrist Pitch around X (Active if DOF >= 5)
    targetModel.joints[4].rotation.x = (n >= 5) ? q[4] : 0.0;

    // Joint 6: Tool Roll around Y (Active if DOF >= 6)
    targetModel.joints[5].rotation.y = (n >= 6) ? q[5] : 0.0;

    // Position coordinate frames directly at physical joint pivot centers (No overlapping!)
    if (targetModel === this.robotModel && this.jointFrames) {
      targetModel.root.updateMatrixWorld(true);

      // Frame 0: Base Origin
      this.jointFrames[0].position.set(0, 0, 0);

      // Frame 1: Base Yaw Turret
      targetModel.joints[0].getWorldPosition(this.jointFrames[1].position);
      targetModel.joints[0].getWorldQuaternion(this.jointFrames[1].quaternion);

      // Frame 2: Shoulder Pitch Pivot
      targetModel.joints[1].getWorldPosition(this.jointFrames[2].position);
      targetModel.joints[1].getWorldQuaternion(this.jointFrames[2].quaternion);

      // Frame 3: Elbow Pitch Pivot
      targetModel.joints[2].getWorldPosition(this.jointFrames[3].position);
      targetModel.joints[2].getWorldQuaternion(this.jointFrames[3].quaternion);

      // Frame 4: Wrist Forearm Roll Collar
      targetModel.joints[3].getWorldPosition(this.jointFrames[4].position);
      targetModel.joints[3].getWorldQuaternion(this.jointFrames[4].quaternion);

      // Frame 5: Wrist Pitch Pivot Pin (separated 65mm forward)
      targetModel.joints[4].getWorldPosition(this.jointFrames[5].position);
      targetModel.joints[4].getWorldQuaternion(this.jointFrames[5].quaternion);

      // Frame 6: Tool Mounting Flange Face (separated 55mm forward)
      targetModel.joints[5].getWorldPosition(this.jointFrames[6].position);
      targetModel.joints[5].getWorldQuaternion(this.jointFrames[6].quaternion);
    }

    if (targetModel === this.robotModel) {
      // Store current TCP position in Three.js world coordinates
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
