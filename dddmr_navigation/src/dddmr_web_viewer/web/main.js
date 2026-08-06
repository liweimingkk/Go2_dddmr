import * as THREE from "./vendor/three.module.js";
import { OrbitControls } from "./vendor/jsm/controls/OrbitControls.js";

const ROSLIB = window.ROSLIB;
if (!ROSLIB) {
  throw new Error("roslibjs 未加载");
}

const canvas = document.getElementById("viewport");
const connectionDot = document.getElementById("connection-dot");
const connectionStatus = document.getElementById("connection-status");
const bridgeStatus = document.getElementById("bridge-status");
const executionBadge = document.getElementById("execution-badge");
const operatorStatus = document.getElementById("operator-status");
const statusCard = document.querySelector(".status-card");
const websocketInput = document.getElementById("websocket-url");
const connectButton = document.getElementById("connect-button");
const mapCount = document.getElementById("map-count");
const groundCount = document.getElementById("ground-count");
const tfStateLabel = document.getElementById("tf-state");
const placementHelp = document.getElementById("placement-help");
const setInitialPoseButton = document.getElementById("set-initial-pose");
const setGoalButton = document.getElementById("set-goal");
const executePreviewButton = document.getElementById("execute-preview");
const cancelNavigationButton = document.getElementById("cancel-navigation");
const resetViewButton = document.getElementById("reset-view");
const toggleMap = document.getElementById("toggle-map");
const toggleGround = document.getElementById("toggle-ground");
const toggleGlobalPath = document.getElementById("toggle-global-path");
const toggleLocalPath = document.getElementById("toggle-local-path");
const previewModal = document.getElementById("preview-modal");
const previewSummary = document.getElementById("preview-summary");
const modalDismissButton = document.getElementById("modal-dismiss");
const modalExecuteButton = document.getElementById("modal-execute");

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x061013);
scene.fog = new THREE.FogExp2(0x061013, 0.012);

const camera = new THREE.PerspectiveCamera(52, 1, 0.02, 2500);
camera.up.set(0, 0, 1);
camera.position.set(10, -13, 9);

const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.dampingFactor = 0.07;
controls.target.set(0, 0, 0.5);
controls.maxDistance = 1000;

scene.add(new THREE.HemisphereLight(0xbcefff, 0x102126, 1.25));
const keyLight = new THREE.DirectionalLight(0xffffff, 1.5);
keyLight.position.set(8, -6, 15);
scene.add(keyLight);

const grid = new THREE.GridHelper(80, 80, 0x31545b, 0x173036);
grid.rotation.x = Math.PI * 0.5;
grid.material.opacity = 0.34;
grid.material.transparent = true;
scene.add(grid);

const axes = new THREE.AxesHelper(1.5);
scene.add(axes);

let ros = null;
let reconnectTimer = null;
let connected = false;
let executionAllowed = false;
let previewReady = false;
let webNavigationActive = false;
let mapObject = null;
let groundObject = null;
let globalPathObject = null;
let localPathObject = null;
let previewPathObject = null;
let selectionObject = null;
let latestBounds = null;
let autoFramed = false;
let placementMode = null;
let placementOrigin = null;
let placementYawPoint = null;
let activePointerId = null;
let previewGoalTopic = null;
let executePreviewTopic = null;
let cancelNavigationTopic = null;
let initialPoseTopic = null;
let requestStatusTopic = null;
const subscriptions = [];
const tfTransforms = new Map();

const raycaster = new THREE.Raycaster();
raycaster.params.Points.threshold = 0.24;
const pointer = new THREE.Vector2();
const horizontalPlane = new THREE.Plane();
const maximumRenderedPoints = 180000;

function setOperatorStatus(message, ok = true) {
  operatorStatus.textContent = message;
  statusCard.classList.toggle("error", !ok);
}

function updateConnectionUi() {
  connectionDot.classList.toggle("connected", connected);
  connectionStatus.textContent = connected ? "ROSBridge 已连接" : "未连接";
  bridgeStatus.textContent = connected ? "等待桥接状态" : "等待 ROSBridge";
  executionBadge.textContent = executionAllowed ? "可执行" : "无运动";
  executionBadge.classList.toggle("live", executionAllowed);
  updateExecutionButtons();
}

function updateExecutionButtons() {
  const enabled = connected && executionAllowed && previewReady && !webNavigationActive;
  executePreviewButton.disabled = !enabled;
  modalExecuteButton.disabled = !enabled;
  executePreviewButton.textContent = executionAllowed ? "确认后执行" : "执行被安全门禁用";
  modalExecuteButton.textContent = executionAllowed ? "确认执行" : "只读模式";
}

function defaultWebsocketUrl() {
  const protocol = window.location.protocol === "https:" ? "wss" : "ws";
  return `${protocol}://${window.location.hostname || "127.0.0.1"}:9090`;
}

websocketInput.value = defaultWebsocketUrl();

function makeRobotModel() {
  const robot = new THREE.Group();
  const bodyMaterial = new THREE.MeshStandardMaterial({
    color: 0xe8f1ef,
    roughness: 0.38,
    metalness: 0.12,
  });
  const darkMaterial = new THREE.MeshStandardMaterial({
    color: 0x101719,
    roughness: 0.5,
  });
  const accentMaterial = new THREE.MeshStandardMaterial({
    color: 0x62f4c5,
    emissive: 0x174d3e,
    roughness: 0.3,
  });
  const body = new THREE.Mesh(new THREE.BoxGeometry(0.58, 0.26, 0.17), bodyMaterial);
  body.position.z = 0.04;
  robot.add(body);

  const head = new THREE.Mesh(new THREE.BoxGeometry(0.15, 0.17, 0.14), darkMaterial);
  head.position.set(0.34, 0, 0.05);
  robot.add(head);

  const hips = [
    [0.21, 0.17],
    [0.21, -0.17],
    [-0.21, 0.17],
    [-0.21, -0.17],
  ];
  hips.forEach(([x, y]) => {
    const hip = new THREE.Mesh(new THREE.BoxGeometry(0.07, 0.07, 0.08), accentMaterial);
    hip.position.set(x, y, -0.02);
    robot.add(hip);
    const leg = new THREE.Mesh(new THREE.BoxGeometry(0.045, 0.045, 0.25), bodyMaterial);
    leg.position.set(x, y, -0.17);
    robot.add(leg);
  });

  const direction = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(0, 0, 0.18),
    0.55,
    0x62f4c5,
    0.16,
    0.08,
  );
  robot.add(direction);
  robot.visible = false;
  return robot;
}

const robotObject = makeRobotModel();
scene.add(robotObject);

function disposeObject(object) {
  if (!object) {
    return;
  }
  object.traverse((child) => {
    if (child.geometry) {
      child.geometry.dispose();
    }
    if (Array.isArray(child.material)) {
      child.material.forEach((material) => material.dispose());
    } else if (child.material) {
      child.material.dispose();
    }
  });
}

function removeObject(object) {
  if (!object) {
    return;
  }
  scene.remove(object);
  disposeObject(object);
}

function resizeRenderer() {
  const width = Math.max(1, canvas.clientWidth);
  const height = Math.max(1, canvas.clientHeight);
  renderer.setSize(width, height, false);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
}

new ResizeObserver(resizeRenderer).observe(canvas);
resizeRenderer();

function bytesFromRosbridge(data) {
  if (typeof data === "string") {
    const binary = window.atob(data);
    const bytes = new Uint8Array(binary.length);
    for (let index = 0; index < binary.length; index += 1) {
      bytes[index] = binary.charCodeAt(index);
    }
    return bytes;
  }
  if (data instanceof Uint8Array) {
    return data;
  }
  if (Array.isArray(data)) {
    return Uint8Array.from(data);
  }
  return new Uint8Array();
}

function readPointField(view, offset, datatype, littleEndian) {
  switch (datatype) {
    case 1:
      return view.getInt8(offset);
    case 2:
      return view.getUint8(offset);
    case 3:
      return view.getInt16(offset, littleEndian);
    case 4:
      return view.getUint16(offset, littleEndian);
    case 5:
      return view.getInt32(offset, littleEndian);
    case 6:
      return view.getUint32(offset, littleEndian);
    case 7:
      return view.getFloat32(offset, littleEndian);
    case 8:
      return view.getFloat64(offset, littleEndian);
    default:
      return Number.NaN;
  }
}

function pointFieldSize(datatype) {
  if (datatype === 1 || datatype === 2) {
    return 1;
  }
  if (datatype === 3 || datatype === 4) {
    return 2;
  }
  if (datatype === 5 || datatype === 6 || datatype === 7) {
    return 4;
  }
  if (datatype === 8) {
    return 8;
  }
  return 0;
}

function parsePointCloud2(message) {
  const bytes = bytesFromRosbridge(message.data);
  const fields = new Map((message.fields || []).map((field) => [field.name, field]));
  const xField = fields.get("x");
  const yField = fields.get("y");
  const zField = fields.get("z");
  if (!xField || !yField || !zField || !message.point_step || bytes.length === 0) {
    return { positions: new Float32Array(), sourceCount: 0 };
  }

  const width = Number(message.width || 0);
  const height = Math.max(1, Number(message.height || 1));
  const sourceCount = width * height;
  const stride = Math.max(1, Math.ceil(sourceCount / maximumRenderedPoints));
  const rowStep = Number(message.row_step || width * message.point_step);
  const pointStep = Number(message.point_step);
  const littleEndian = !message.is_bigendian;
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const output = [];
  let flatIndex = 0;

  for (let row = 0; row < height; row += 1) {
    for (let column = 0; column < width; column += 1) {
      if (flatIndex % stride !== 0) {
        flatIndex += 1;
        continue;
      }
      flatIndex += 1;
      const base = row * rowStep + column * pointStep;
      const maximumOffset = Math.max(
        xField.offset + pointFieldSize(xField.datatype),
        yField.offset + pointFieldSize(yField.datatype),
        zField.offset + pointFieldSize(zField.datatype),
      );
      if (base < 0 || base + maximumOffset > bytes.length) {
        continue;
      }
      const x = readPointField(view, base + xField.offset, xField.datatype, littleEndian);
      const y = readPointField(view, base + yField.offset, yField.datatype, littleEndian);
      const z = readPointField(view, base + zField.offset, zField.datatype, littleEndian);
      if (Number.isFinite(x) && Number.isFinite(y) && Number.isFinite(z)) {
        output.push(x, y, z);
      }
    }
  }
  return { positions: new Float32Array(output), sourceCount };
}

function createPointCloud(message, color, size, opacity) {
  const parsed = parsePointCloud2(message);
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(parsed.positions, 3));
  geometry.computeBoundingBox();
  const material = new THREE.PointsMaterial({
    color,
    size,
    transparent: opacity < 1,
    opacity,
    sizeAttenuation: true,
    depthWrite: opacity > 0.75,
  });
  const points = new THREE.Points(geometry, material);
  return { points, count: parsed.positions.length / 3, sourceCount: parsed.sourceCount };
}

function expandLatestBounds(box) {
  if (!box || box.isEmpty()) {
    return;
  }
  if (!latestBounds) {
    latestBounds = box.clone();
  } else {
    latestBounds.union(box);
  }
}

function setMapCloud(message) {
  removeObject(mapObject);
  const created = createPointCloud(message, 0xffc766, 0.12, 0.88);
  mapObject = created.points;
  mapObject.visible = toggleMap.checked;
  scene.add(mapObject);
  mapCount.textContent = created.count.toLocaleString();
  latestBounds = null;
  expandLatestBounds(mapObject.geometry.boundingBox);
  if (groundObject) {
    expandLatestBounds(groundObject.geometry.boundingBox);
  }
  if (!autoFramed && latestBounds) {
    frameMap();
  }
}

function setGroundCloud(message) {
  removeObject(groundObject);
  const created = createPointCloud(message, 0x62f4c5, 0.085, 0.62);
  groundObject = created.points;
  groundObject.visible = toggleGround.checked;
  scene.add(groundObject);
  groundCount.textContent = created.count.toLocaleString();
  latestBounds = null;
  if (mapObject) {
    expandLatestBounds(mapObject.geometry.boundingBox);
  }
  expandLatestBounds(groundObject.geometry.boundingBox);
  if (!autoFramed && latestBounds) {
    frameMap();
  }
}

function frameMap() {
  if (!latestBounds || latestBounds.isEmpty()) {
    setOperatorStatus("尚未收到可用于取景的地图。", false);
    return;
  }
  const center = latestBounds.getCenter(new THREE.Vector3());
  const size = latestBounds.getSize(new THREE.Vector3());
  const maximumDimension = Math.max(size.x, size.y, size.z, 2);
  const distance = maximumDimension * 1.25 + 4;
  camera.position.copy(center).add(
    new THREE.Vector3(0.8, -1.1, 0.72).normalize().multiplyScalar(distance),
  );
  controls.target.copy(center);
  camera.near = Math.max(0.02, distance / 10000);
  camera.far = Math.max(1000, distance * 15);
  camera.updateProjectionMatrix();
  controls.update();
  autoFramed = true;
}

function createPathObject(message, color, opacity = 1) {
  if (!message.poses || message.poses.length < 2) {
    return null;
  }
  const positions = new Float32Array(message.poses.length * 3);
  message.poses.forEach((pose, index) => {
    positions[index * 3] = pose.pose.position.x;
    positions[index * 3 + 1] = pose.pose.position.y;
    positions[index * 3 + 2] = pose.pose.position.z + 0.06;
  });
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  const material = new THREE.LineBasicMaterial({ color, transparent: opacity < 1, opacity });
  return new THREE.Line(geometry, material);
}

function replacePath(current, message, color, opacity, visible) {
  removeObject(current);
  const next = createPathObject(message, color, opacity);
  if (next) {
    next.visible = visible;
    scene.add(next);
  }
  return next;
}

function normalizeFrame(frame) {
  return String(frame || "").replace(/^\/+/, "");
}

function transformKey(parent, child) {
  return `${normalizeFrame(parent)}|${normalizeFrame(child)}`;
}

function storeTfMessage(message) {
  (message.transforms || []).forEach((stamped) => {
    tfTransforms.set(
      transformKey(stamped.header.frame_id, stamped.child_frame_id),
      stamped.transform,
    );
  });
  updateRobotPose();
}

function findTransform(parent, childCandidates) {
  const normalizedParent = normalizeFrame(parent);
  for (const [key, transform] of tfTransforms.entries()) {
    const [storedParent, storedChild] = key.split("|");
    if (storedParent !== normalizedParent) {
      continue;
    }
    for (const candidate of childCandidates) {
      if (
        storedChild === candidate ||
        storedChild.endsWith(`/${candidate}`) ||
        storedChild.endsWith(`_${candidate}`)
      ) {
        return { transform, child: storedChild };
      }
    }
  }
  return null;
}

function composeTransforms(first, second) {
  const firstRotation = new THREE.Quaternion(
    first.rotation.x,
    first.rotation.y,
    first.rotation.z,
    first.rotation.w,
  ).normalize();
  const secondRotation = new THREE.Quaternion(
    second.rotation.x,
    second.rotation.y,
    second.rotation.z,
    second.rotation.w,
  ).normalize();
  const translation = new THREE.Vector3(
    second.translation.x,
    second.translation.y,
    second.translation.z,
  )
    .applyQuaternion(firstRotation)
    .add(new THREE.Vector3(first.translation.x, first.translation.y, first.translation.z));
  const rotation = firstRotation.clone().multiply(secondRotation);
  return { translation, rotation };
}

function updateRobotPose() {
  const bases = ["base_link", "base_footprint", "robot_base_link"];
  const direct = findTransform("map", bases);
  let pose = null;
  if (direct) {
    pose = composeTransforms(
      { translation: { x: 0, y: 0, z: 0 }, rotation: { x: 0, y: 0, z: 0, w: 1 } },
      direct.transform,
    );
  } else {
    const mapToOdom = findTransform("map", ["odom"]);
    const odomToBase = findTransform("odom", bases);
    if (mapToOdom && odomToBase) {
      pose = composeTransforms(mapToOdom.transform, odomToBase.transform);
    }
  }

  if (!pose) {
    robotObject.visible = false;
    tfStateLabel.textContent = "等待";
    return;
  }
  robotObject.position.copy(pose.translation);
  robotObject.quaternion.copy(pose.rotation);
  robotObject.visible = true;
  tfStateLabel.textContent = "已定位";
}

function makeTopic(name, messageType) {
  return new ROSLIB.Topic({ ros, name, messageType });
}

function subscribe(name, messageType, callback) {
  const topic = makeTopic(name, messageType);
  topic.subscribe(callback);
  subscriptions.push(topic);
  return topic;
}

function clearRosInterfaces() {
  subscriptions.forEach((topic) => topic.unsubscribe());
  subscriptions.length = 0;
  previewGoalTopic = null;
  executePreviewTopic = null;
  cancelNavigationTopic = null;
  initialPoseTopic = null;
  requestStatusTopic = null;
  tfTransforms.clear();
  robotObject.visible = false;
}

function handleBridgeStatus(message) {
  let payload;
  try {
    payload = JSON.parse(message.data);
  } catch (_error) {
    setOperatorStatus(message.data, false);
    return;
  }
  executionAllowed = Boolean(payload.execution_allowed);
  bridgeStatus.textContent = executionAllowed ? "执行门已显式开启" : "安全无运动模式";
  setOperatorStatus(payload.message || payload.event, Boolean(payload.ok));

  if (payload.event === "preview_ready") {
    previewReady = true;
    const points = Number(payload.path_points || 0);
    previewSummary.textContent = `路径包含 ${points.toLocaleString()} 个轨迹点。请检查紫色路径后决定是否执行。`;
    previewModal.hidden = false;
  } else if (
    payload.event === "preview_failed" ||
    payload.event === "preview_rejected" ||
    payload.event === "preview_unavailable"
  ) {
    previewReady = false;
  } else if (
    payload.event === "execution_requested" ||
    payload.event === "execution_accepted"
  ) {
    webNavigationActive = true;
    previewReady = false;
    previewModal.hidden = true;
  } else if (
    payload.event === "execution_succeeded" ||
    payload.event === "execution_canceled" ||
    payload.event === "execution_failed" ||
    payload.event === "execution_rejected"
  ) {
    webNavigationActive = false;
    previewReady = false;
  }
  updateConnectionUi();
}

function setupRosInterfaces() {
  previewGoalTopic = makeTopic("/dddmr_web/preview_goal", "geometry_msgs/PoseStamped");
  executePreviewTopic = makeTopic("/dddmr_web/execute_preview", "std_msgs/Empty");
  cancelNavigationTopic = makeTopic("/dddmr_web/cancel_navigation", "std_msgs/Empty");
  initialPoseTopic = makeTopic(
    "/dddmr_web/initial_pose",
    "geometry_msgs/PoseWithCovarianceStamped",
  );
  requestStatusTopic = makeTopic("/dddmr_web/request_status", "std_msgs/Empty");

  subscribe("/dddmr_web/mapcloud", "sensor_msgs/PointCloud2", setMapCloud);
  subscribe("/dddmr_web/mapground", "sensor_msgs/PointCloud2", setGroundCloud);
  subscribe("/dddmr_web/status", "std_msgs/String", handleBridgeStatus);
  subscribe("/tf", "tf2_msgs/TFMessage", storeTfMessage);
  subscribe("/tf_static", "tf2_msgs/TFMessage", storeTfMessage);
  subscribe("/global_path", "nav_msgs/Path", (message) => {
    globalPathObject = replacePath(
      globalPathObject,
      message,
      0xa878ff,
      0.75,
      toggleGlobalPath.checked,
    );
  });
  subscribe("/prune_plan", "nav_msgs/Path", (message) => {
    localPathObject = replacePath(
      localPathObject,
      message,
      0x54c9e8,
      0.95,
      toggleLocalPath.checked,
    );
  });
  subscribe("/dddmr_web/preview_path", "nav_msgs/Path", (message) => {
    previewPathObject = replacePath(
      previewPathObject,
      message,
      0xc79aff,
      1,
      toggleGlobalPath.checked,
    );
  });

  window.setTimeout(() => {
    if (requestStatusTopic && connected) {
      requestStatusTopic.publish(new ROSLIB.Message({}));
    }
  }, 300);
}

function connectRosbridge() {
  const url = websocketInput.value.trim();
  if (!url) {
    setOperatorStatus("请输入 WebSocket 地址。", false);
    return;
  }
  if (reconnectTimer) {
    window.clearTimeout(reconnectTimer);
    reconnectTimer = null;
  }
  if (ros) {
    clearRosInterfaces();
    ros.close();
  }
  connected = false;
  executionAllowed = false;
  updateConnectionUi();
  setOperatorStatus(`正在连接 ${url} …`);
  const connection = new ROSLIB.Ros({ url });
  ros = connection;

  connection.on("connection", () => {
    if (ros !== connection) {
      return;
    }
    connected = true;
    setupRosInterfaces();
    updateConnectionUi();
    setOperatorStatus("连接成功，正在请求缓存地图。", true);
  });
  connection.on("error", (error) => {
    if (ros !== connection) {
      return;
    }
    setOperatorStatus(`ROSBridge 错误：${String(error)}`, false);
  });
  connection.on("close", () => {
    if (ros !== connection) {
      return;
    }
    clearRosInterfaces();
    connected = false;
    executionAllowed = false;
    previewReady = false;
    webNavigationActive = false;
    updateConnectionUi();
    setOperatorStatus("ROSBridge 连接已断开，3 秒后重试。", false);
    if (!reconnectTimer) {
      reconnectTimer = window.setTimeout(() => {
        reconnectTimer = null;
        connectRosbridge();
      }, 3000);
    }
  });
}

function pointerCoordinates(event) {
  const bounds = canvas.getBoundingClientRect();
  pointer.x = ((event.clientX - bounds.left) / bounds.width) * 2 - 1;
  pointer.y = -((event.clientY - bounds.top) / bounds.height) * 2 + 1;
}

function pickGroundPoint(event) {
  if (!groundObject || !groundObject.visible) {
    return null;
  }
  pointerCoordinates(event);
  raycaster.setFromCamera(pointer, camera);
  const hits = raycaster.intersectObject(groundObject, false);
  if (hits.length === 0 || hits[0].index === undefined) {
    return null;
  }
  const position = groundObject.geometry.getAttribute("position");
  return new THREE.Vector3(
    position.getX(hits[0].index),
    position.getY(hits[0].index),
    position.getZ(hits[0].index),
  );
}

function planePointForEvent(event, z) {
  pointerCoordinates(event);
  raycaster.setFromCamera(pointer, camera);
  horizontalPlane.set(new THREE.Vector3(0, 0, 1), -z);
  return raycaster.ray.intersectPlane(horizontalPlane, new THREE.Vector3());
}

function updateSelectionVisual(mode, origin, yaw) {
  removeObject(selectionObject);
  const color = mode === "initial" ? 0x54c9e8 : 0xc79aff;
  const group = new THREE.Group();
  const ring = new THREE.Mesh(
    new THREE.TorusGeometry(0.22, 0.025, 10, 40),
    new THREE.MeshBasicMaterial({ color }),
  );
  ring.position.copy(origin);
  ring.position.z += 0.03;
  group.add(ring);
  const arrow = new THREE.ArrowHelper(
    new THREE.Vector3(Math.cos(yaw), Math.sin(yaw), 0),
    origin.clone().add(new THREE.Vector3(0, 0, 0.05)),
    0.75,
    color,
    0.2,
    0.1,
  );
  group.add(arrow);
  selectionObject = group;
  scene.add(selectionObject);
}

function yawFromPlacement() {
  if (!placementOrigin || !placementYawPoint) {
    return 0;
  }
  const dx = placementYawPoint.x - placementOrigin.x;
  const dy = placementYawPoint.y - placementOrigin.y;
  return Math.hypot(dx, dy) < 0.08 ? 0 : Math.atan2(dy, dx);
}

function rosTimestamp() {
  const now = Date.now();
  return {
    sec: Math.floor(now / 1000),
    nanosec: (now % 1000) * 1000000,
  };
}

function orientationFromYaw(yaw) {
  return { x: 0, y: 0, z: Math.sin(yaw * 0.5), w: Math.cos(yaw * 0.5) };
}

function publishPlacement(mode, origin, yaw) {
  if (!connected) {
    setOperatorStatus("ROSBridge 未连接。", false);
    return;
  }
  if (mode === "goal") {
    previewReady = false;
    updateExecutionButtons();
    previewGoalTopic.publish(
      new ROSLIB.Message({
        header: { frame_id: "map", stamp: rosTimestamp() },
        pose: {
          position: { x: origin.x, y: origin.y, z: origin.z },
          orientation: orientationFromYaw(yaw),
        },
      }),
    );
    setOperatorStatus("目标已发送，等待无运动路径预览。", true);
    return;
  }

  const covariance = new Array(36).fill(0);
  covariance[0] = 0.25;
  covariance[7] = 0.25;
  covariance[14] = 0.04;
  covariance[21] = 0.01;
  covariance[28] = 0.01;
  covariance[35] = 0.06853891909122467;
  initialPoseTopic.publish(
    new ROSLIB.Message({
      header: { frame_id: "map", stamp: rosTimestamp() },
      pose: {
        pose: {
          position: { x: origin.x, y: origin.y, z: origin.z },
          orientation: orientationFromYaw(yaw),
        },
        covariance,
      },
    }),
  );
  setOperatorStatus("初始姿态已发送给桥接节点校验。", true);
}

function beginPlacement(mode) {
  if (!connected) {
    setOperatorStatus("请先连接 ROSBridge。", false);
    return;
  }
  if (!groundObject) {
    setOperatorStatus("尚未收到可通行地面点云。", false);
    return;
  }
  placementMode = mode;
  placementOrigin = null;
  placementYawPoint = null;
  canvas.classList.add("placing");
  placementHelp.classList.add("active");
  placementHelp.textContent =
    mode === "goal"
      ? "在绿色地面上按下并拖动，松开后只规划预览路径。"
      : "在绿色地面上按下并拖动，松开后设置定位初始姿态。";
}

function finishPlacement() {
  placementMode = null;
  placementOrigin = null;
  placementYawPoint = null;
  activePointerId = null;
  controls.enabled = true;
  canvas.classList.remove("placing");
  placementHelp.classList.remove("active");
  placementHelp.textContent = "在绿色地面点上按下并拖动，拖动方向为目标朝向。";
}

canvas.addEventListener("pointerdown", (event) => {
  if (!placementMode || activePointerId !== null) {
    return;
  }
  const picked = pickGroundPoint(event);
  if (!picked) {
    setOperatorStatus("未选中绿色地面点，请放大后重试。", false);
    return;
  }
  activePointerId = event.pointerId;
  placementOrigin = picked;
  placementYawPoint = picked.clone();
  controls.enabled = false;
  canvas.setPointerCapture(event.pointerId);
  updateSelectionVisual(placementMode, placementOrigin, 0);
  event.preventDefault();
});

canvas.addEventListener("pointermove", (event) => {
  if (event.pointerId !== activePointerId || !placementOrigin) {
    return;
  }
  const planePoint = planePointForEvent(event, placementOrigin.z);
  if (planePoint) {
    placementYawPoint = planePoint;
    updateSelectionVisual(placementMode, placementOrigin, yawFromPlacement());
  }
});

canvas.addEventListener("pointerup", (event) => {
  if (event.pointerId !== activePointerId || !placementOrigin) {
    return;
  }
  const mode = placementMode;
  const origin = placementOrigin.clone();
  const yaw = yawFromPlacement();
  updateSelectionVisual(mode, origin, yaw);
  if (canvas.hasPointerCapture(event.pointerId)) {
    canvas.releasePointerCapture(event.pointerId);
  }
  finishPlacement();
  publishPlacement(mode, origin, yaw);
});

canvas.addEventListener("pointercancel", finishPlacement);

function executePreview() {
  if (!executionAllowed || !previewReady || !executePreviewTopic) {
    setOperatorStatus("当前预览未获得执行授权。", false);
    return;
  }
  executePreviewTopic.publish(new ROSLIB.Message({}));
  previewModal.hidden = true;
  setOperatorStatus("执行确认已发送，等待 P2P Action 接受。", true);
}

setInitialPoseButton.addEventListener("click", () => beginPlacement("initial"));
setGoalButton.addEventListener("click", () => beginPlacement("goal"));
executePreviewButton.addEventListener("click", executePreview);
modalExecuteButton.addEventListener("click", executePreview);
modalDismissButton.addEventListener("click", () => {
  previewModal.hidden = true;
  setOperatorStatus("保留路径预览，未启动导航。", true);
});
cancelNavigationButton.addEventListener("click", () => {
  if (!connected || !cancelNavigationTopic) {
    setOperatorStatus("ROSBridge 未连接。", false);
    return;
  }
  cancelNavigationTopic.publish(new ROSLIB.Message({}));
  setOperatorStatus("正在请求取消网页启动的导航。", true);
});
resetViewButton.addEventListener("click", frameMap);
connectButton.addEventListener("click", connectRosbridge);
websocketInput.addEventListener("keydown", (event) => {
  if (event.key === "Enter") {
    connectRosbridge();
  }
});

toggleMap.addEventListener("change", () => {
  if (mapObject) {
    mapObject.visible = toggleMap.checked;
  }
});
toggleGround.addEventListener("change", () => {
  if (groundObject) {
    groundObject.visible = toggleGround.checked;
  }
});
toggleGlobalPath.addEventListener("change", () => {
  if (globalPathObject) {
    globalPathObject.visible = toggleGlobalPath.checked;
  }
  if (previewPathObject) {
    previewPathObject.visible = toggleGlobalPath.checked;
  }
});
toggleLocalPath.addEventListener("change", () => {
  if (localPathObject) {
    localPathObject.visible = toggleLocalPath.checked;
  }
});

function animate() {
  window.requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}

updateConnectionUi();
animate();
connectRosbridge();
