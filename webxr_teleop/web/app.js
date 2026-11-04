// ViperX WebXR client, adapted from StairVLA PiperX teleoperation.
// Streams both Touch controllers to the robot PC over wss:// (same host/port as this page)
// and shows session status sent by webxr_teleop.run.
// The in-headset view follows XLeVR / telegrip (MIT, Copyright (c) 2025 Emil Rofors):
// controller models, RGB axes and a Pos/Rot readout on each controller, headset readout on top.

const $ = (id) => document.getElementById(id);
let ws = null;
let reconnectDelay = 500;
let status = {};
let aidsVisible = false; // gripper ghost + robot base axes (right stick click)
let lastRightStick = false;
let referenceEpoch = 0;
let freshnessToken = null;
let packetSequence = 0;
let statusDirty = false;
let lastStatusRender = -Infinity;

function flushStatus(now = Date.now()) {
  if (!statusDirty || now - lastStatusRender < 100) return;
  statusDirty = false;
  lastStatusRender = now;
  renderStatus();
}

function resetReference() {
  referenceEpoch += 1;
  send({ type: 'reset', reference_epoch: referenceEpoch });
}

// ----------------------------------------------------------------------------- websocket
function connect() {
  ws = new WebSocket(`wss://${location.host}`);
  ws.onopen = () => {
    reconnectDelay = 500;
    setConnection(true);
  };
  ws.onclose = () => {
    ws = null;
    freshnessToken = null;
    setConnection(false);
    setTimeout(connect, reconnectDelay);
    reconnectDelay = Math.min(reconnectDelay * 1.5, 4000);
  };
  ws.onerror = () => {};
  ws.onmessage = (event) => {
    let msg;
    try { msg = JSON.parse(event.data); } catch (e) { return; }
    if (msg.type === 'freshness') freshnessToken = msg.token;
    if (msg.type === 'status') { status = msg; statusDirty = true; }
    if (msg.type === 'haptic') pulse(msg.hand, msg.intensity, msg.duration_ms);
  };
}

function send(message) {
  // Drop frames instead of queueing them if the network stalls: stale poses are useless.
  if (ws && ws.readyState === WebSocket.OPEN && ws.bufferedAmount < 16384) ws.send(JSON.stringify(message));
}

function setConnection(on) {
  const pill = $('connection');
  pill.textContent = on ? 'Connected' : 'Disconnected';
  pill.className = `pill ${on ? 'pill-on' : 'pill-off'}`;
  if (!on) {
    statusDirty = false;
    setStatusText('Robot PC disconnected - reconnecting...', '#ff6060');
    $('recordingHudText').setAttribute('value', 'DISCONNECTED\nRecording status unavailable');
    $('recordingHudText').setAttribute('color', '#ff6060');
  }
}

// ----------------------------------------------------------------------------- status
function formatTime(seconds) {
  if (seconds === undefined || seconds === null) return '';
  const s = Math.max(0, Math.floor(seconds));
  return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}`;
}

function warnings() {
  const out = [];
  if (status.tracking === false) out.push('active controller tracking lost');
  if (status.ik_ok === false) out.push('tracking limited');
  if (status.clipped) out.push('workspace limit');
  if (status.encoder_backlog > 30) out.push('video encoder falling behind');
  if (status.recording_error) out.push(status.recording_error);
  for (const [name, camera] of Object.entries(status.cameras || {})) {
    if (!camera.healthy) out.push(`${name}: camera unavailable`);
  }
  return out;
}

function setFlag(id, on, bad = false) {
  $(id).className = `flag${on ? ' on' : ''}${bad ? ' bad' : ''}`;
}

// Recording status stays visible on the right controller while gripping.
const NEXT = {
  idle: 'A: move home', homing: 'moving home...  B: cancel', ready: 'A: start recording',
  recording: 'A: save   B: discard', saving: 'saving...', stopped: 'done', teleop: 'grip: move',
};

function setStatusText(text, color) {
  const el = $('rightHandHint');
  if (!el) return;
  el.setAttribute('value', text);
  el.setAttribute('color', color);
}

function renderStatus() {
  const phase = status.phase || 'idle';
  const title = (status.title || phase).toUpperCase();
  const recording = phase === 'recording';
  if (status.hardware) {
    $('controlA').textContent = status.record ? 'Start / save episode; after saving, request return to opening.' : 'Dataset recording is disabled.';
    $('controlB').textContent = 'Powered STOP. Abort the active dataset episode.';
    $('controlY').textContent = 'Hold for one second to request sleep.';
    $('recordingNote').textContent = 'Terminal: → starts/saves; ↑ returns to opening; ← discards; Space stops. X aligns before each episode. In recording mode, Escape / Ctrl+C sleeps then exits; an unfinished episode is aborted.';
  }
  const episode = status.record === false ? '' : `${status.saved ?? 0}/${status.target_episodes ?? '-'} saved`;

  // 2D page (browser / monitor)
  $('phase').textContent = title;
  $('phase').className = `phase phase-${phase}`;
  $('timer').textContent = status.progress_text || (recording ? formatTime(status.elapsed_s) : '');
  const progress = $('episodeProgress');
  progress.hidden = !status.record || status.episode_progress == null;
  progress.value = status.episode_progress ?? 0;
  $('hint').textContent = status.hint || '';
  $('episode').textContent = status.record === false ? 'not recording' : `${status.episode || '-'}`;
  $('saved').textContent = status.record === false ? '-' : `${status.saved ?? 0} / ${status.target_episodes ?? '-'}`;
  $('task').textContent = status.task || '-';
  $('message').textContent = [status.message, status.recording_result].filter(Boolean).join(' | ');
  setFlag('flagTracking', !!status.tracking, status.tracking === false);
  setFlag('flagDeadman', !!status.deadman);
  setFlag('flagGripper', !!status.gripper_closed);
  setFlag('flagIk', status.ik_ok !== false, status.ik_ok === false);
  $('flagGripper').textContent = status.gripper_closed ? 'Gripper closed' : 'Gripper open';

  // Headset: persistent progress below the center of view, plus controller hints.
  const head = [title, episode].filter(Boolean).join('   ');
  const warn = warnings().join(' | ');
  const lines = [head];
  if (status.progress_text) lines.push(status.progress_text);
  else if (recording) lines.push(formatTime(status.elapsed_s));
  lines.push(status.hint || NEXT[phase] || '');
  if (warn) lines.push(warn);
  setStatusText(lines.join('\n'), recording ? '#ff5050' : warn ? '#ffb020' : '#ffffff');
  $('recordingHud').setAttribute('visible', !!status.record);
  $('recordingHudText').setAttribute('value', lines.slice(0, 3).join('\n'));
  $('recordingHudText').setAttribute('color', recording ? '#ff7070' : warn ? '#ffb020' : '#ffffff');
}

// ----------------------------------------------------------------------------- controllers
function gamepadOf(handEl) {
  const tracked = handEl && handEl.components && handEl.components['tracked-controls'];
  return tracked && tracked.controller ? tracked.controller.gamepad : null;
}

function pulse(hand, intensity = 0.5, durationMs = 60) {
  const gamepad = gamepadOf($(hand === 'left' ? 'leftHand' : 'rightHand'));
  const actuator = gamepad && gamepad.hapticActuators && gamepad.hapticActuators[0];
  if (actuator && actuator.pulse) actuator.pulse(intensity, durationMs).catch(() => {});
}

function readController(handEl, hand) {
  if (!handEl || !handEl.object3D) return null;
  const gamepad = gamepadOf(handEl);
  if (!gamepad) return null;
  // Read an actual tracked grip pose; a model transform may remain frozen after tracking loss.
  const renderer = $('scene').renderer;
  const frame = renderer && renderer.xr.getFrame();
  const reference = renderer && renderer.xr.getReferenceSpace();
  const session = renderer && renderer.xr.getSession();
  const source = session && Array.from(session.inputSources).find(s => s.handedness === hand);
  const pose = frame && reference && source && source.gripSpace && frame.getPose(source.gripSpace, reference);
  if (!pose || pose.emulatedPosition) return null;
  const p = pose.transform.position;
  const q = pose.transform.orientation;
  const pressed = (i) => !!(gamepad.buttons[i] && gamepad.buttons[i].pressed);
  const value = (i) => (gamepad.buttons[i] ? gamepad.buttons[i].value : 0);
  // WebXR "xr-standard" mapping on Touch controllers: 0 trigger, 1 squeeze, 3 stick press, 4 A/X, 5 B/Y.
  const face1 = pressed(4);
  const face2 = pressed(5);
  const buttons = { squeeze: pressed(1), thumbstick: pressed(3) };
  if (hand === 'left') Object.assign(buttons, { x: face1, y: face2 });
  else Object.assign(buttons, { a: face1, b: face2 });
  return {
    tracked: true,
    position: { x: p.x, y: p.y, z: p.z },
    quaternion: { x: q.x, y: q.y, z: q.z, w: q.w },
    trigger: value(0),
    gripActive: buttons.squeeze,
    buttons,
  };
}

function poseText(object3D) {
  const p = object3D.position;
  const r = object3D.rotation;
  const deg = (v) => (v * 180 / Math.PI).toFixed(0);
  return `Pos: ${p.x.toFixed(2)} ${p.y.toFixed(2)} ${p.z.toFixed(2)}\nRot: ${deg(r.x)} ${deg(r.y)} ${deg(r.z)}`;
}

// ----------------------------------------------------------------------------- axes
// Red = X, green = Y, blue = Z (same colours and sizes as the original XLeVR indicators).
const AXES = [
  { color: '#ff0000', rotation: '0 0 -90', dir: [1, 0, 0] },
  { color: '#00ff00', rotation: '0 0 0', dir: [0, 1, 0] },
  { color: '#0000ff', rotation: '90 0 0', dir: [0, 0, 1] },
];

function addAxes(parent, { length = 0.08, radius = 0.003, tip = 0.015, opacity = 1 } = {}) {
  const material = opacity < 1 ? `material="transparent: true; opacity: ${opacity}"` : '';
  for (const { color, rotation, dir } of AXES) {
    const at = (d) => dir.map((v) => v * d).join(' ');
    parent.insertAdjacentHTML('beforeend', `
      <a-cylinder position="${at(length / 2)}" rotation="${rotation}" height="${length}" radius="${radius}" color="${color}" ${material}></a-cylinder>
      <a-cone position="${at(length + tip / 2)}" rotation="${rotation}" height="${tip}" radius-bottom="${radius * 2.7}" radius-top="0" color="${color}" ${material}></a-cone>`);
  }
}

function setQuaternion(el, q) {
  if (q && q.length === 4) el.object3D.quaternion.set(q[0], q[1], q[2], q[3]);
}

AFRAME.registerComponent('teleop-stream', {
  init() {
    this.left = $('leftHand');
    this.right = $('rightHand');
    this.head = $('head');
    this.ghost = $('gripperGhost');
    this.leftGhost = $('leftGripperGhost');
    this.robotFrame = $('robotFrame');
    addAxes(this.left);
    addAxes(this.right);
    addAxes(this.leftGhost, { length: 0.12, radius: 0.005, tip: 0.02, opacity: 0.45 });
    addAxes(this.ghost, { length: 0.12, radius: 0.005, tip: 0.02, opacity: 0.45 });
    addAxes(this.robotFrame, { length: 0.1, radius: 0.004, tip: 0.018 });
  },
  tick() {
    const left = readController(this.left, 'left');
    const right = readController(this.right, 'right');
    if (right) {
      if (right.buttons.thumbstick && !lastRightStick) aidsVisible = !aidsVisible;
      lastRightStick = right.buttons.thumbstick;
    }
    send({ type: 'controllers', freshness: freshnessToken, sequence: ++packetSequence, reference_epoch: referenceEpoch, t: Date.now(), left, right });
    // Send tracking first. Text geometry and DOM work need not run at XR rate.
    const now = Date.now();
    flushStatus(now);
    if (this.lastPoseText == null || now - this.lastPoseText >= 100) {
      this.lastPoseText = now;
      $('leftHandInfo').setAttribute('value', poseText(this.left.object3D));
      $('rightHandInfo').setAttribute('value', poseText(this.right.object3D));
      $('headsetInfo').setAttribute('value', poseText(this.head.object3D));
    }
    // Keep recording progress visible during teleoperation.
    const gripping = !!(right && right.buttons.squeeze);
    $('leftHandHint').object3D.visible = !gripping;
    $('rightHandHint').object3D.visible = !!status.record || !gripping;
    this.updateAids(left, right);
  },
  updateAids(left, right) {
    // Optional alignment aids: the real gripper orientation (translucent) drawn at the right controller,
    // and the robot base axes as currently mapped above the left controller.
    this.leftGhost.object3D.visible = aidsVisible && !!left && !!status.left_gripper_frame_q;
    if (this.leftGhost.object3D.visible) {
      this.leftGhost.object3D.position.copy(this.left.object3D.position);
      setQuaternion(this.leftGhost, status.left_gripper_frame_q);
    }
    const showGhost = aidsVisible && !!right && !!status.gripper_frame_q;
    const showRobot = aidsVisible && !!left && !!status.robot_frame_q;
    this.ghost.object3D.visible = showGhost;
    this.robotFrame.object3D.visible = showRobot;
    if (showGhost) {
      this.ghost.object3D.position.copy(this.right.object3D.position);
      setQuaternion(this.ghost, status.gripper_frame_q);
    }
    if (showRobot) {
      this.robotFrame.object3D.position.copy(this.left.object3D.position);
      this.robotFrame.object3D.position.y += 0.1;
      setQuaternion(this.robotFrame, status.robot_frame_q);
    }
  },
});

// ----------------------------------------------------------------------------- XR session
async function setupEnterButton() {
  const button = $('enterXr');
  const note = $('xrNote');
  const scene = $('scene');
  if (!navigator.xr) {
    note.textContent = 'WebXR is not available in this browser. Open this page in the Meta Quest browser.';
    return;
  }
  const vrButton = $('enterVr');
  const [ar, vr] = await Promise.all([
    navigator.xr.isSessionSupported('immersive-ar').catch(() => false),
    navigator.xr.isSessionSupported('immersive-vr').catch(() => false),
  ]);
  button.disabled = !ar;
  vrButton.disabled = !vr;
  button.textContent = 'Enter passthrough';
  note.textContent = ar
    ? 'Passthrough shows the real room and robot with controller controls and status overlaid.'
    : 'Passthrough is unavailable. Use Meta Quest Browser over HTTPS. VR hides the real room.';
  function bindEntry(control, passthrough) {
    control.onclick = async () => {
      button.disabled = true;
      vrButton.disabled = true;
      const label = passthrough ? 'passthrough' : 'VR';
      try {
        // Invoke directly from the user gesture. Never silently fall back
        // from passthrough to a mode that hides the operator's surroundings.
        if (passthrough) await scene.enterAR();
        else await scene.enterVR();
        note.textContent = `${label} active. Release active grips and press X to align when ready.`;
      } catch (err) {
        note.textContent = `Could not enter ${label}: ${err.message || err}. Close other immersive sessions, then retry.`;
      } finally {
        button.disabled = !ar;
        vrButton.disabled = !vr;
      }
    };
  }
  if (ar) bindEntry(button, true);
  if (vr) bindEntry(vrButton, false);
}

document.addEventListener('DOMContentLoaded', () => {
  // Also update the desktop page when no XR animation loop is running.
  setInterval(flushStatus, 100);
  const scene = $('scene');
  const attach = () => {
    scene.setAttribute('teleop-stream', '');
    setupEnterButton();
  };
  if (scene.hasLoaded) attach(); else scene.addEventListener('loaded', attach);
  scene.addEventListener('enter-vr', () => {
    resetReference();
    const ref = scene.renderer.xr.getReferenceSpace();
    if (ref) ref.addEventListener('reset', resetReference);
  });
  scene.addEventListener('exit-vr', resetReference);
  connect();
});
