/*
Modern telemetry dashboard client
- Connects to WebSocket "/telemetry" (same-origin)
- Expects messages with complete JSON map of telemetry paths -> values
- Gracefully handles reconnects, renders tailored widgets
*/

const wsUrl = (() => {
  const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
  const host = location.host || 'localhost';
  // Default to same origin path; if served from file:// fallback to localhost
  return `${proto}//${host || 'localhost'}/telemetry`;
})();

const els = {
  connStatus: document.getElementById('conn-status'),
  reconnect: document.getElementById('reconnect'),
  kbToggle: document.getElementById('kb-toggle'),
  wsUrlText: document.getElementById('ws-url'),
  lastUpdate: document.getElementById('last-update'),
  motorCount: document.getElementById('motor-count'),
  motors: document.getElementById('motors'),
  rssBar: document.getElementById('rss-bar'),
  rssText: document.getElementById('rss-text'),
  rateText: document.getElementById('rate-text'),
  rateCanvas: document.getElementById('telemetry-rate'),
  fieldSelect: document.getElementById('field-select'),
  robotSelect: document.getElementById('robot-select'),
  fieldImage: document.getElementById('field-image'),
  robotCanvas: document.getElementById('robot-canvas'),
  poseText: document.getElementById('pose-text'),
};

els.wsUrlText.textContent = wsUrl;

// Lightweight sparkline for update rate
class Sparkline {
  constructor(canvas, max = 120) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.max = max; this.data = [];
    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = this.canvas.clientWidth * dpr;
    this.canvas.height = this.canvas.clientHeight * dpr;
    this.ctx.scale(dpr, dpr);
  }
  push(v) { this.data.push(v); if (this.data.length > this.max) this.data.shift(); this.draw(); }
  draw() {
    const { ctx, canvas } = this;
    const w = canvas.clientWidth; const h = canvas.clientHeight;
    ctx.clearRect(0,0,w,h);
    if (this.data.length < 2) return;
    const maxV = Math.max(...this.data, 60);
    const minV = Math.min(...this.data, 0);
    ctx.strokeStyle = '#7c94ff'; ctx.lineWidth = 1.5;
    ctx.beginPath();
    this.data.forEach((v, i) => {
      const x = (i/(this.data.length-1)) * w;
      const y = h - ((v - minV)/(maxV - minV || 1)) * h;
      if (i === 0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
    });
    ctx.stroke();
  }
}
const rateSpark = new Sparkline(els.rateCanvas);

// State
let ws; let lastTickTime = 0; let fpsCounter = { count: 0, windowStart: performance.now() };
const motorCards = new Map(); // key: port -> elements
let keyboardEnabled = false;
let pose = { x: 0, y: 0, rot: 0 };
let fieldConfig = null; let robotConfig = null;
let robotImg = null; // Image for robot rendering
let configs = { fields: [], robots: [] };

function setConnected(on) {
  els.connStatus.textContent = on ? 'Connected' : 'Disconnected';
  els.connStatus.className = `badge ${on ? 'badge-on' : 'badge-off'}`;
}

function formatBytes(n) {
  const units = ['B','KB','MB','GB'];
  let u = 0; let v = n;
  while (v >= 1024 && u < units.length-1) { v /= 1024; u++; }
  return `${v.toFixed(v>=100?0: v>=10?1:2)} ${units[u]}`;
}

function createMotorCard(port) {
  const card = document.createElement('div'); card.className = 'card'; card.dataset.port = String(port);
  card.innerHTML = `
    <div class="title"><div>Motor <strong>#${port}</strong></div><div class="mono">rpm</div></div>
    <div class="value" data-role="velocity">0</div>
    <div class="sub">Velocity (rpm)</div>
    <div class="gauge">
      <div class="fill-left" style="width:0%" data-role="posbar-left"></div>
      <div class="fill-right" style="width:0%" data-role="posbar-right"></div>
    </div>
    <div class="mono" data-role="position">0.000</div>
  `;
  els.motors.appendChild(card);
  motorCards.set(port, {
    root: card,
    velocity: card.querySelector('[data-role="velocity"]'),
    position: card.querySelector('[data-role="position"]'),
    posLeft: card.querySelector('[data-role="posbar-left"]'),
    posRight: card.querySelector('[data-role="posbar-right"]'),
  });
}

function updateMotor(port, velocity, position) {
  if (!motorCards.has(port)) createMotorCard(port);
  const ui = motorCards.get(port);
  ui.velocity.textContent = (velocity ?? 0).toFixed(1);
  ui.position.textContent = (position ?? 0).toFixed(3);
  // Bipolar velocity bar: negatives fill left, positives fill right
  const vel = Number(velocity ?? 0);
  const pct = Math.max(0, Math.min(100, Math.abs(vel)));
  if (vel < 0) {
    ui.posLeft.style.width = `${pct}%`;
    ui.posRight.style.width = `0%`;
  } else {
    ui.posRight.style.width = `${pct}%`;
    ui.posLeft.style.width = `0%`;
  }
}

function updateAxes(map) {
  for (let i=1; i<=4; i++) {
    const key = `controller/Axis${i}`;
    const v = Number(map[key] ?? 0);
    const axisEl = document.querySelector(`.axis[data-axis="Axis${i}"]`);
    if (!axisEl) continue;
    axisEl.querySelector('.axis-value').textContent = `${v.toFixed(0)}%`;
    const left = axisEl.querySelector('.meter-left');
    const right = axisEl.querySelector('.meter-right');
    const pct = Math.max(0, Math.min(100, Math.abs(v)));
    if (v < 0) {
      left.style.width = `${pct}%`;
      right.style.width = `0%`;
    } else {
      right.style.width = `${pct}%`;
      left.style.width = `0%`;
    }
  }
}

function updateButtons(map) {
  // Intentionally ignore telemetry counters for button highlight.
  // Button highlight reflects only the current user's local pointer interactions.
}

function updateSystem(map) {
  const rss = Number(map['memory/rss_bytes'] ?? map['host/rss_bytes'] ?? 0);
  const cap = Number(map['memory/cap_bytes'] ?? map['host/cap_bytes'] ?? (15 * 1024 * 1024));
  if (rss) {
    const pct = Math.max(0, Math.min(100, cap ? (rss / cap) * 100 : 0));
    els.rssBar.style.width = `${pct.toFixed(1)}%`;
    els.rssText.textContent = `${formatBytes(rss)} / ${formatBytes(cap)}`;
  }
}

function updateMotorCount(map) {
  const cnt = Number(map['counters/motor'] ?? 0);
  els.motorCount.textContent = String(cnt);
}

function updatePose(map) {
  const x = Number(map['drivetrain/pose/x']);
  const y = Number(map['drivetrain/pose/y']);
  const r = Number(map['drivetrain/pose/rotation']);
  if (!Number.isNaN(x)) pose.x = x;
  if (!Number.isNaN(y)) pose.y = y;
  if (!Number.isNaN(r)) pose.rot = r;
  if (els.poseText) els.poseText.textContent = `x=${pose.x.toFixed(2)} m, y=${pose.y.toFixed(2)} m, rot=${pose.rot.toFixed(2)} rad`;
  drawRobot();
}

function metersToPixels(xMeters, yMeters) {
  if (!fieldConfig) return { x: 0, y: 0 };
  const { widthMeters, heightMeters, topLeft, bottomRight } = fieldConfig;
  const stageRect = els.robotCanvas.getBoundingClientRect();
  const img = els.fieldImage;
  const imgRect = img.getBoundingClientRect();
  const naturalW = img.naturalWidth || Math.abs(bottomRight[0] - topLeft[0]);
  const naturalH = img.naturalHeight || Math.abs(bottomRight[1] - topLeft[1]);
  // Working area in image pixels
  const workW = Math.abs(bottomRight[0] - topLeft[0]);
  const workH = Math.abs(bottomRight[1] - topLeft[1]);
  // Pixels per meter within the working area
  const sx = workW / widthMeters;
  const sy = workH / heightMeters;
  // Bottom-left origin: x grows right, y grows up
  const pxImg = topLeft[0] + xMeters * sx;
  const pyImg = bottomRight[1] - yMeters * sy;
  // Map from image space to on-screen image rect (object-fit contain) and then to overlay canvas
  const scaleX = (imgRect.width || stageRect.width) / naturalW;
  const scaleY = (imgRect.height || stageRect.height) / naturalH;
  const offsetX = (stageRect.width - imgRect.width) / 2;
  const offsetY = (stageRect.height - imgRect.height) / 2;
  return { x: pxImg * scaleX + offsetX, y: pyImg * scaleY + offsetY };
}

function drawRobot() {
  const ctx = els.robotCanvas.getContext('2d'); if (!ctx || !fieldConfig || !robotConfig) return;
  const rect = els.robotCanvas.getBoundingClientRect();
  els.robotCanvas.width = rect.width * (window.devicePixelRatio || 1);
  els.robotCanvas.height = rect.height * (window.devicePixelRatio || 1);
  ctx.save(); ctx.scale(window.devicePixelRatio || 1, window.devicePixelRatio || 1);
  ctx.clearRect(0,0,rect.width,rect.height);
  const { x, y } = metersToPixels(pose.x, pose.y);
  // Robot dimensions in meters mapped to pixels
  const { widthMeters, heightMeters } = robotConfig;
  const rpW = metersToPixels(widthMeters, 0);
  const rpW0 = metersToPixels(0, 0);
  const rpH = metersToPixels(0, heightMeters);
  const robotPxW = Math.abs(rpW.x - rpW0.x);
  const robotPxH = Math.abs(rpH.y - rpW0.y);
  // Draw robot image centered and rotated CCW
  ctx.translate(x, y);
  // Telemetry rotation is CW-positive; canvas expects CCW-positive
  ctx.rotate(-pose.rot);
  if (robotImg && robotImg.complete) {
    // Preserve aspect ratio of the robot image while fitting into robotPxW x robotPxH
    const rW = robotImg.naturalWidth || robotImg.width;
    const rH = robotImg.naturalHeight || robotImg.height;
    const sx = robotPxW / rW;
    const sy = robotPxH / rH;
    const s = Math.min(sx, sy);
    const drawW = rW * s;
    const drawH = rH * s;
    ctx.drawImage(robotImg, -drawW/2, -drawH/2, drawW, drawH);
  } else {
    // Fallback rectangle if image not ready
    ctx.fillStyle = 'rgba(124,148,255,0.5)';
    ctx.strokeStyle = '#7c94ff';
    ctx.lineWidth = 2;
    ctx.fillRect(-robotPxW/2, -robotPxH/2, robotPxW, robotPxH);
    ctx.strokeRect(-robotPxW/2, -robotPxH/2, robotPxW, robotPxH);
  }
  ctx.restore();
}

async function loadConfig(url) {
  const res = await fetch(url); if (!res.ok) throw new Error(`Failed to load ${url}`);
  return res.json();
}

async function setupFieldConfigs() {
  try {
    const field = await loadConfig('./config/field/vex.json');
    const robot = await loadConfig('./config/robot/bot.json');
    configs.fields = [field]; configs.robots = [robot];
    // Populate selects by name
    els.fieldSelect.innerHTML = configs.fields.map((f,i)=>`<option value="${i}">${f.name}</option>`).join('');
    els.robotSelect.innerHTML = configs.robots.map((r,i)=>`<option value="${i}">${r.name}</option>`).join('');
    els.fieldSelect.addEventListener('change', onFieldChange);
    els.robotSelect.addEventListener('change', onRobotChange);
    // Initialize
    onFieldChange(); onRobotChange();
  } catch (e) { console.warn('Config load failed', e); }
}

function onFieldChange() {
  const idx = Number(els.fieldSelect.value || 0);
  fieldConfig = configs.fields[idx];
  if (!fieldConfig) return;
  els.fieldImage.src = `./config/field/${fieldConfig.image}`;
  // Resize stage aspect ratio to fit field image proportions if known
  els.fieldImage.onload = () => { drawRobot(); };
}

function onRobotChange() {
  const idx = Number(els.robotSelect.value || 0);
  robotConfig = configs.robots[idx];
  // Load robot image from config
  if (robotConfig?.image) {
    robotImg = new Image();
    robotImg.src = `./config/robot/${robotConfig.image}`;
    robotImg.onload = () => drawRobot();
    robotImg.onerror = () => drawRobot();
  } else {
    robotImg = null;
  }
  drawRobot();
}

function handleTelemetry(map) {
  lastTickTime = performance.now();
  fpsCounter.count++;
  const now = performance.now();
  if (now - fpsCounter.windowStart >= 1000) {
    const fps = fpsCounter.count;
    rateSpark.push(fps);
    els.rateText.textContent = `${fps.toFixed(0)} fps`;
    fpsCounter.count = 0; fpsCounter.windowStart = now;
  }
  els.lastUpdate.textContent = new Date().toLocaleTimeString();

  updateAxes(map);
  updateButtons(map);
  updateSystem(map);
  updateMotorCount(map);
  updatePose(map);

  // Motors: detect keys like motor/{port}/velocity, position
  Object.keys(map).forEach(k => {
    const m = k.match(/^motor\/(\d+)\/(velocity|position)$/);
    if (m) {
      const port = Number(m[1]);
      const type = m[2];
      const v = Number(map[k]);
      const velocityKey = `motor/${port}/velocity`;
      const positionKey = `motor/${port}/position`;
      updateMotor(port, Number(map[velocityKey] ?? (type==='velocity'? v: 0)), Number(map[positionKey] ?? (type==='position'? v: 0)));
    }
  });
}

function connect() {
  try { if (ws) ws.close(); } catch {}
  setConnected(false);
  ws = new WebSocket(wsUrl);
  ws.addEventListener('open', () => setConnected(true));
  ws.addEventListener('message', (ev) => {
    try {
      const data = JSON.parse(typeof ev.data === 'string' ? ev.data : new TextDecoder().decode(ev.data));
      if (data && typeof data === 'object') handleTelemetry(data);
    } catch (e) {
      console.warn('Invalid telemetry payload', e);
    }
  });
  ws.addEventListener('close', () => setConnected(false));
  ws.addEventListener('error', () => setConnected(false));
}

// Auto-reconnect with backoff
let backoffMs = 1000;
setInterval(() => {
  const connected = ws && ws.readyState === WebSocket.OPEN;
  if (!connected) {
    connect();
    backoffMs = Math.min(backoffMs * 1.5, 8000);
  } else {
    backoffMs = 1000;
  }
}, backoffMs);

// Manual reconnect
els.reconnect.addEventListener('click', connect);
els.kbToggle?.addEventListener('change', (e) => {
  keyboardEnabled = !!e.target.checked;
});

// Send helpers
function sendAxisUpdate(index, valuePct) {
  const payload = JSON.stringify({ type: 'axis', index, value: Number(valuePct) });
  try { ws?.readyState === WebSocket.OPEN && ws.send(payload); } catch {}
}

function sendButtonUpdate(name, pressed) {
  const payload = JSON.stringify({ type: 'button', name, pressed: !!pressed });
  try { ws?.readyState === WebSocket.OPEN && ws.send(payload); } catch {}
}

// Axis drag interaction
function setupAxisDrag() {
  document.querySelectorAll('.axis').forEach(axisEl => {
    const meter = axisEl.querySelector('.meter');
    const left = axisEl.querySelector('.meter-left');
    const right = axisEl.querySelector('.meter-right');
    const label = axisEl.dataset.axis; // e.g., Axis1
    const index = Number(label?.replace('Axis',''));
    if (!meter || !left || !right || !index) return;
    let dragging = false;
    const updateFromEvent = (clientX) => {
      const rect = meter.getBoundingClientRect();
      const rel = (clientX - rect.left) / rect.width; // 0..1
      const pct = Math.max(-100, Math.min(100, (rel * 200) - 100)); // center is 0
      if (pct < 0) { left.style.width = `${Math.abs(pct)}%`; right.style.width = `0%`; }
      else { right.style.width = `${Math.abs(pct)}%`; left.style.width = `0%`; }
      axisEl.querySelector('.axis-value').textContent = `${pct.toFixed(0)}%`;
      sendAxisUpdate(index, pct);
    };
    meter.addEventListener('pointerdown', (e) => { dragging = true; meter.setPointerCapture(e.pointerId); updateFromEvent(e.clientX); });
    meter.addEventListener('pointermove', (e) => { if (!dragging) return; updateFromEvent(e.clientX); });
    meter.addEventListener('pointerup', (e) => { dragging = false; meter.releasePointerCapture(e.pointerId); });
    meter.addEventListener('pointerleave', () => { dragging = false; });
  });
}

// Button press interaction
function setupButtons() {
  const buttons = document.querySelectorAll('.buttons [data-button]');
  buttons.forEach(btn => {
    const name = btn.getAttribute('data-button');
    btn.addEventListener('pointerdown', (e) => {
      e.preventDefault();
      btn.setAttribute('aria-pressed','true');
      sendButtonUpdate(name, true);
    });
    btn.addEventListener('pointerup', (e) => {
      e.preventDefault();
      btn.setAttribute('aria-pressed','false');
      sendButtonUpdate(name, false);
    });
    btn.addEventListener('pointerleave', () => {
      // Ensure highlight clears when pointer leaves the button area
      if (btn.getAttribute('aria-pressed') === 'true') {
        btn.setAttribute('aria-pressed','false');
        sendButtonUpdate(name, false);
      }
    });
    btn.addEventListener('pointercancel', () => {
      if (btn.getAttribute('aria-pressed') === 'true') {
        btn.setAttribute('aria-pressed','false');
        sendButtonUpdate(name, false);
      }
    });
  });
  // Global release safeguard: if user releases outside the button, clear all highlights
  window.addEventListener('pointerup', () => {
    buttons.forEach(btn => {
      if (btn.getAttribute('aria-pressed') === 'true') {
        btn.setAttribute('aria-pressed','false');
        const name = btn.getAttribute('data-button');
        sendButtonUpdate(name, false);
      }
    });
  });
}

// Keyboard control: WASD -> Axis1/Axis2, IJKL -> Axis3/Axis4
const keyAxisMap = {
  'w': { index: 2, value: +100 }, 's': { index: 2, value: -100 },
  'd': { index: 1, value: +100 }, 'a': { index: 1, value: -100 },
  'i': { index: 3, value: +100 }, 'k': { index: 3, value: -100 },
  'l': { index: 4, value: +100 }, 'j': { index: 4, value: -100 },
};
const downState = new Map();
function setupKeyboard() {
  window.addEventListener('keydown', (e) => {
    if (!keyboardEnabled) return;
    const k = e.key.toLowerCase();
    const m = keyAxisMap[k];
    if (m) { e.preventDefault(); downState.set(m.index, m.value); sendAxisUpdate(m.index, m.value); }
  });
  window.addEventListener('keyup', (e) => {
    if (!keyboardEnabled) return;
    const k = e.key.toLowerCase(); const m = keyAxisMap[k];
    if (m) { e.preventDefault(); downState.delete(m.index); sendAxisUpdate(m.index, 0); }
  });
}

// Initialize UI
(function init() {
  // Pre-create a few motor cards on first sighting via updates,
  // UI builds dynamically in updateMotor
  connect();
  setupAxisDrag();
  setupButtons();
  setupKeyboard();
  setupFieldConfigs();
})();
