#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <MD_MAX72xx.h>
#include <SPI.h>
#include <math.h>

// =====================================================
// ESP32 LOCAL WI-FI HOTSPOT
// =====================================================
// Connect the PHONE to this Wi-Fi network, then open
// http://192.168.4.1 in the phone browser.
// The phone screen becomes the second liquid container.

const char* AP_SSID = "ESP32_LIQUID";
const char* AP_PASSWORD = "liquid123"; // 8+ characters

IPAddress apIP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);

// =====================================================
// MAX7219 CONFIGURATION
// =====================================================

#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 4

#define DATA_PIN 23
#define CLK_PIN 18
#define CS_PIN 5

// Portrait simulation grid: 8 columns x 32 rows. Rotate the FC-16 board clockwise.
#define WIDTH 8
#define HEIGHT 32

MD_MAX72XX matrix(
  HARDWARE_TYPE,
  DATA_PIN,
  CLK_PIN,
  CS_PIN,
  MAX_DEVICES
);

// =====================================================
// MPU6500 CONFIGURATION
// =====================================================

#define SDA_PIN 21
#define SCL_PIN 22

byte MPU_ADDR = 0x68;

// =====================================================
// LIQUID CONFIGURATION
// =====================================================

#define PARTICLE_COUNT 150
#define FRAME_DELAY 55

#define FILTER_ALPHA 0.15
#define ONE_AXIS_DEAD_ZONE_DEG 3.0f
#define POUR_GRAVITY_THRESHOLD -0.18f
#define POUR_DIRECTION_THRESHOLD 0.15f

#define SIDE_SPREAD 20
#define DIAGONAL_CHANCE 55

// One control axis: roll the upright glass left/right in its own plane.
// Use -1.0f here only if the physical sensor is mounted with reversed direction.
#define ONE_AXIS_DIRECTION -1.0f

// =====================================================
// CALIBRATION CONFIGURATION
// =====================================================

#define CALIBRATION_SAMPLES 500
#define CALIBRATION_DELAY 5
#define CALIBRATION_RETRIES 3

// Tighter stability limits prevent a moving/tilted sensor from being
// accidentally accepted as the neutral reference.
#define MAX_CAL_STDDEV 4.5
#define MIN_CONCENTRATION 0.97

float gravityBaseTiltDeg = 0.0f;
float filteredTiltDeg = 0.0f;

float gravityX = 0;
float gravityY = 1;
bool tiltActive = false;
bool sensorCalibrated = false;
bool matrixCalibrationRequested = false;
bool matrixCalibrationRunning = false;

// =====================================================
// LIQUID GRID AND PHONE ROUND-TRIP EVENTS
// =====================================================

bool liquid[HEIGHT][WIDTH];
bool nextLiquid[HEIGHT][WIDTH];

// Round-trip event log. Each outgoing matrix particle has a permanent ID.
// The phone sends that same ID back when its particle exits the phone's top.
// returnedEvents[] prevents duplicate return requests from creating extra LEDs.
#define EVENT_CAPACITY 1024
uint8_t transferColumns[EVENT_CAPACITY];
bool returnedEvents[EVENT_CAPACITY];
uint32_t transferCount = 0;
uint16_t pendingPhoneReturns = 0;

// Matrix particle count is conserved: an outgoing event removes one matrix
// particle; its accepted phone return queues exactly one particle for reinsertion.

// =====================================================
// PHONE WEB PAGE
// =====================================================
// The phone browser uses its own orientation sensor for the second container.
// Matrix events enter the phone at top-middle. Returned event IDs are POSTed
// to /return and inserted into the MAX7219 matrix exactly once.

const char webpage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>Phone Liquid Container</title>
<style>
* { box-sizing: border-box; }
html, body { margin:0; width:100%; height:100%; overflow:hidden; background:#000; color:#eee; font-family:Arial,sans-serif; touch-action:none; }
canvas { position:fixed; inset:0; width:100vw; height:100vh; background:#000; }
#panel { position:fixed; z-index:2; top:env(safe-area-inset-top, 8px); left:0; right:0; padding:9px 12px; text-align:center; pointer-events:none; }
#title { font-size:14px; font-weight:700; letter-spacing:1px; }
#status { margin-top:5px; font-size:11px; line-height:1.35; color:#9c9c9c; }
#counts { margin-top:4px; font-size:11px; color:#aaa; }
#controls { position:fixed; z-index:3; left:0; right:0; bottom:calc(env(safe-area-inset-bottom, 0px) + 8px); display:flex; justify-content:center; gap:7px; padding:0 8px; }
button { border:1px solid #555; border-radius:9px; background:#171717; color:#eee; padding:10px 11px; font-size:11px; font-weight:600; }
button:active { background:#333; }
#hint { position:fixed; z-index:2; bottom:62px; left:12px; right:12px; text-align:center; color:#777; font-size:10px; pointer-events:none; }
</style>
</head>
<body>
<canvas id="screen"></canvas>
<div id="panel">
  <div id="title">PHONE LIQUID CONTAINER</div>
  <div id="status">Connect to ESP32, then enable phone motion.</div>
  <div id="counts">Phone: 0 · Returned to matrix: 0</div>
</div>
<div id="hint">An upright glass: liquid settles at the bottom. Tilt past the rim to pour through either top corner.</div>
<div id="controls">
  <button id="enableBtn">ENABLE / PERMIT MOTION</button>
  <button id="calPhoneBtn">CALIBRATE PHONE</button>
  <button id="calMatrixBtn">CALIBRATE MATRIX</button>
</div>
<script>
'use strict';

const canvas = document.getElementById('screen');
const ctx = canvas.getContext('2d', { alpha:false });
const statusEl = document.getElementById('status');
const countsEl = document.getElementById('counts');
const DOT_RADIUS = 3.6;
const CELL_SPACING = 12.0;
const PHONE_DEAD_ZONE_DEG = 7.0;
const POUR_GRAVITY_THRESHOLD = -0.18;
const POUR_DIRECTION_THRESHOLD = 0.15;
const PHONE_TILT_DIRECTION = -1.0;
const STEP_MS = 85;

let screenWidth = 0, screenHeight = 0, dpr = 1;
let container = { left:0, right:0, top:0, bottom:0, width:0, height:0 };
let gridCols = 0, gridRows = 0, gridLeft = 0, gridTop = 0;
let particles = [];
let particleIds = new Set();
let lastEventId = 0;
let polling = false;
let lastStepAt = 0;
let returnedCount = 0;

// Phone motion sensor state. DeviceOrientation is a fused orientation feed
// (accelerometer + gyroscope on supported phones), not a raw gyro-only stream.
let motionEnabled = false;
let motionSeen = false;
// One-axis sensor value: screen-plane left/right tilt angle, in degrees.
let phoneBaseTiltDeg = null;
let rawPhoneTiltDeg = null;
let rawGamma = null;
let phoneGravityX = 0;
let phoneGravityY = 1;
let phoneTiltActive = false;
let lastMotionAt = 0;

function setStatus(message, warning=false) {
  statusEl.textContent = message;
  statusEl.style.color = warning ? '#ff9b6b' : '#a8a8a8';
}
function wrapAngle(a) { return ((a + 180) % 360 + 360) % 360 - 180; }
function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }

function calculateContainer() {
  // Portrait glass: keep it taller than it is wide, like the vertically
  // mounted 8 x 32 MAX7219 matrix.
  const width = Math.max(176, Math.min(screenWidth - 28, screenWidth * 0.68, 320));
  const top = Math.max(105, Math.min(135, screenHeight * 0.16));
  const bottom = Math.max(top + 220, screenHeight - 76);
  const actualBottom = Math.min(bottom, screenHeight - 62);
  container = {
    left:(screenWidth-width)/2, right:(screenWidth+width)/2,
    top:top, bottom:actualBottom, width:width, height:actualBottom-top
  };
}

function setupGrid() {
  gridCols = Math.max(8, Math.floor((container.width - DOT_RADIUS*2) / CELL_SPACING));
  gridRows = Math.max(10, Math.floor((container.height - DOT_RADIUS*2) / CELL_SPACING));
  gridLeft = container.left + (container.width-gridCols*CELL_SPACING)/2;
  gridTop = container.top + DOT_RADIUS + 4;
}
function cellX(col) { return gridLeft + (col+0.5)*CELL_SPACING; }
function cellY(row) { return gridTop + row*CELL_SPACING; }

function findFreeSpawnCell() {
  const occupied = new Set(particles.map(p=>p.col+','+p.row));
  const middle = Math.floor(gridCols/2);
  for (let row=0; row<gridRows; row++) {
    for (let radius=0; radius<gridCols; radius++) {
      const candidates = radius===0 ? [middle] : [middle-radius, middle+radius];
      for (const col of candidates) {
        if (col<0 || col>=gridCols) continue;
        if (!occupied.has(col+','+row)) return {col,row};
      }
    }
  }
  return null;
}

function resizeCanvas() {
  dpr = Math.min(window.devicePixelRatio || 1, 2);
  screenWidth = window.innerWidth;
  screenHeight = window.innerHeight;
  canvas.width = Math.max(1, Math.floor(screenWidth*dpr));
  canvas.height = Math.max(1, Math.floor(screenHeight*dpr));
  ctx.setTransform(dpr,0,0,dpr,0,0);
  calculateContainer();
  setupGrid();
  // Reposition the existing phone particles without changing their count.
  for (const p of particles) {
    p.x = cellX(p.col);
    p.y = cellY(p.row);
  }
}

function addMatrixParticle(event) {
  const id = Number(event.id);
  if (!Number.isInteger(id) || id<=0 || particleIds.has(id)) return;
  const slot = findFreeSpawnCell();
  if (!slot) { setStatus('Phone container is full. Return some particles to the matrix.', true); return; }
  const entranceX = container.left + container.width/2;
  const entranceY = container.top + DOT_RADIUS + 2;
  particles.push({
    id:id, col:slot.col, row:slot.row,
    x:entranceX, y:entranceY,
    targetX:cellX(slot.col), targetY:cellY(slot.row),
    entering:true, enterAt:performance.now(), returnPending:false, retryAt:0
  });
  particleIds.add(id);
  if (id>lastEventId) lastEventId=id;
}

async function pollESP32() {
  if (polling) return;
  polling = true;
  try {
    const response = await fetch('/state?_='+Date.now(), {cache:'no-store'});
    if (!response.ok) throw new Error('HTTP '+response.status);
    const data = await response.json();
    if (Array.isArray(data.events)) for (const event of data.events) addMatrixParticle(event);
    const matrixText = data.matrixCalibrated ? 'MAX calibration OK' : 'MAX calibration needed';
    const sensorText = motionSeen ? 'Phone motion active' : 'Phone motion not enabled';
    // Android companion app supplies motion through a native JavaScript bridge.
    // A normal browser still needs a secure HTTPS context for device sensors.
    if (!motionSeen && !motionEnabled) {
      if (window.AndroidMotionBridge) {
        setStatus(matrixText+' · Android app connected · Tap ENABLE PHONE MOTION.');
      } else if (!window.isSecureContext) {
        setStatus(matrixText+' · PHONE SENSOR BLOCKED: use the Android companion app or HTTPS.', true);
      } else {
        setStatus(matrixText+' · '+sensorText+' · Tap ENABLE PHONE MOTION.');
      }
    }
    countsEl.textContent = 'Phone: '+particles.length+' · Returned to matrix: '+returnedCount+' · MAX pending: '+(data.pendingReturns||0);
  } catch (error) {
    setStatus('ESP32 connection lost: '+error.message, true);
  } finally {
    polling=false;
    window.setTimeout(pollESP32,180);
  }
}

function applyPhoneTiltAngle(deltaTiltDeg) {
  // A single scalar angle is converted into a 2D gravity vector.
  // There is no independent forward/backward control axis.
  const angle = wrapAngle(deltaTiltDeg * PHONE_TILT_DIRECTION);
  if (Math.abs(angle) < PHONE_DEAD_ZONE_DEG) {
    phoneGravityX = 0;
    phoneGravityY = 1;
    phoneTiltActive = false;
  } else {
    const radians = angle * Math.PI / 180;
    phoneGravityX = Math.sin(radians);
    phoneGravityY = Math.cos(radians);
    phoneTiltActive = true;
  }
}

function handleDeviceOrientation(event) {
  // Browser fallback: gamma is the single left/right tilt angle. Beta is ignored.
  if (event.gamma === null || !Number.isFinite(event.gamma)) return;
  rawGamma = event.gamma;
  rawPhoneTiltDeg = event.gamma;
  lastMotionAt = Date.now();
  motionSeen = true;
  if (phoneBaseTiltDeg === null) {
    setStatus('Phone sensor detected. Hold the phone upright and tap CALIBRATE PHONE.');
    return;
  }
  applyPhoneTiltAngle(wrapAngle(rawPhoneTiltDeg - phoneBaseTiltDeg));
  setStatus('Phone motion active · one-axis left/right tilt · tilt the rim to pour');
}

// Android sends one screen-relative left/right tilt angle in degrees.
// The WebView does not receive independent X/Y control values.
window.receiveNativeTilt = function(tiltDeg) {
  tiltDeg = Number(tiltDeg);
  if (!Number.isFinite(tiltDeg)) return;
  rawPhoneTiltDeg = tiltDeg;
  lastMotionAt = Date.now();
  motionSeen = true;
  if (phoneBaseTiltDeg === null) {
    setStatus('Phone tilt detected. Hold the phone upright and tap CALIBRATE PHONE.');
    return;
  }
  applyPhoneTiltAngle(wrapAngle(rawPhoneTiltDeg - phoneBaseTiltDeg));
  setStatus('Phone motion active · one-axis left/right tilt · tilt the rim to pour');
};

// Compatibility fallback for a browser that supports DeviceOrientation over HTTPS.
window.receiveNativeOrientation = function(beta, gamma) {
  handleDeviceOrientation({ beta: Number(beta), gamma: Number(gamma) });
};
window.receiveNativeMotionStatus = function(message, warning) {
  setStatus(String(message), !!warning);
};

async function enablePhoneMotion() {
  // Native Android companion path: uses SensorManager, so no browser HTTPS
  // sensor permission prompt is required on the ESP32 local hotspot.
  if (window.AndroidMotionBridge) {
    motionEnabled = true;
    motionSeen = false;
    rawGamma = null;
    rawPhoneTiltDeg = null;
    phoneBaseTiltDeg = null;
    setStatus('Starting Android one-axis left/right tilt sensor…');
    try {
      window.AndroidMotionBridge.startMotion();
      window.setTimeout(() => {
        if (motionEnabled && !motionSeen) {
          setStatus('Android app connected, but no rotation-vector sensor data arrived. Check that this phone has motion sensors.', true);
        }
      }, 5000);
    } catch (error) {
      motionEnabled = false;
      setStatus('Could not start Android motion sensors: ' + String(error), true);
    }
    return;
  }

  // Browser-only fallback: device orientation requires a secure context.
  if (!window.isSecureContext) {
    motionEnabled = false;
    setStatus('PHONE SENSOR BLOCKED BY HTTP. This page is http://192.168.4.1, not HTTPS. The browser will not show a motion permission prompt here. Use a trusted HTTPS page or a native phone companion app.', true);
    return;
  }

  try {
    if (!('DeviceOrientationEvent' in window)) {
      setStatus('This browser does not support DeviceOrientationEvent. Try Safari on iPhone or Chrome on Android using HTTPS.', true);
      return;
    }

    // iOS Safari asks for permission. This call MUST stay inside the button
    // click handler. Android Chrome commonly has no requestPermission method
    // and may start sensor events without displaying a separate prompt.
    if (typeof DeviceOrientationEvent.requestPermission === 'function') {
      const permission = await DeviceOrientationEvent.requestPermission();
      if (permission !== 'granted') throw new Error('Orientation permission was denied. Allow Motion & Orientation Access in browser settings.');
    }

    // Only DeviceOrientation is used by this sketch; do not request the
    // separate DeviceMotion permission unnecessarily.
    window.removeEventListener('deviceorientation', handleDeviceOrientation, true);
    motionSeen = false;
    rawGamma = null;
    rawPhoneTiltDeg = null;
    phoneBaseTiltDeg = null;
    window.addEventListener('deviceorientation', handleDeviceOrientation, true);
    motionEnabled = true;
    setStatus('Motion listener enabled. Waiting for left/right tilt data…');

    window.setTimeout(() => {
      if (motionEnabled && !motionSeen) {
        setStatus('No phone sensor data received. Check browser sensor permissions, use the page in Safari/Chrome (not an in-app browser), and keep the page open over HTTPS.', true);
      }
    }, 5000);
  } catch (error) {
    motionEnabled = false;
    setStatus('Could not enable phone motion: ' + (error && error.message ? error.message : String(error)), true);
  }
}

function calibratePhone() {
  if (!window.isSecureContext && !window.AndroidMotionBridge) {
    setStatus('Phone calibration is unavailable because this page is HTTP. Use the Android companion app or HTTPS.', true);
    return;
  }
  if (rawPhoneTiltDeg === null || !motionSeen) {
    setStatus('Phone calibration is waiting for live sensor readings. Tap ENABLE PHONE MOTION first.', true);
    return;
  }
  phoneBaseTiltDeg = rawPhoneTiltDeg;
  phoneGravityX = 0;
  phoneGravityY = 1;
  phoneTiltActive = false;
  setStatus('Phone calibrated. Only left/right tilt controls the liquid.');
}

async function calibrateMatrix() {
  try {
    setStatus('Matrix calibration requested. Hold the MAX matrix vertically upright and still for about 5 seconds…');
    const response=await fetch('/matrix-calibrate',{method:'POST',cache:'no-store'});
    if (!response.ok) throw new Error('HTTP '+response.status);
    setStatus('Matrix calibration started. Keep the MAX matrix vertically upright and still.');
  } catch (error) {
    setStatus('Could not start matrix calibration: '+error.message,true);
  }
}

document.getElementById('enableBtn').addEventListener('click',enablePhoneMotion);
document.getElementById('calPhoneBtn').addEventListener('click',calibratePhone);
document.getElementById('calMatrixBtn').addEventListener('click',calibrateMatrix);

function buildOccupancy() {
  const occ=Array.from({length:gridRows},()=>new Array(gridCols).fill(null));
  for (const p of particles) {
    if (p.col>=0 && p.col<gridCols && p.row>=0 && p.row<gridRows) occ[p.row][p.col]=p;
  }
  return occ;
}

async function sendParticleBackToMatrix(p) {
  if (p.returnPending) return;
  p.returnPending=true;
  try {
    // The ID is the original matrix->phone event ID. ESP32 accepts each ID once.
    const response=await fetch('/return',{method:'POST',headers:{'Content-Type':'text/plain'},body:String(p.id),cache:'no-store'});
    const result=await response.json();
    if (!response.ok || !result.ok) throw new Error(result.error||('HTTP '+response.status));
    particles=particles.filter(item=>item.id!==p.id);
    particleIds.delete(p.id);
    returnedCount++;
    setStatus('Particle returned to MAX matrix.');
  } catch (error) {
    p.returnPending=false;
    p.retryAt=performance.now()+700;
    setStatus('Return not confirmed; retrying particle '+p.id+'.',true);
  }
}

function tryMovePhoneParticle(p, newCol, newRow, occ) {
  // PHONE RETURN OUTLETS: top-left and top-right CORNERS ONLY.
  // A particle leaves the phone only when it is at one of those corners
  // and the phone is tilted so gravity points upward. The rest of the
  // phone's top edge remains a wall.
  if (newRow < 0) {
    const atTopLeftCorner = (p.row === 0 && p.col === 0);
    const atTopRightCorner = (p.row === 0 && p.col === gridCols - 1);
    const pouringLeft = phoneGravityX < -POUR_DIRECTION_THRESHOLD && phoneGravityY < POUR_GRAVITY_THRESHOLD;
    const pouringRight = phoneGravityX > POUR_DIRECTION_THRESHOLD && phoneGravityY < POUR_GRAVITY_THRESHOLD;
    if ((atTopLeftCorner && pouringLeft) || (atTopRightCorner && pouringRight)) {
      sendParticleBackToMatrix(p);
      return true;
    }
    return false;
  }
  if (newCol<0 || newCol>=gridCols || newRow>=gridRows) return false;
  if (occ[newRow][newCol]) return false;
  occ[p.row][p.col]=null;
  p.col=newCol;
  p.row=newRow;
  occ[p.row][p.col]=p;
  return true;
}

function updatePhonePhysics(now) {
  if (now-lastStepAt<STEP_MS) return;
  lastStepAt=now;
  const occ=buildOccupancy();
  const dx=Math.abs(phoneGravityX)>0.18 ? Math.sign(phoneGravityX) : 0;
  const dy=Math.abs(phoneGravityY)>0.18 ? Math.sign(phoneGravityY) : 0;
  // Process particles from the gravity-facing side so they do not overlap.
  const ordered=particles.slice().sort((a,b)=>{
    const rowOrder=dy>0 ? b.row-a.row : (dy<0 ? a.row-b.row : 0);
    if (rowOrder!==0) return rowOrder;
    return dx>0 ? b.col-a.col : a.col-b.col;
  });
  for (const p of ordered) {
    if (p.returnPending) continue;
    if (p.entering) {
      const t=clamp((now-p.enterAt)/420,0,1);
      p.x=p.x+(p.targetX-p.x)*Math.min(1,0.18+t*0.55);
      p.y=p.y+(p.targetY-p.y)*Math.min(1,0.18+t*0.55);
      if (t>=1) { p.x=p.targetX; p.y=p.targetY; p.entering=false; }
      continue;
    }
    if (p.retryAt && now<p.retryAt) continue;
    let moved=false;
    if (dx!==0 && dy!==0) moved=tryMovePhoneParticle(p,p.col+dx,p.row+dy,occ);
    if (!moved && dx!==0) moved=tryMovePhoneParticle(p,p.col+dx,p.row,occ);
    if (!moved && dy!==0) moved=tryMovePhoneParticle(p,p.col,p.row+dy,occ);
    // Granular roll: try a diagonal into an open neighbour when directly
    // downhill is blocked. Sideways spreading is small and deterministic.
    if (!moved && dy!==0) {
      const first=(p.id%2===0)?-1:1;
      moved=tryMovePhoneParticle(p,p.col+first,p.row+dy,occ);
      if (!moved) moved=tryMovePhoneParticle(p,p.col-first,p.row+dy,occ);
    }
    if (!moved && dx!==0) {
      moved=tryMovePhoneParticle(p,p.col+dx,p.row-1,occ);
    }
  }
  for (const p of particles) {
    if (!p.returnPending) { p.targetX=cellX(p.col); p.targetY=cellY(p.row); }
  }
}

function drawContainer() {
  // The outline is a glass: solid side walls and base, with two corner
  // pour lips and a separate top-middle inlet for matrix particles.
  const middle=(container.left+container.right)/2;
  const inletGap=Math.max(48,container.width*0.20);
  const outletGap=Math.max(12,container.width*0.055);
  ctx.strokeStyle='#343434';
  ctx.lineWidth=2;
  ctx.beginPath();
  ctx.moveTo(container.left,container.top+outletGap);
  ctx.lineTo(container.left,container.bottom);
  ctx.lineTo(container.right,container.bottom);
  ctx.lineTo(container.right,container.top+outletGap);
  ctx.moveTo(container.left+outletGap,container.top);
  ctx.lineTo(middle-inletGap/2,container.top);
  ctx.moveTo(middle+inletGap/2,container.top);
  ctx.lineTo(container.right-outletGap,container.top);
  ctx.stroke();

  // Red corner lips are the phone-to-matrix outlets; the blue middle lip
  // marks the matrix-to-phone inlet.
  ctx.lineWidth=3;
  ctx.strokeStyle='#b23a3a';
  ctx.beginPath();
  ctx.moveTo(container.left,container.top+outletGap);
  ctx.lineTo(container.left+outletGap,container.top);
  ctx.moveTo(container.right-outletGap,container.top);
  ctx.lineTo(container.right,container.top+outletGap);
  ctx.stroke();
  ctx.strokeStyle='#4d86b8';
  ctx.beginPath();
  ctx.moveTo(middle-inletGap/2+3,container.top);
  ctx.lineTo(middle+inletGap/2-3,container.top);
  ctx.stroke();
}

function render() {
  ctx.fillStyle='#000';
  ctx.fillRect(0,0,screenWidth,screenHeight);
  drawContainer();
  updatePhonePhysics(performance.now());
  ctx.fillStyle='#ff2020';
  for (const p of particles) {
    if (p.entering) {
      const t=clamp((performance.now()-p.enterAt)/420,0,1);
      p.x=p.x+(p.targetX-p.x)*Math.min(1,0.18+t*0.55);
      p.y=p.y+(p.targetY-p.y)*Math.min(1,0.18+t*0.55);
    } else {
      p.x+=(p.targetX-p.x)*0.45;
      p.y+=(p.targetY-p.y)*0.45;
    }
    ctx.beginPath();
    ctx.arc(p.x,p.y,DOT_RADIUS,0,Math.PI*2);
    ctx.fill();
  }
  requestAnimationFrame(render);
}

window.addEventListener('resize',resizeCanvas);
resizeCanvas();
render();
pollESP32();
if (window.AndroidMotionBridge) {
  setStatus('Android companion connected. Hold the phone upright, tap ENABLE PHONE MOTION, then calibrate it.');
} else if (!window.isSecureContext) {
  setStatus('PHONE SENSOR BLOCKED: this ESP32 hotspot page is HTTP. Open it in the Android companion app or use HTTPS.',true);
} else {
  setStatus('ESP32 connected. Hold the phone upright, tap ENABLE PHONE MOTION, then calibrate it.');
}
</script>
</body>
</html>
)rawliteral";

// =====================================================
// MPU6500 REGISTER FUNCTIONS
// =====================================================

bool writeRegister(byte reg, byte value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readBytes(byte reg, byte count, byte *buffer) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom((uint8_t)MPU_ADDR, count) != count) {
    while (Wire.available()) Wire.read();
    return false;
  }

  for (byte i = 0; i < count; i++) {
    buffer[i] = Wire.read();
  }

  return true;
}

// =====================================================
// INITIALIZE MPU6500
// =====================================================

bool initializeMPU() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  delay(200);

  byte id = 0;
  byte addresses[] = {0x68, 0x69};
  bool found = false;

  Serial.println("Searching for MPU6500...");

  for (byte i = 0; i < 2; i++) {
    MPU_ADDR = addresses[i];

    if (readBytes(0x75, 1, &id)) {
      Serial.print("MPU address: 0x");
      Serial.println(MPU_ADDR, HEX);
      Serial.print("WHO_AM_I: 0x");
      Serial.println(id, HEX);

      if (id == 0x70) {
        found = true;
        break;
      }
    }
  }

  if (!found) {
    Serial.println("MPU6500 NOT FOUND! Check SDA/SCL/VCC/GND.");
    return false;
  }

  if (!writeRegister(0x6B, 0x00)) return false; // Wake up
  delay(100);
  if (!writeRegister(0x1C, 0x00)) return false; // Accelerometer +/-2g
  if (!writeRegister(0x1B, 0x00)) return false; // Gyroscope +/-250 dps
  if (!writeRegister(0x1A, 0x03)) return false; // DLPF

  Serial.println("MPU6500 initialized.");
  return true;
}

// =====================================================
// READ SENSOR ANGLES
// =====================================================

bool readPlaneGravity(float &gx, float &gy) {
  // The matrix is mounted in portrait, with the MPU6500 PCB parallel to
  // the display. Its X/Y accelerometer axes therefore lie in the glass plane.
  // At rest the accelerometer measures specific force (opposite physical gravity).
  byte data[6];
  if (!readBytes(0x3B, 6, data)) return false;

  int16_t rawAX = (int16_t)((data[0] << 8) | data[1]);
  int16_t rawAY = (int16_t)((data[2] << 8) | data[3]);
  float ax = rawAX / 16384.0f;
  float ay = rawAY / 16384.0f;
  float magnitude = sqrtf(ax * ax + ay * ay);

  // A vertical glass has gravity projected into the display plane. If the
  // matrix is laid flat, that projection becomes too small to calibrate.
  if (magnitude < 0.30f) return false;
  gx = ax / magnitude;
  gy = ay / magnitude;
  return true;
}

// =====================================================
// ANGLE UTILITIES
// =====================================================

float wrapAngle(float angle) {
  return atan2(
    sin(angle * PI / 180.0),
    cos(angle * PI / 180.0)
  ) * 180.0 / PI;
}

float angleDifference(float a, float b) {
  return wrapAngle(a - b);
}

// =====================================================
// SENSOR CALIBRATION
// =====================================================

bool calibrateSensor() {
  const bool previouslyCalibrated = sensorCalibrated;
  matrixCalibrationRunning = true;

  Serial.println();
  Serial.println("MAX MATRIX ONE-AXIS CALIBRATION");
  Serial.println("Hold the matrix vertically upright and still.");
  Serial.println("Only the in-plane left/right tilt axis will control the liquid.");

  for (int attempt = 1; attempt <= CALIBRATION_RETRIES; attempt++) {
    Serial.print("Calibration attempt ");
    Serial.print(attempt);
    Serial.print("/");
    Serial.println(CALIBRATION_RETRIES);
    delay(500);

    float sumSin = 0.0f;
    float sumCos = 0.0f;
    bool readOK = true;

    // Average a single angle: atan2(screen-right gravity, screen-down gravity).
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
      float gx, gy;
      if (!readPlaneGravity(gx, gy)) {
        readOK = false;
        break;
      }
      float angle = atan2f(gx, gy);
      sumSin += sinf(angle);
      sumCos += cosf(angle);
      delay(CALIBRATION_DELAY);
    }

    if (!readOK) {
      Serial.println("Gravity projection too small or sensor read failed. Keep the matrix vertical and retrying.");
      continue;
    }

    float concentration = hypotf(sumSin, sumCos) / CALIBRATION_SAMPLES;
    float candidateAngle = atan2f(sumSin, sumCos) * 180.0f / PI;
    if (!isfinite(candidateAngle)) {
      Serial.println("Could not determine upright tilt reference; retrying.");
      continue;
    }

    // Verify that the matrix remained steady around the candidate angle.
    float angleVariance = 0.0f;
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
      float gx, gy;
      if (!readPlaneGravity(gx, gy)) {
        readOK = false;
        break;
      }
      float sampleAngle = atan2f(gx, gy) * 180.0f / PI;
      float deviation = angleDifference(sampleAngle, candidateAngle);
      angleVariance += deviation * deviation;
      delay(CALIBRATION_DELAY);
    }

    if (!readOK) {
      Serial.println("Sensor read failed during calibration verification; retrying.");
      continue;
    }

    float directionDeviation = sqrtf(angleVariance / CALIBRATION_SAMPLES);
    Serial.print("One-axis upright angle: ");
    Serial.println(candidateAngle, 2);
    Serial.print("Direction deviation: ");
    Serial.println(directionDeviation, 2);
    Serial.print("Stability concentration: ");
    Serial.println(concentration, 4);

    bool stable = concentration >= MIN_CONCENTRATION &&
                  directionDeviation <= MAX_CAL_STDDEV;

    if (stable) {
      gravityBaseTiltDeg = candidateAngle;
      filteredTiltDeg = 0.0f;
      gravityX = 0.0f;
      gravityY = 1.0f;
      tiltActive = false;
      sensorCalibrated = true;
      matrixCalibrationRunning = false;
      Serial.println("ONE-AXIS MATRIX CALIBRATION SUCCESSFUL.");
      return true;
    }

    Serial.println("Matrix moved during calibration. Hold it vertically and still; retrying...");
  }

  sensorCalibrated = previouslyCalibrated;
  matrixCalibrationRunning = false;
  Serial.println("CALIBRATION FAILED after retries. Previous reference was preserved.");
  return false;
}

// =====================================================
// UPDATE GRAVITY USING ONE LEFT/RIGHT TILT AXIS
// =====================================================

void updateTilt() {
  float sensorGX, sensorGY;
  if (!readPlaneGravity(sensorGX, sensorGY)) return;

  // Collapse the screen-plane gravity reading into ONE angle. The independent
  // out-of-plane axis is not used. Upright is 0 degrees; gravity points down.
  float currentTiltDeg = atan2f(sensorGX, sensorGY) * 180.0f / PI;
  float targetTiltDeg = angleDifference(currentTiltDeg, gravityBaseTiltDeg);
  targetTiltDeg *= ONE_AXIS_DIRECTION;

  // Filter the circular angle without introducing a wraparound jump.
  filteredTiltDeg += FILTER_ALPHA * angleDifference(targetTiltDeg, filteredTiltDeg);
  if (fabsf(filteredTiltDeg) < ONE_AXIS_DEAD_ZONE_DEG) filteredTiltDeg = 0.0f;

  float radians = filteredTiltDeg * PI / 180.0f;
  gravityX = sinf(radians);
  gravityY = cosf(radians);
  float magnitude = hypotf(gravityX, gravityY);
  if (magnitude > 0.001f) {
    gravityX /= magnitude;
    gravityY /= magnitude;
  }

  tiltActive = fabsf(filteredTiltDeg) >= ONE_AXIS_DEAD_ZONE_DEG;
}

// =====================================================
// INITIALIZE LIQUID
// =====================================================

void initializeLiquid() {
  memset(liquid, 0, sizeof(liquid));
  memset(nextLiquid, 0, sizeof(nextLiquid));

  // Start as a settled pool at the bottom of the portrait 8-column x 32-row glass.
  // Fill complete rows first, then distribute the remaining particles
  // across the next row. This avoids a random cloud at startup.
  int count = 0;
  for (int y = HEIGHT - 1; y >= 0 && count < PARTICLE_COUNT; y--) {
    for (int x = 0; x < WIDTH && count < PARTICLE_COUNT; x++) {
      liquid[y][x] = true;
      count++;
    }
  }

  Serial.print("Settled particles initialized: ");
  Serial.println(count);
  Serial.println("Container: 8 columns x 32 rows (portrait glass), particles settle at bottom");
}

// =====================================================
// PARTICLE MOVEMENT + EDGE TRANSFER
// =====================================================

bool tryMove(int x, int y, int nx, int ny) {
  // THE TWO OUTLETS ARE THE TOP-LEFT AND TOP-RIGHT CORNERS ONLY.
  // The upright glass keeps every other edge cell as a wall. Once gravity
  // points through the open rim, a particle at either top corner pours out.
  // Both outlets feed the phone glass at its top-middle inlet.
  bool topLeftExit  = (x == 0 && y == 0 && ny < 0 && gravityX < -POUR_DIRECTION_THRESHOLD);
  bool topRightExit = (x == WIDTH - 1 && y == 0 && ny < 0 && gravityX > POUR_DIRECTION_THRESHOLD);
  if ((topLeftExit || topRightExit) && gravityY < POUR_GRAVITY_THRESHOLD) {
    if (!nextLiquid[y][x]) return false;

    nextLiquid[y][x] = false;
    if (transferCount >= EVENT_CAPACITY) {
      // Do not remove a particle unless there is room to log the event.
      nextLiquid[y][x] = true;
      return false;
    }
    transferColumns[transferCount] = (uint8_t)x;
    returnedEvents[transferCount] = false;
    transferCount++;
    return true;
  }

  // Side walls, base, and the top rim away from its two corner outlets are solid glass.
  if (nx < 0 || nx >= WIDTH || ny < 0 || ny >= HEIGHT) return false;
  if (!nextLiquid[y][x]) return false;
  if (nextLiquid[ny][nx]) return false;

  nextLiquid[ny][nx] = true;
  nextLiquid[y][x] = false;
  return true;
}

// =====================================================
// NO PARTICLE RESPAWNING
// =====================================================
// A particle that transfers to the phone is removed from the matrix.
// When that same phone particle reaches the phone's top outlet, the phone
// returns its original event ID and the ESP32 reinserts exactly one matrix cell.

// =====================================================
// REINSERT PHONE PARTICLES INTO THE MATRIX
// =====================================================

bool findMatrixReturnCell(int &outX, int &outY) {
  const int middle = WIDTH / 2;
  for (int y = 0; y < HEIGHT; y++) {
    for (int distance = 0; distance < WIDTH; distance++) {
      int candidates[2] = { middle - distance, middle + distance };
      for (int k = 0; k < (distance == 0 ? 1 : 2); k++) {
        int x = candidates[k];
        if (x < 0 || x >= WIDTH) continue;
        if (!liquid[y][x]) {
          outX = x;
          outY = y;
          return true;
        }
      }
    }
  }
  return false;
}

void processPhoneReturns() {
  // One accepted phone return becomes exactly one occupied MAX7219 cell.
  // Search from the top-middle inlet outward, then down if the top row fills.
  while (pendingPhoneReturns > 0) {
    int x, y;
    if (!findMatrixReturnCell(x, y)) break;
    liquid[y][x] = true;
    pendingPhoneReturns--;
  }
}

uint16_t countMatrixParticles() {
  uint16_t count = 0;
  for (int y = 0; y < HEIGHT; y++)
    for (int x = 0; x < WIDTH; x++)
      if (liquid[y][x]) count++;
  return count;
}

// =====================================================
// GRAVITY SIMULATION
// =====================================================

void simulateLiquid() {
  memcpy(nextLiquid, liquid, sizeof(liquid));

  float gx = gravityX;
  float gy = gravityY;

  int sx = gx >= 0 ? 1 : -1;
  int sy = gy >= 0 ? 1 : -1;
  int startX = gx >= 0 ? 1 : 0;

  for (int yi = 0; yi < HEIGHT; yi++) {
    int y = gy >= 0 ? HEIGHT - 1 - yi : yi;

    for (int xi = 0; xi < WIDTH; xi++) {
      int x = startX == 0 ? xi : WIDTH - 1 - xi;
      if (!nextLiquid[y][x]) continue;

      int dx = fabs(gx) > 0.15 ? sx : 0;
      int dy = fabs(gy) > 0.15 ? sy : 0;
      bool moved = false;

      // Main diagonal movement.
      if (dx != 0 && dy != 0) {
        moved = tryMove(x, y, x + dx, y + dy);
      }

      // Horizontal movement.
      if (!moved && dx != 0) {
        moved = tryMove(x, y, x + dx, y);
      }

      // Vertical movement.
      if (!moved && dy != 0) {
        moved = tryMove(x, y, x, y + dy);
      }

      // Side spreading is allowed only during a deliberate tilt.
      // At neutral, particles simply fall and settle without jittering.
      if (!moved && tiltActive && random(100) < SIDE_SPREAD) {
        int side = random(2) ? 1 : -1;
        if (dx != 0) {
          moved = tryMove(x, y, x, y + side);
        } else {
          moved = tryMove(x, y, x + side, y);
        }
      }

      // Alternate diagonal movement is also tilt-only.
      if (!moved && tiltActive && random(100) < DIAGONAL_CHANCE) {
        if (dx != 0 && dy != 0) {
          tryMove(x, y, x + dx, y - dy);
        } else if (dy != 0) {
          int side = random(2) ? 1 : -1;
          tryMove(x, y, x + side, y + dy);
        } else if (dx != 0) {
          int side = random(2) ? 1 : -1;
          tryMove(x, y, x + dx, y + side);
        }
      }
    }
  }

  memcpy(liquid, nextLiquid, sizeof(liquid));

}

// =====================================================
// DISPLAY RENDERING
// =====================================================

void renderLiquid() {
  matrix.clear();

  for (int y = 0; y < HEIGHT; y++) {
    for (int x = 0; x < WIDTH; x++) {
      if (liquid[y][x]) {
        // Reverse ONLY the physical vertical pixel mapping.
        // The simulation still uses y=0 at the top and y=HEIGHT-1 at the
        // bottom, with gravity moving toward increasing y. On the user's
        // portrait-mounted FC-16 board, the previous mapping displayed that
        // logical direction upside down, making falling particles look like
        // they were rising. Reversing the hardware column fixes the display
        // orientation without changing gravity, particle physics, or IDs.
        const uint8_t hardwareColumn = (uint8_t)(HEIGHT - 1 - y);
        matrix.setPoint(x, hardwareColumn, true);
      }
    }
  }

  matrix.update();
}

// =====================================================
// WEB SERVER
// =====================================================

void setupWebServer() {
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", webpage);
  });

  server.on("/state", HTTP_GET, []() {
    // Only particles still on the phone are returned to its page.
    // A particle disappears from this list as soon as its return is accepted.
    uint16_t outstanding = 0;
    for (uint32_t i = 0; i < transferCount; i++) {
      if (!returnedEvents[i]) outstanding++;
    }
    String json;
    json.reserve(80 + outstanding * 24);
    json = "{\"matrixCalibrated\":";
    json += (sensorCalibrated ? "true" : "false");
    json += ",\"matrixParticles\":" + String(countMatrixParticles());
    json += ",\"pendingReturns\":" + String(pendingPhoneReturns);
    json += ",\"events\":[";

    bool first = true;
    for (uint32_t i = 0; i < transferCount; i++) {
      if (returnedEvents[i]) continue;
      if (!first) json += ",";
      first = false;
      json += "{\"id\":" + String(i + 1) + ",\"x\":" + String(transferColumns[i]) + "}";
    }
    json += "]}";

    server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
    server.send(200, "application/json", json);
  });

  server.on("/return", HTTP_POST, []() {
    String body = server.arg("plain");
    body.trim();
    uint32_t id = (uint32_t)strtoul(body.c_str(), nullptr, 10);

    if (id == 0 || id > transferCount) {
      server.send(400, "application/json", "{\"ok\":false,\"error\":\"unknown particle id\"}");
      return;
    }

    uint32_t index = id - 1;
    if (returnedEvents[index]) {
      // Idempotent retry: an earlier request may have succeeded even if its
      // HTTP response was lost. Never enqueue the same particle twice.
      server.send(200, "application/json", "{\"ok\":true,\"duplicate\":true}");
      return;
    }

    if (pendingPhoneReturns >= EVENT_CAPACITY) {
      server.send(503, "application/json", "{\"ok\":false,\"error\":\"return queue full\"}");
      return;
    }

    returnedEvents[index] = true;
    pendingPhoneReturns++;
    server.send(200, "application/json", "{\"ok\":true,\"duplicate\":false}");
  });

  server.on("/matrix-calibrate", HTTP_POST, []() {
    matrixCalibrationRequested = true;
    server.send(202, "application/json", "{\"ok\":true,\"message\":\"calibration queued\"}");
  });

  server.onNotFound([]() {
    server.send(404, "text/plain", "Not found");
  });

  server.begin();
  Serial.println("Phone liquid web server started on port 80.");
}

// =====================================================
// START ESP32 LOCAL WI-FI HOTSPOT
// =====================================================

bool startLocalWiFi() {
  Serial.println();
  Serial.println("Starting ESP32 local Wi-Fi hotspot...");

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);

  if (!WiFi.softAPConfig(apIP, gateway, subnet)) {
    Serial.println("Could not configure hotspot IP.");
    return false;
  }

  // Channel 6, visible SSID, maximum 4 connected clients.
  if (!WiFi.softAP(AP_SSID, AP_PASSWORD, 6, false, 4)) {
    Serial.println("Hotspot creation failed!");
    return false;
  }

  delay(300);

  Serial.println("ESP32 HOTSPOT READY!");
  Serial.print("Wi-Fi name: ");
  Serial.println(AP_SSID);
  Serial.print("Wi-Fi password: ");
  Serial.println(AP_PASSWORD);
  Serial.print("Open on phone browser: http://");
  Serial.println(WiFi.softAPIP());

  return true;
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("ESP32 EXTENDED LIQUID SIMULATION");
  Serial.println("================================");

  randomSeed(esp_random());

  matrix.begin();
  matrix.control(MD_MAX72XX::INTENSITY, 4);
  matrix.clear();
  matrix.update();

  if (!initializeMPU()) {
    Serial.println("MPU6500 NOT FOUND! Check wiring and restart.");
    while (true) delay(1000);
  }

  while (!calibrateSensor()) {
    Serial.println("Keep the MAX matrix vertical and steady. Retrying calibration in 1 second...");
    delay(1000);
  }

  initializeLiquid();

  if (!startLocalWiFi()) {
    Serial.println("Could not start ESP32 hotspot.");
    while (true) delay(1000);
  }

  setupWebServer();

  Serial.println();
  Serial.println("LED MATRIX READY!");
  Serial.println("Portrait glass: 8 columns x 32 rows. One-axis left/right tilt controls gravity.");
  Serial.println("The two top matrix corners pour particles into the phone glass.");
  Serial.println("Phone particles enter at top-middle and settle under one-axis left/right tilt.");
  Serial.println("When a phone particle exits through the phone top, that same ID returns to the MAX matrix.");
  Serial.println("PHONE PAGE READY!");
  Serial.println("Connect phone to ESP32_LIQUID.");
  Serial.println("Then open http://192.168.4.1");
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  server.handleClient();

  if (matrixCalibrationRequested && !matrixCalibrationRunning) {
    matrixCalibrationRequested = false;
    calibrateSensor();
  }

  processPhoneReturns();
  updateTilt();
  simulateLiquid();
  renderLiquid();

  delay(FRAME_DELAY);
}
