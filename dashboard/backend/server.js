const express = require('express');
const cors = require('cors');
const http = require('http');
const path = require('path');
const { Server } = require('socket.io');
const mqtt = require('mqtt');

const app = express();
const PORT = process.env.PORT || 5000;

app.use(cors());
app.use(express.json({ limit: '10mb' }));
app.use(express.static(path.join(__dirname, '../frontend/public')));

const server = http.createServer(app);
const io = new Server(server, {
  cors: { origin: '*' }
});

// ==============================================================================
// AUTERA Heavy Commercial Vehicle Profiles
// ==============================================================================
const VEHICLE_PROFILES = {
  autera_40t_tractor: {
    id: 'autera_40t_tractor',
    name: 'AUTERA 40t Long-Haul Tractor-Trailer',
    category: 'Heavy Commercial Vehicle (Class 8 / N3)',
    massKg: 40000,
    lengthM: 16.5,
    widthM: 2.55,
    heightM: 4.0,
    brakeSystem: 'Dual-Circuit S-Cam Pneumatic Air Brakes',
    airBrakeLagSeconds: 0.40,
    maxDecelMps2: 5.5,
    maxGovernedSpeedKmh: 90,
    wheelbaseM: 6.8,
    provenance: {
      mass: 'CONFIGURED',
      dimensions: 'CONFIGURED',
      brakeLag: 'ASSUMED (ISO 7637-2)',
      maxDecel: 'ASSUMED (UNECE R13)'
    }
  },
  autera_26t_rigid: {
    id: 'autera_26t_rigid',
    name: 'AUTERA 26t Rigid Distribution Truck',
    category: 'Medium-Heavy Commercial Vehicle (N3)',
    massKg: 26000,
    lengthM: 10.2,
    widthM: 2.50,
    heightM: 3.8,
    brakeSystem: 'Pneumatic Air Disc Brakes with ABS/EBS',
    airBrakeLagSeconds: 0.30,
    maxDecelMps2: 6.0,
    maxGovernedSpeedKmh: 90,
    wheelbaseM: 5.4,
    provenance: {
      mass: 'CONFIGURED',
      dimensions: 'CONFIGURED',
      brakeLag: 'ASSUMED',
      maxDecel: 'ASSUMED'
    }
  },
  autera_16t_urban: {
    id: 'autera_16t_urban',
    name: 'AUTERA 16t Urban Delivery Truck',
    category: 'Medium Commercial Vehicle (N2)',
    massKg: 16000,
    lengthM: 8.5,
    widthM: 2.45,
    heightM: 3.4,
    brakeSystem: 'Hydraulic-Over-Air Disc Brakes',
    airBrakeLagSeconds: 0.25,
    maxDecelMps2: 6.8,
    maxGovernedSpeedKmh: 80,
    wheelbaseM: 4.5,
    provenance: {
      mass: 'CONFIGURED',
      dimensions: 'CONFIGURED',
      brakeLag: 'ASSUMED',
      maxDecel: 'ASSUMED'
    }
  },
  test_suv: {
    id: 'test_suv',
    name: 'Standard Autonomous Test Platform (SUV)',
    category: 'Passenger Vehicle (M1)',
    massKg: 2100,
    lengthM: 4.8,
    widthM: 1.95,
    heightM: 1.7,
    brakeSystem: 'Electro-Hydraulic Brake by Wire (iBooster)',
    airBrakeLagSeconds: 0.15,
    maxDecelMps2: 8.5,
    maxGovernedSpeedKmh: 120,
    wheelbaseM: 2.9,
    provenance: {
      mass: 'CONFIGURED',
      dimensions: 'CONFIGURED',
      brakeLag: 'MEASURED',
      maxDecel: 'MEASURED'
    }
  }
};

// ==============================================================================
// AUTERA Standard Validation Scenarios (UNECE R152 / Euro NCAP Aligned)
// ==============================================================================
const VALIDATION_SCENARIOS = {
  'SCN-01': {
    id: 'SCN-01',
    name: 'Urban Cruising Baseline',
    category: 'Normal Operation',
    description: 'Vehicle cruises at 50 km/h in clear lane with lead vehicle maintaining 45m distance. Baseline ADAS integrity test.',
    initialEgoSpeedKmh: 50,
    leadSpeedKmh: 50,
    initialGapM: 45,
    roadFrictionMu: 0.85,
    durationSeconds: 15,
    passCriteria: 'Zero false FCW alarms; ego speed governed within ±2 km/h; zero collisions.',
    type: 'baseline'
  },
  'SCN-02': {
    id: 'SCN-02',
    name: 'Lead Vehicle Deceleration (ACC/FCW)',
    category: 'Forward Collision Warning',
    description: 'Lead vehicle decelerates smoothly from 50 to 20 km/h at 2.5 m/s². Tests ACC headway maintenance and timely FCW alert.',
    initialEgoSpeedKmh: 60,
    leadSpeedKmh: 50,
    initialGapM: 40,
    roadFrictionMu: 0.85,
    durationSeconds: 18,
    passCriteria: 'FCW Advisory triggered at TTC ≥ 2.5s; speed reduced to match lead; min separation ≥ 12m.',
    type: 'fcw'
  },
  'SCN-03': {
    id: 'SCN-03',
    name: 'Stationary Lead Obstacle (AEB Critical)',
    category: 'Automatic Emergency Braking',
    description: 'Ego approaches stationary commercial vehicle at 65 km/h. Tests FCW alert and critical 100% emergency brake actuation.',
    initialEgoSpeedKmh: 65,
    leadSpeedKmh: 0,
    initialGapM: 55,
    roadFrictionMu: 0.85,
    durationSeconds: 12,
    passCriteria: 'Collision avoided (Min separation ≥ 1.5m); AEB commanded at TTC ≤ 1.5s; FCW issued ≥ 0.8s prior to AEB.',
    type: 'aeb'
  },
  'SCN-04': {
    id: 'SCN-04',
    name: 'Highway Cut-In Intrusion',
    category: 'Acute Intrusion',
    description: 'Adjacent vehicle changes lane abruptly into ego path at 14m distance with lower speed (-15 km/h relative).',
    initialEgoSpeedKmh: 80,
    leadSpeedKmh: 65,
    initialGapM: 14,
    roadFrictionMu: 0.85,
    durationSeconds: 10,
    passCriteria: 'Immediate FCW Warning; AEB intervention commanded; collision avoided with min separation ≥ 2.0m.',
    type: 'cut_in'
  },
  'SCN-05': {
    id: 'SCN-05',
    name: 'Unsafe Following Distance (Tailgating)',
    category: 'Headway Governance',
    description: 'Ego approaches lead vehicle closer than legal headway threshold (TTC < 2.0s at 70 km/h). Tests distance alert & gentle throttle cutoff.',
    initialEgoSpeedKmh: 70,
    leadSpeedKmh: 70,
    initialGapM: 16,
    roadFrictionMu: 0.85,
    durationSeconds: 12,
    passCriteria: 'Headway warning advisory issued within 400ms; throttle cut off; safe following distance restored.',
    type: 'headway'
  },
  'SCN-06': {
    id: 'SCN-06',
    name: 'Lane Departure & Drift (LDW/LKA)',
    category: 'Lateral Safety',
    description: 'Vehicle drifts laterally across lane boundary without turn indicator at lateral velocity 0.4 m/s.',
    initialEgoSpeedKmh: 60,
    leadSpeedKmh: 60,
    initialGapM: 50,
    roadFrictionMu: 0.85,
    durationSeconds: 12,
    passCriteria: 'LDW alert triggered before tire crosses line (offset ≥ 0.35m); steering correction command issued.',
    type: 'ldw'
  },
  'SCN-07': {
    id: 'SCN-07',
    name: 'Low-Friction Wet Asphalt (Extended Braking)',
    category: 'Adverse Road Friction',
    description: 'AEB intervention on degraded road surface (mu = 0.40). Braking distance increased by ~110%. Tests heavy vehicle stopping envelope.',
    initialEgoSpeedKmh: 60,
    leadSpeedKmh: 0,
    initialGapM: 65,
    roadFrictionMu: 0.40,
    durationSeconds: 14,
    passCriteria: 'AEB commands max available deceleration (adjusted for mu=0.40); vehicle stops without jackknife or collision.',
    type: 'adverse_friction'
  },
  'SCN-08': {
    id: 'SCN-08',
    name: 'Stationary Pedestrian / Crosswalk (AEB-VRU)',
    category: 'Vulnerable Road Users',
    description: 'Pedestrian detected in vehicle trajectory at 35m while ego travels at 40 km/h.',
    initialEgoSpeedKmh: 40,
    leadSpeedKmh: 0,
    initialGapM: 35,
    roadFrictionMu: 0.85,
    durationSeconds: 10,
    passCriteria: 'Pedestrian detection registered; AEB commanded within 300ms; complete vehicle stop before crosswalk boundary.',
    type: 'vru'
  }
};

// ==============================================================================
// In-Memory Platform State
// ==============================================================================
const state = {
  vehicles: {},
  status: 'Waiting for simulation telemetry...',
  lastUpdate: null,
  activeProfile: 'autera_40t_tractor',

  // Telemetry provenance & CARLA connection verification
  carlaConnected: false,
  lastCarlaPacketTime: 0,
  activeSource: 'none', // 'carla_simulation' | 'synthetic_injection' | 'replay' | 'none'

  // Performance telemetry
  sampleTimes: [],
  sampleRateHz: 0.0,
  packetCount: 0,

  // Scenario Execution & Validation
  activeScenario: null,
  scenarioStartTime: null,
  scenarioHistory: [],
  scenarioTelemetry: [],

  // Chronological Event Log
  events: [],

  // Camera / BEV Latest Frame
  cameraFrame: null,
  cameraTimestamp: null,
};

// Periodic heartbeat: accurately verify whether real CARLA packets are arriving
setInterval(() => {
  const now = Date.now();
  const wasConnected = state.carlaConnected;
  // If no genuine CARLA packet received within last 3500ms, mark CARLA as disconnected
  state.carlaConnected = (now - state.lastCarlaPacketTime) < 3500;

  if (wasConnected && !state.carlaConnected) {
    recordEvent('TELEMETRY_SOURCE', 'ADVISORY', 'CARLA simulation stream timed out or disconnected', { source: 'carla_simulation' });
    if (state.activeSource === 'carla_simulation') {
      state.activeSource = 'none';
      state.status = 'CARLA stream disconnected (Waiting for client)';
      io.emit('statusUpdate', { status: state.status, carlaConnected: false, activeSource: state.activeSource });
    }
  }

  // Calculate sliding sample rate
  const oneSecondAgo = now - 1000;
  state.sampleTimes = state.sampleTimes.filter(t => t > oneSecondAgo);
  state.sampleRateHz = state.sampleTimes.length;
}, 500);

// Helper: Record chronological validation events
function recordEvent(type, severity, description, details = {}) {
  const event = {
    id: `EVT-${Date.now()}-${Math.floor(Math.random() * 1000)}`,
    timestamp: new Date().toISOString(),
    localTime: new Date().toLocaleTimeString(),
    type,
    severity, // 'INFO' | 'ADVISORY' | 'WARNING' | 'CRITICAL'
    description,
    scenarioId: state.activeScenario ? state.activeScenario.id : null,
    details
  };
  state.events.unshift(event);
  if (state.events.length > 500) state.events.pop();
  io.emit('newEvent', event);
  return event;
}

// Initial system event
recordEvent('SYSTEM', 'INFO', 'AUTERA ADAS Validation Server initialized on port ' + PORT);

// ---- MQTT client ----
let mqttLoggedError = false;
const mqttClient = mqtt.connect('mqtt://localhost:1883', { reconnectPeriod: 10000 });

mqttClient.on('connect', () => {
  console.log('[AUTERA Backend] MQTT broker connected on localhost:1883');
  mqttLoggedError = false;
  mqttClient.subscribe('carla/vehicles/+/telemetry');
  mqttClient.subscribe('carla/status');
  recordEvent('MQTT', 'INFO', 'Connected to local MQTT telemetry broker');
});

mqttClient.on('message', (topic, payload) => {
  try {
    const data = JSON.parse(payload.toString());

    if (topic.startsWith('carla/vehicles/')) {
      const id = topic.split('/')[2];
      handleIncomingTelemetry({ id, source: 'carla_simulation', ...data });
    }

    if (topic === 'carla/status') {
      state.status = data.status || 'unknown';
      io.emit('statusUpdate', { status: state.status, carlaConnected: state.carlaConnected, activeSource: state.activeSource });
    }
  } catch (err) {
    console.error('Bad MQTT payload:', err.message);
  }
});

mqttClient.on('error', (err) => {
  if (!mqttLoggedError) {
    console.log('[AUTERA Backend] MQTT broker not detected on port 1883. Telemetry available via HTTP POST /api/telemetry.');
    mqttLoggedError = true;
  }
});

// Central Telemetry Processor
function handleIncomingTelemetry(data) {
  const now = Date.now();
  state.sampleTimes.push(now);
  state.packetCount++;

  const id = String(data.id || 'ego_vehicle');
  const source = data.source || (data.id === 'ego_simulated' ? 'synthetic_injection' : 'carla_simulation');

  // Verify real CARLA provenance
  if (source === 'carla_simulation') {
    state.lastCarlaPacketTime = now;
    if (!state.carlaConnected) {
      state.carlaConnected = true;
      recordEvent('TELEMETRY_SOURCE', 'INFO', 'Verified CARLA simulation client connected via telemetry stream', { vehicleId: id });
    }
  }
  state.activeSource = source;

  // Enrich data with server-side time, profile info, and derived kinematics if absent
  const profile = VEHICLE_PROFILES[state.activeProfile] || VEHICLE_PROFILES.autera_40t_tractor;
  const speedKmh = typeof data.speed_kmh === 'number' ? data.speed_kmh : parseFloat(data.speed || 0);
  const distanceM = typeof data.distance_m === 'number' ? data.distance_m : parseFloat(data.distance || 999.0);
  const ttcS = typeof data.ttc_s === 'number' ? data.ttc_s : parseFloat(data.ttc || 99.0);

  // Compute relative closing speed if not provided
  let relativeSpeedMs = data.relative_speed_ms;
  if (typeof relativeSpeedMs !== 'number') {
    if (ttcS < 99 && distanceM < 999 && ttcS > 0) {
      relativeSpeedMs = Math.round((distanceM / ttcS) * 10) / 10;
    } else {
      relativeSpeedMs = 0.0;
    }
  }

  // Auto-detect safety events based on state transitions
  const prevVehicle = state.vehicles[id];
  const adasStatus = (data.adas_status || 'SAFE').toUpperCase();
  const emergencyBrake = Boolean(data.emergency_brake);
  const laneStatus = data.lane_status || 'OK';

  if (!prevVehicle) {
    recordEvent('VEHICLE', 'INFO', `Tracking vehicle #${id} (${source})`, { vehicleId: id, source });
  } else {
    // Detect FCW activation
    if (!prevVehicle.adas_status?.includes('WARN') && adasStatus.includes('WARN')) {
      recordEvent('FCW_ACTIVATION', 'WARNING', `Forward Collision Warning (FCW) triggered on Vehicle #${id}`, {
        vehicleId: id, speedKmh, distanceM, ttcS, relativeSpeedMs
      });
    }
    // Detect AEB Command
    if (!prevVehicle.emergency_brake && emergencyBrake) {
      recordEvent('AEB_COMMAND', 'CRITICAL', `Automatic Emergency Braking (AEB) commanded on Vehicle #${id} (Air brake onset)`, {
        vehicleId: id, speedKmh, distanceM, ttcS, relativeSpeedMs, airLagMs: profile.airBrakeLagSeconds * 1000
      });
    }
    // Detect Lane Departure
    if (prevVehicle.lane_status === 'OK' && laneStatus !== 'OK') {
      recordEvent('LANE_DEPARTURE', 'WARNING', `Lane departure detected on Vehicle #${id}: ${laneStatus}`, {
        vehicleId: id, laneStatus
      });
    }
  }

  const enrichedVehicle = {
    ...data,
    id,
    source,
    speed_kmh: speedKmh,
    distance_m: distanceM,
    ttc_s: ttcS,
    relative_speed_ms: relativeSpeedMs,
    adas_status: adasStatus,
    lane_status: laneStatus,
    emergency_brake: emergencyBrake,
    profileId: profile.id,
    profileName: profile.name,
    serverTimestamp: new Date().toISOString(),
  };

  state.vehicles[id] = enrichedVehicle;
  state.lastUpdate = new Date().toISOString();

  // If a scenario validation run is currently recording, save telemetry frame
  if (state.activeScenario) {
    state.scenarioTelemetry.push({
      timestamp: Date.now(),
      ...enrichedVehicle
    });
  }

  io.emit('vehicleUpdate', enrichedVehicle);
  return enrichedVehicle;
}

// ==============================================================================
// REST Endpoints
// ==============================================================================

// Health and Diagnostics
app.get('/api/health', (req, res) => {
  res.json({
    status: 'ok',
    platform: 'AUTERA ADAS Validation Platform',
    version: '2.4.0-commercial',
    timestamp: new Date().toISOString(),
    carlaConnected: state.carlaConnected,
    activeSource: state.activeSource,
    sampleRateHz: state.sampleRateHz,
    packetCount: state.packetCount,
    activeProfile: state.activeProfile
  });
});

// Telemetry & Vehicle Fleet
app.get('/api/vehicles', (req, res) => {
  res.json(Object.values(state.vehicles));
});

app.post('/api/telemetry', (req, res) => {
  try {
    const data = req.body;
    if (!data || !data.id) {
      return res.status(400).json({ error: 'Missing vehicle data or vehicle id' });
    }
    const vehicle = handleIncomingTelemetry(data);
    res.json({ success: true, vehicle });
  } catch (err) {
    res.status(500).json({ error: err.message });
  }
});

// Platform Status
app.get('/api/status', (req, res) => {
  res.json({
    status: state.status,
    lastUpdate: state.lastUpdate,
    carlaConnected: state.carlaConnected,
    activeSource: state.activeSource,
    sampleRateHz: state.sampleRateHz,
    activeProfile: VEHICLE_PROFILES[state.activeProfile],
    activeScenario: state.activeScenario
  });
});

app.post('/api/status', (req, res) => {
  try {
    const { status } = req.body;
    if (!status) return res.status(400).json({ error: 'Missing status string' });
    state.status = status;
    state.lastUpdate = new Date().toISOString();
    io.emit('statusUpdate', { status: state.status, carlaConnected: state.carlaConnected, activeSource: state.activeSource });
    res.json({ success: true, status: state.status });
  } catch (err) {
    res.status(500).json({ error: err.message });
  }
});

// Vehicle Profiles API
app.get('/api/profiles', (req, res) => {
  res.json({
    activeProfile: state.activeProfile,
    profiles: VEHICLE_PROFILES
  });
});

app.post('/api/profiles/select', (req, res) => {
  const { profileId } = req.body;
  if (!profileId || !VEHICLE_PROFILES[profileId]) {
    return res.status(400).json({ error: 'Invalid profileId. Available: ' + Object.keys(VEHICLE_PROFILES).join(', ') });
  }
  state.activeProfile = profileId;
  const profile = VEHICLE_PROFILES[profileId];
  recordEvent('VEHICLE_PROFILE', 'INFO', `Vehicle profile changed to ${profile.name}`, { profile });
  io.emit('profileChanged', profile);
  res.json({ success: true, activeProfile: profile });
});

// Scenarios API
app.get('/api/scenarios', (req, res) => {
  res.json({
    activeScenario: state.activeScenario,
    scenarios: VALIDATION_SCENARIOS,
    history: state.scenarioHistory.slice(-10)
  });
});

app.post('/api/scenarios/run', (req, res) => {
  const { scenarioId } = req.body;
  if (!scenarioId || !VALIDATION_SCENARIOS[scenarioId]) {
    return res.status(400).json({ error: 'Invalid scenarioId' });
  }
  const scn = VALIDATION_SCENARIOS[scenarioId];
  state.activeScenario = scn;
  state.scenarioStartTime = Date.now();
  state.scenarioTelemetry = [];

  recordEvent('SCENARIO_START', 'INFO', `Scenario Started: [${scn.id}] ${scn.name}`, { scenario: scn });
  io.emit('scenarioStarted', scn);
  res.json({ success: true, activeScenario: scn });
});

app.post('/api/scenarios/stop', (req, res) => {
  if (!state.activeScenario) {
    return res.status(400).json({ error: 'No active scenario is currently running' });
  }
  const scn = state.activeScenario;
  const durationSec = Math.round((Date.now() - state.scenarioStartTime) / 100) / 10;
  const frames = state.scenarioTelemetry;

  // Evaluate validation metrics
  let minTtc = 99.0;
  let minDistance = 999.0;
  let maxSpeed = 0.0;
  let fcwTriggered = false;
  let aebCommanded = false;
  let collisionOccurred = false;

  frames.forEach(f => {
    if (f.ttc_s > 0 && f.ttc_s < minTtc) minTtc = f.ttc_s;
    if (f.distance_m > 0 && f.distance_m < minDistance) minDistance = f.distance_m;
    if (f.speed_kmh > maxSpeed) maxSpeed = f.speed_kmh;
    if (f.adas_status?.includes('WARN')) fcwTriggered = true;
    if (f.emergency_brake) aebCommanded = true;
  });

  // Determine collision: distance dropped below commercial bumper clearance (1.0m) while moving
  if (minDistance < 1.0 && minTtc < 0.2) {
    collisionOccurred = true;
  }

  // Determine Pass / Fail against automotive standard
  let passVerdict = true;
  let verdictReason = 'All validation criteria met successfully.';

  if (scn.type === 'aeb') {
    if (collisionOccurred) {
      passVerdict = false;
      verdictReason = `Collision occurred (Min separation: ${minDistance.toFixed(1)}m < 1.5m standard requirement).`;
    } else if (!aebCommanded) {
      passVerdict = false;
      verdictReason = 'AEB failed to command emergency deceleration during critical proximity.';
    } else {
      verdictReason = `AEB deployed successfully. Min separation: ${minDistance.toFixed(1)}m, Min TTC: ${minTtc.toFixed(2)}s.`;
    }
  } else if (scn.type === 'fcw') {
    if (!fcwTriggered) {
      passVerdict = false;
      verdictReason = 'FCW failed to issue auditory/visual advisory prior to headway expiration.';
    } else {
      verdictReason = `FCW alert issued at TTC ${minTtc.toFixed(2)}s. Minimum separation preserved: ${minDistance.toFixed(1)}m.`;
    }
  }

  const result = {
    scenarioId: scn.id,
    scenarioName: scn.name,
    category: scn.category,
    profile: VEHICLE_PROFILES[state.activeProfile].name,
    timestamp: new Date().toISOString(),
    durationSeconds: durationSec,
    framesRecorded: frames.length,
    minTtcS: minTtc < 90 ? Math.round(minTtc * 100) / 100 : null,
    minDistanceM: minDistance < 900 ? Math.round(minDistance * 10) / 10 : null,
    maxSpeedKmh: Math.round(maxSpeed * 10) / 10,
    fcwTriggered,
    aebCommanded,
    collisionOccurred,
    passVerdict,
    verdictReason
  };

  state.scenarioHistory.push(result);
  state.activeScenario = null;
  state.scenarioStartTime = null;

  recordEvent('SCENARIO_RESULT', passVerdict ? 'INFO' : 'CRITICAL',
    `Scenario [${scn.id}] Finished - ${passVerdict ? 'PASSED' : 'FAILED'}: ${verdictReason}`, { result });

  io.emit('scenarioFinished', result);
  res.json({ success: true, result });
});

// Event Log API
app.get('/api/events', (req, res) => {
  const { severity, limit } = req.query;
  let filtered = state.events;
  if (severity) {
    filtered = filtered.filter(e => e.severity === severity.toUpperCase());
  }
  const maxLimit = parseInt(limit, 10) || 100;
  res.json(filtered.slice(0, maxLimit));
});

app.post('/api/events', (req, res) => {
  const { type, severity, description, details } = req.body;
  if (!type || !description) return res.status(400).json({ error: 'Missing type or description' });
  const event = recordEvent(type, severity || 'INFO', description, details || {});
  res.json({ success: true, event });
});

// Camera Frame API (Allows CARLA Python client or test scripts to push live camera frames)
app.post('/api/camera/frame', (req, res) => {
  const { imageBase64, frameNumber, timestamp } = req.body;
  if (!imageBase64) return res.status(400).json({ error: 'Missing imageBase64 payload' });
  state.cameraFrame = imageBase64;
  state.cameraTimestamp = timestamp || new Date().toISOString();
  io.emit('cameraFrame', { imageBase64, frameNumber, timestamp: state.cameraTimestamp });
  res.json({ success: true });
});

app.get('/api/camera/frame', (req, res) => {
  if (!state.cameraFrame) {
    return res.status(404).json({ error: 'No live camera frame currently available' });
  }
  res.json({ imageBase64: state.cameraFrame, timestamp: state.cameraTimestamp });
});

// ==============================================================================
// Socket.IO Real-Time Connection
// ==============================================================================
io.on('connection', (socket) => {
  console.log('[AUTERA Backend] ADAS Cockpit client connected (ID: ' + socket.id + ')');
  socket.emit('initialState', {
    vehicles: Object.values(state.vehicles),
    status: state.status,
    carlaConnected: state.carlaConnected,
    activeSource: state.activeSource,
    sampleRateHz: state.sampleRateHz,
    activeProfile: VEHICLE_PROFILES[state.activeProfile],
    profiles: VEHICLE_PROFILES,
    activeScenario: state.activeScenario,
    scenarios: VALIDATION_SCENARIOS,
    recentEvents: state.events.slice(0, 30),
    cameraFrame: state.cameraFrame
  });
});

server.listen(PORT, () => {
  console.log(`================================================================`);
  console.log(`  AUTERA ADAS Commercial Vehicle Validation Platform`);
  console.log(`  Backend listening at: http://localhost:${PORT}`);
  console.log(`  Serving Dashboard at: http://localhost:${PORT}/`);
  console.log(`================================================================`);
});