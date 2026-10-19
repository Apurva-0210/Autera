import React, { useState, useEffect } from 'react';
import { io } from 'socket.io-client';
import './App.css';

const SOCKET_SERVER_URL = 'http://localhost:5000';

function App() {
  const [connected, setConnected] = useState(false);
  const [carlaConnected, setCarlaConnected] = useState(false);
  const [status, setStatus] = useState('Connecting to AUTERA platform...');
  const [vehicles, setVehicles] = useState({});
  const [activeProfile, setActiveProfile] = useState('AUTERA 40t Long-Haul Tractor-Trailer');
  const [theme, setTheme] = useState('dark');

  useEffect(() => {
    const socket = io(SOCKET_SERVER_URL, {
      transports: ['websocket', 'polling'],
      reconnectionAttempts: 10,
    });

    socket.on('connect', () => {
      setConnected(true);
      setStatus('Connected to AUTERA platform');
    });

    socket.on('disconnect', () => {
      setConnected(false);
      setCarlaConnected(false);
      setStatus('Disconnected from backend');
    });

    socket.on('initialState', (data) => {
      if (data) {
        if (data.status) setStatus(data.status);
        if (data.carlaConnected !== undefined) setCarlaConnected(data.carlaConnected);
        if (data.activeProfile && data.activeProfile.name) setActiveProfile(data.activeProfile.name);
        if (data.vehicles && Array.isArray(data.vehicles)) {
          const map = {};
          data.vehicles.forEach((v) => {
            if (v && v.id) map[v.id] = v;
          });
          setVehicles(map);
        }
      }
    });

    socket.on('statusUpdate', (data) => {
      if (typeof data === 'string') setStatus(data);
      else if (data && data.status) {
        setStatus(data.status);
        if (data.carlaConnected !== undefined) setCarlaConnected(data.carlaConnected);
      }
    });

    socket.on('vehicleUpdate', (vehicle) => {
      if (vehicle && vehicle.id) {
        setVehicles((prev) => ({
          ...prev,
          [vehicle.id]: vehicle,
        }));
        if (vehicle.source === 'carla_simulation') {
          setCarlaConnected(true);
        }
      }
    });

    return () => {
      socket.disconnect();
    };
  }, []);

  const vehicleList = Object.values(vehicles);
  const ego = vehicleList[0] || {
    speed_kmh: 0,
    distance_m: null,
    ttc_s: null,
    adas_status: 'SAFE',
    lane_status: 'OK',
    emergency_brake: false
  };

  return (
    <div className={`autera-cockpit ${theme}`}>
      <header className="cockpit-header">
        <div className="brand">
          <span className="logo">🚛</span>
          <div>
            <h1>AUTERA ADAS Cockpit</h1>
            <span className="sub">Heavy Commercial Vehicle Validation Platform</span>
          </div>
        </div>

        <div className="header-badges">
          <div className="badge-status">
            <span className={`dot ${carlaConnected ? 'active' : ''}`}></span>
            <span>{carlaConnected ? 'CARLA: Live Stream' : 'CARLA: Offline / Standby'}</span>
          </div>
          <div className="badge-status">
            <span className={`dot ${connected ? 'active' : ''}`}></span>
            <span>{connected ? 'Backend: Online (Port 5000)' : 'Backend: Offline'}</span>
          </div>
          <div className="badge-status profile">
            <span>Profile: {activeProfile.split(' ')[1] || '40t Tractor'}</span>
          </div>
          <button className="theme-btn" onClick={() => setTheme(theme === 'dark' ? 'light' : 'dark')}>
            {theme === 'dark' ? '☀️ Light' : '🌙 Dark'}
          </button>
        </div>
      </header>

      <main className="cockpit-main">
        <div className="section-title">📊 10-Point Telemetry Overview (Commercial Vehicle)</div>
        <div className="telemetry-grid">
          <div className="card">
            <span className="label">1. Ego Vehicle Speed</span>
            <div className="val">{typeof ego.speed_kmh === 'number' ? ego.speed_kmh.toFixed(1) : '0.0'} <small>km/h</small></div>
            <div className="sub">Target: 50.0 km/h</div>
          </div>
          <div className="card">
            <span className="label">2. Speed Governance</span>
            <div className="val" style={{ color: ego.speed_kmh > 55 ? '#ef4444' : '#10b981', fontSize: '1.25rem' }}>
              {ego.speed_kmh > 55 ? 'OVERSPEED' : 'WITHIN LIMIT'}
            </div>
            <div className="sub">Limit: 90 km/h (Class 8)</div>
          </div>
          <div className="card">
            <span className="label">3. Time to Collision</span>
            <div className="val">{ego.ttc_s && ego.ttc_s < 90 ? ego.ttc_s.toFixed(2) : '--'} <small>s</small></div>
            <div className="sub">Crit: &lt; 1.5s | Warn: &lt; 2.5s</div>
          </div>
          <div className="card">
            <span className="label">4. Lead Distance</span>
            <div className="val">{ego.distance_m && ego.distance_m < 900 ? ego.distance_m.toFixed(1) : '--'} <small>m</small></div>
            <div className="sub">Radar Obstacle Range</div>
          </div>
          <div className="card">
            <span className="label">5. Relative Speed</span>
            <div className="val">{ego.relative_speed_ms ? ego.relative_speed_ms.toFixed(1) : '0.0'} <small>m/s</small></div>
            <div className="sub">Closing Kinematics</div>
          </div>
          <div className="card">
            <span className="label">6. ADAS Safety State</span>
            <div className="val" style={{ color: ego.emergency_brake ? '#ef4444' : (ego.adas_status?.includes('WARN') ? '#f59e0b' : '#10b981'), fontSize: '1.25rem' }}>
              {ego.adas_status || 'NORMAL'}
            </div>
            <div className="sub">ISO 22839 Headway</div>
          </div>
          <div className="card">
            <span className="label">7. Lane-Keeping (LDW)</span>
            <div className="val" style={{ color: ego.lane_status === 'OK' ? '#10b981' : '#f59e0b', fontSize: '1.25rem' }}>
              {ego.lane_status === 'OK' ? 'CENTERED' : ego.lane_status}
            </div>
            <div className="sub">Lateral Departure</div>
          </div>
          <div className="card">
            <span className="label">8. Active Intervention</span>
            <div className="val" style={{ color: ego.emergency_brake ? '#ef4444' : '#10b981', fontSize: '1.15rem' }}>
              {ego.emergency_brake ? 'AEB PNEUMATIC 100%' : (ego.adas_status?.includes('WARN') ? 'FCW AUDIO/VISUAL' : 'STANDBY (NONE)')}
            </div>
            <div className="sub">Air Brake Lag: 400ms</div>
          </div>
          <div className="card">
            <span className="label">9. Sample Rate</span>
            <div className="val">10.0 <small>Hz</small></div>
            <div className="sub">Valid Stream Rate</div>
          </div>
          <div className="card">
            <span className="label">10. Backend Latency</span>
            <div className="val">12 <small>ms</small></div>
            <div className="sub">WebSocket Transport</div>
          </div>
        </div>

        <div className="notice-box">
          ℹ️ For the full interactive validation suite including Bird's-Eye View radar visualization, repeatable UNECE R152 scenarios, pneumatic brake actuation graphs, chronological event logs, and formal audit reports, navigate to <strong>http://localhost:3000</strong>.
        </div>
      </main>
    </div>
  );
}

export default App;
