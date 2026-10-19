# AUTERA
### An India-Calibrated ADAS Framework for Heavy Commercial Vehicles in Heterogeneous Urban Traffic

> A simulation-driven Advanced Driver Assistance System (ADAS) framework focused on improving the validation and evaluation of safety-critical driving functions for heavy commercial vehicles operating in complex urban environments.

---

## 📌 Overview

**AUTERA** is a research-oriented ADAS framework designed around the challenges of deploying and validating driver-assistance systems for **heavy commercial vehicles (HCVs)** in heterogeneous urban traffic.

Unlike controlled highway environments, real-world urban traffic involves a highly dynamic mixture of:

- Cars
- Buses
- Heavy commercial vehicles
- Two-wheelers
- Pedestrians
- Lane-changing traffic
- Variable road conditions
- Dense and unpredictable interactions

These conditions introduce significant challenges for conventional ADAS validation approaches.

AUTERA explores a **simulation-driven validation framework** intended to evaluate ADAS behavior under scenarios representative of complex Indian urban traffic.

---

## 🎯 Research Objective

The primary objective of AUTERA is to develop an **India-calibrated simulation and validation framework for ADAS in heavy commercial vehicles**.

The framework focuses on evaluating safety-critical driving situations through controlled simulation while accounting for the heterogeneous traffic conditions encountered in Indian urban environments.

---

## 🚛 Why Heavy Commercial Vehicles?

ADAS validation for heavy commercial vehicles presents additional challenges because of their:

- Larger vehicle dimensions
- Longer stopping distances
- Different acceleration and braking characteristics
- Larger blind spots
- Higher mass and inertia
- Different interaction dynamics with smaller road users

Consequently, safety thresholds and driver-assistance behavior designed primarily around passenger vehicles may not directly represent HCV operating conditions.

AUTERA investigates these challenges through simulation-based evaluation.

---

## 🌆 Target Environment

AUTERA focuses on **heterogeneous urban traffic**, where multiple road-user types interact simultaneously.

### Representative Traffic Environment

```text
                    Urban Traffic Environment
                               │
           ┌───────────────────┴───────────────────┐
           │                   │                   │
        Passenger          Two-Wheelers       Pedestrians
         Vehicles
           │                   │                   │
           └───────────────────┬───────────────────┘
                               │
                               ▼
                      Heavy Commercial
                          Vehicle
                               │
                               ▼
                        ADAS Functions
                               │
                               ▼
                      Safety Evaluation
```

---

## 🧠 System Concept

AUTERA follows a simulation-driven validation workflow:

```text
Scenario Definition
        │
        ▼
Traffic & Environment Simulation
        │
        ▼
Multi-Sensor Simulation
        │
        ▼
Sensor Data Processing
        │
        ▼
Object / Environment Understanding
        │
        ▼
ADAS Decision Logic
        │
        ▼
Safety Metric Evaluation
        │
        ▼
Scenario-Level Validation
```

The framework allows controlled experimentation across different traffic configurations and safety-critical scenarios.

---

## 🛰️ Multi-Sensor Simulation

A central component of the framework is the use of multi-sensor information to represent the perception inputs available to an ADAS system.

The simulation environment studies sensor observations under varying traffic and environmental conditions:

```text
       Camera
         │
         │
       Radar
         │
         │
   Other Sensor Inputs
         │
         ▼
   ┌───────────────┐
   │ Sensor Fusion │
   └───────────────┘
         │
         ▼
  Environment / Object
     Representation
         │
         ▼
    ADAS Decision
```

---

## ⏱️ Adaptive Time-to-Collision Reasoning

AUTERA incorporates adaptive Time-to-Collision (TTC) reasoning as part of its safety-evaluation approach.

TTC-based analysis provides a quantitative representation of the temporal proximity between the ego vehicle and another road user, accounting for pneumatic brake lag and payload dynamics:

```text
Relative Position
       +
Relative Velocity
       +
Vehicle Dynamics
       │
       ▼
  TTC Estimation
       │
       ▼
Adaptive Safety Assessment
       │
       ▼
ADAS Response / Evaluation
```

---

## 🖥️ Live ADAS Cockpit & Validation Platform

The framework is paired with an interactive automotive engineering cockpit running at `localhost:3000` with a backend telemetry server at `localhost:5000`.

- **HTML5 Canvas Bird's-Eye View (BEV)**: Real-time radar range rings ($10\text{m}$ to $50\text{m}$), field of view cone, closing-velocity vectors, and dynamic color-coded hazard levels.
- **Calibrated Vehicle Profiles**: Commercial 40t Semi-Trailer, 26t Rigid Truck, 16t Medium Duty, and reference SUV baseline.
- **UNECE R152 Validation Scenarios**: Automated test executions (CCRs stationary, CCRm moving, CCRb $4\text{ m/s}^2$ deceleration, cut-in, low friction $\mu = 0.35$).
- **Audit Reporting**: One-click generation and download of UNECE R152 safety compliance reports in structured JSON and certified text formats.

---

## 📁 Repository Structure

```plaintext
Autera/
├── README.md                    # Research overview, architecture, and documentation
├── requirements.txt             # Python environment dependencies
├── adas_final1.py               # Core CARLA ADAS simulation controller & dynamics
├── adas_final1_paper_metrics.csv# Research validation metrics & benchmark data
├── adas_final1_ttc_data.csv     # Time-To-Collision telemetry logs
├── dashboard/
│   ├── backend/
│   │   ├── server.js            # Node.js ADAS telemetry server (port 5000)
│   │   └── package.json         # Backend Express & Socket.IO dependencies
│   └── frontend/
│       └── public/
│           └── index.html       # AUTERA ADAS Cockpit & Canvas BEV (port 3000)
├── Lonavala Trial.xodr          # Custom OpenDRIVE Indian highway road network
├── Lonavala Trial.fbx           # 3D roadway and terrain mesh
├── kothrud_projected.osm        # Projected OpenStreetMap network (Pune, India)
├── kothrud_sample.osm           # Sample road network
├── PythonAPI/                   # CARLA Python API client libraries
├── Co-Simulation/               # SUMO & PTV-Vissim co-simulation bridges
├── Skills/                      # Antigravity agentic skill extensions
└── Docs/                        # Complete CARLA architectural documentation
```

---

## 🛠️ Quickstart & Setup Guide

### 1. Prerequisites
- **Python 3.10+**
- **Node.js 18+**
- **CARLA 0.9.15 (Windows)**: Pre-compiled Unreal Engine assets ($16.6\text{ GB}$) exceed GitHub's $100\text{ MB}$ limit. Download CARLA 0.9.15 from the [CARLA Releases page](https://github.com/carla-simulator/carla/releases/tag/0.9.15) and place the `CarlaUE4` folder in the root directory.

### 2. Environment Setup
```bash
# Python dependencies
python -m venv .venv
.\.venv\Scripts\activate
pip install -r requirements.txt

# Backend dependencies
cd dashboard/backend
npm install
```

### 3. Launching the Platform
```bash
# Terminal 1: Launch CARLA Simulator (DirectX 11 mode)
CarlaUE4.exe -dx11 -windowed -ResX=1280 -ResY=720 -quality-level=Low

# Terminal 2: Start Telemetry Backend (Port 5000)
cd dashboard/backend
node server.js

# Terminal 3: Launch Validation Cockpit (Port 3000)
python -m http.server 3000 --directory dashboard/frontend/public

# Terminal 4: Run the ADAS Controller
python adas_final1.py
```
Open **`http://localhost:3000`** in any modern web browser to monitor live telemetry, inspect radar feeds, run UNECE R152 test scenarios, and export validation reports.

---

## 🔬 Research Workflow

AUTERA is structured around a scenario-based validation methodology:

1. **Scenario Generation**: Define traffic, road geometry, vehicle configuration, and interaction conditions.
2. **Simulation**: Execute the scenario within the simulation environment.
3. **Sensor Observation**: Generate simulated observations from the configured sensor suite.
4. **Perception & Fusion**: Process sensor information to represent surrounding road users and the driving environment.
5. **Safety Reasoning**: Evaluate the relative motion and safety state of the ego vehicle.
6. **ADAS Evaluation**: Assess the behavior of the ADAS system under the given scenario.
7. **Performance Analysis**: Compare system behavior across different scenario configurations.

---

## 🧪 Validation Philosophy

AUTERA is designed around the principle that ADAS systems should be evaluated across diverse and representative traffic conditions, rather than relying exclusively on a small number of deterministic scenarios.

The framework emphasizes:
- Scenario diversity
- Heterogeneous traffic
- Heavy-vehicle dynamics
- Multi-sensor perception
- Safety-critical interactions
- Quantitative safety metrics
- Repeatable simulation experiments

---

## 📄 Publication Status

**Research Work — Under Review**

AUTERA is associated with ongoing research work that is currently under review.

To respect the publication process, this repository provides the simulation architecture, telemetry pipeline, validation cockpit, and experimental scenario framework, while publication-sensitive experimental comparisons remain under review.

Additional implementation details, experimental results, and figures will be updated following completion of the formal peer review process.

---

## 👨‍💻 Author

**Apurva Kumar**  
B.Tech — Electronics & Communication Engineering (AI & ML Specialization)  
MIT World Peace University, Pune, India  

---

## 📌 Project

**AUTERA**: An India-Calibrated ADAS Framework for Heavy Commercial Vehicles in Heterogeneous Urban Traffic  
*Research Status: Under Review · ADAS · Automotive Systems · Heavy Commercial Vehicles · Simulation · CARLA · RoadRunner · Sensor Fusion*
