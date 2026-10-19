#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Full ADAS Demo System for CARLA 0.9.15+
========================================
Production-ready single-file demo: Forward Collision Warning, Adaptive Cruise Control,
Lane Invasion & Wrong-Way Detection, manual driving with pygame HUD.

Requirements: CARLA server running at localhost:2000, pygame, carla PythonAPI in path.
Maps: Custom hell.xodr map with hell.fbx reference. Uses only built-in vehicles and custom map.
"""

from __future__ import print_function

import math
import sys
import time
import weakref
import os
import csv
import glob

# Add CARLA PythonAPI to path
repo_root = os.path.dirname(os.path.abspath(__file__))
carla_path = os.path.join(repo_root, "carla")
for p in (repo_root, carla_path):
    if os.path.exists(p) and p not in sys.path:
        sys.path.append(p)

python_api = os.path.join(repo_root, "PythonAPI")
# Append PythonAPI/carla/agents for navigation utilities without shadowing binary carla
agents_path = os.path.join(python_api, "carla", "agents")
if os.path.exists(agents_path) and agents_path not in sys.path:
    sys.path.append(agents_path)
if os.path.exists(python_api) and python_api not in sys.path:
    sys.path.append(python_api)

# Optional: add CARLA PythonAPI egg if running from CARLA root
try:
    for egg in glob.glob(os.path.join(repo_root, "PythonAPI", "carla", "dist", "carla-*.egg")):
        if egg not in sys.path:
            sys.path.append(egg)
except Exception:
    pass

# Check Python version compatibility
python_version = f"{sys.version_info.major}.{sys.version_info.minor}"
print(f"Using Python {python_version}")

try:
    import carla
    print("CARLA imported successfully")
except ImportError:
    print("CARLA not found, attempting installation...")
    
    # Try to install carla from PyPI with verbose output
    try:
        import subprocess
        print("Installing CARLA from PyPI...")
        result = subprocess.run([sys.executable, "-m", "pip", "install", "carla"], 
                              capture_output=True, text=True)
        if result.returncode == 0:
            import carla
            print("CARLA installed successfully from PyPI")
        else:
            print(f"PyPI installation failed: {result.stderr}")
            raise RuntimeError("PyPI installation failed")
    except Exception as e:
        print(f"CARLA installation failed: {e}")
        print("\nPlease install CARLA manually:")
        print("1. Open command prompt as Administrator")
        print("2. Run: pip install carla")
        print("3. Or try: pip install --upgrade pip && pip install carla")
        print("\nIf that doesn't work, you may need to:")
        print("- Install Python 3.12 and use: py -3.12 adas_demo_full3.py")
        print("- Or build CARLA from source for Python 3.14")
        raise RuntimeError(f"Cannot import carla. Manual installation required. Error: {e}")

try:
    import pygame
    from pygame.locals import K_ESCAPE, K_SPACE, K_a, K_d, K_s, K_w, K_q, K_l, K_UP, K_DOWN, K_c, K_r, K_e
except ImportError:
    raise RuntimeError("Cannot import pygame. Install with: pip install pygame")

try:
    import numpy as np
except ImportError:
    raise RuntimeError("Cannot import numpy. Install with: pip install numpy")


# ==============================================================================
# Constants
# ==============================================================================

CARLA_HOST = "localhost"
CARLA_PORT = 2000
TM_PORT = 8000
FIXED_DELTA_SECONDS = 0.03
FCW_DETECTION_RANGE_M = 10.0
FCW_TTC_WARNING_S = 4.0
FCW_TTC_EMERGENCY_S = 2.0
ACC_SAFE_DISTANCE_MIN_M = 7.0
ACC_SAFE_DISTANCE_MAX_M = 10.0
# Distance threshold: when closer than this, speed is reduced and limited until safe distance
ACC_DISTANCE_THRESHOLD_M = 7.0   # Below this: limit speed
ACC_SAFE_DISTANCE_CRUISE_M = 10.0  # Above this: allow normal cruising
MAX_SPEED_WHEN_TOO_CLOSE_KMH = 25.0  # Max speed (km/h) when within threshold distance
# Direct front only: ignore vehicles in adjacent lanes (e.g. bike on left)
FRONT_DOT_THRESHOLD = 0.98       # Cosine: ~11 deg cone - directly ahead only
FRONT_RADAR_AZIMUTH_DEG = 10     # Radar: ±10 deg of center (direct front only)
FRONT_LATERAL_M = 2.0            # Max lateral offset (m) - object must be in same lane corridor
PEDESTRIAN_FRONT_M = 25.0        # Detect pedestrians within this distance ahead
PEDESTRIAN_LATERAL_M = 2.5       # Pedestrian in path if lateral offset < this
TLIGHT_STOP_DISTANCE_M = 15.0    # Stop when red light within this distance
WRONG_WAY_DOT_THRESHOLD = -0.3  # Dot product with lane forward: negative = wrong way
EMERGENCY_BRAKE_RAMP_UP = 2.0  # Brake increase per second when emergency active
THROTTLE_REDUCTION_WARNING = 0.5  # Factor to reduce throttle when TTC warning
# Lane-keeping: steer back to track when invasion/wrong-way
LANE_KEEPING_STEER_GAIN = 2.5   # How strongly to steer toward lane center
LANE_KEEPING_MAX_STEER = 0.6     # Cap corrective steer
# Camera mouse control
CAMERA_MOUSE_SENSITIVITY = 0.15  # Degrees per pixel
CAMERA_DISTANCE = 8.0            # Distance behind vehicle
CAMERA_HEIGHT = 4.0              # Height above vehicle

# Heavy ego vehicle blueprint preferences (no Cybertruck)
EGO_VEHICLE_PREFERENCES = [
    "vehicle.carlamotors.firetruck",
    "vehicle.mercedes.sprinter",
    "vehicle.volkswagen.t2",
    "vehicle.carlamotors.carlacola",
    "vehicle.ford.ambulance",
    "vehicle.audi.etron",
    "vehicle.chevrolet.impala",
    "vehicle.dodge.charger_police",
]
NUM_TRAFFIC_VEHICLES = 120
NUM_PEDESTRIANS = 30
DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 450
HUD_FONT_SIZE = 22
CUSTOM_HDMAP_FBX_PATH = r"C:\Users\HP\Downloads\CARLA_Latest\HDMaps\hell.fbx"
CUSTOM_HDMAP_XODR_PATH = r"C:\Users\HP\Downloads\CARLA_Latest\HDMaps\hell.xodr"


# ==============================================================================
# Paper-Aligned Sensor Suite
# ==============================================================================

from dataclasses import dataclass
from enum import Enum
from typing import Dict, Tuple

# Paper sensor ranges (meters)
LIDAR_RANGE_M = 100.0
RADAR_RANGE_M = 100.0
ULTRASONIC_RANGE_M = 3.0
IR_LATERAL_RANGE_M = 2.5

# Paper TTC state thresholds (seconds)
TTC_SAFE_GT = 3.0
TTC_WARNING_MAX = 3.0
TTC_WARNING_MIN = 2.0
TTC_EMERGENCY_MAX = 2.0
TTC_EMERGENCY_MIN_EXCLUSIVE = 1.5
TTC_CRITICAL_MAX = 1.5

class AdasState(str, Enum):
    SAFE = "SAFE"
    WARNING = "WARNING"
    EMERGENCY = "EMERGENCY"
    CRITICAL = "CRITICAL"

@dataclass
class SensorReadings:
    lidar_distance_m: float = float("inf")
    radar_relative_velocity_ms: float = 0.0  # positive means closing
    ultrasonic_distance_m: float = float("inf")
    ir_left_distance_m: float = float("inf")
    ir_right_distance_m: float = float("inf")
    timestamp: float = 0.0

@dataclass
class FusedState:
    distance_m: float
    relative_velocity_ms: float
    ttc_s: float
    ultrasonic_critical: bool
    lateral_warning: bool

class FusionEngine:
    def compute_fused_state(
        self, 
        lidar: SensorReadings, 
        radar: SensorReadings, 
        ultrasonic: SensorReadings, 
        ir: SensorReadings, 
        ego_speed_ms: float
    ) -> FusedState:
        # Distance fusion: prioritize ultrasonic < 3m, else LiDAR
        if ultrasonic.ultrasonic_distance_m < 3.0:
            distance = ultrasonic.ultrasonic_distance_m
        else:
            distance = lidar.lidar_distance_m
        
        # Relative velocity from radar
        rel_vel = radar.radar_relative_velocity_ms
        
        # TTC calculation
        if distance <= 0.0 or rel_vel <= 0.0:
            ttc = float("inf")
        else:
            ttc = distance / rel_vel
        
        # Flags
        ultrasonic_critical = ultrasonic.ultrasonic_distance_m < 3.0
        lateral_warning = (ir.ir_left_distance_m < IR_LATERAL_RANGE_M or 
                          ir.ir_right_distance_m < IR_LATERAL_RANGE_M)
        
        return FusedState(
            distance_m=distance,
            relative_velocity_ms=rel_vel,
            ttc_s=ttc,
            ultrasonic_critical=ultrasonic_critical,
            lateral_warning=lateral_warning
        )

class TTCComputer:
    @staticmethod
    def compute_ttc(distance_m: float, rel_vel_closing_ms: float) -> float:
        if distance_m == float("inf"):
            return float("inf")
        if distance_m <= 0.0:
            return 0.0
        if rel_vel_closing_ms <= 1e-3:
            return float("inf")
        return distance_m / rel_vel_closing_ms

class DecisionEngine:
    def __init__(self, mode: str = "proposed"):
        self.mode = mode

    def decide(self, fused: FusedState) -> Tuple[AdasState, float]:
        """Returns (state, brake_command 0..1) using paper thresholds."""
        # CRITICAL conditions
        if fused.ultrasonic_critical or fused.distance_m < 3.0 or fused.ttc_s <= TTC_CRITICAL_MAX:
            return AdasState.CRITICAL, 1.0

        # EMERGENCY: proactive braking (strict interval)
        if TTC_EMERGENCY_MIN_EXCLUSIVE < fused.ttc_s <= TTC_EMERGENCY_MAX:
            return AdasState.EMERGENCY, 0.45

        # WARNING
        if TTC_WARNING_MIN < fused.ttc_s <= TTC_WARNING_MAX:
            return AdasState.WARNING, 0.0

        return AdasState.SAFE, 0.0

class MetricsLogger:
    def __init__(self, csv_path: str):
        self._f = open(csv_path, "w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._f, fieldnames=[
            "timestamp", "mode", "distance_m", "relative_velocity_ms", "ttc_s",
            "ultrasonic_critical", "lateral_warning", "state", "brake_cmd"
        ])
        self._writer.writeheader()
        self.tp = self.fp = self.tn = self.fn = 0

    def log(self, ts: float, mode: str, fused: FusedState, state: AdasState, brake_cmd: float, sensor_ts: float):
        row = {
            "timestamp": ts,
            "mode": mode,
            "distance_m": fused.distance_m,
            "relative_velocity_ms": fused.relative_velocity_ms,
            "ttc_s": fused.ttc_s,
            "ultrasonic_critical": fused.ultrasonic_critical,
            "lateral_warning": fused.lateral_warning,
            "state": state.value,
            "brake_cmd": brake_cmd
        }
        self._writer.writerow(row)
        
        # Update confusion matrix
        ground_truth = (fused.ttc_s <= 3.0 and fused.relative_velocity_ms > 0.1) or fused.ultrasonic_critical or (fused.distance_m < 7.0)
        predicted = (state != AdasState.SAFE) or (brake_cmd > 0.0)
        if predicted and ground_truth:
            self.tp += 1
        elif predicted and not ground_truth:
            self.fp += 1
        elif not predicted and not ground_truth:
            self.tn += 1
        else:
            self.fn += 1

    def close(self):
        try:
            self._f.close()
        except Exception:
            pass

    def summary(self) -> Dict[str, float]:
        total = self.tp + self.fp + self.tn + self.fn
        if total == 0:
            return {"accuracy": 0.0, "fpr": 0.0, "mdr": 0.0, "tp": 0.0, "fp": 0.0, "tn": 0.0, "fn": 0.0}
        return {
            "accuracy": (self.tp + self.tn) / total,
            "fpr": self.fp / (self.fp + self.tn) if (self.fp + self.tn) > 0 else 0.0,
            "mdr": self.fn / (self.fn + self.tp) if (self.fn + self.tp) > 0 else 0.0,
            "tp": float(self.tp), "fp": float(self.fp), "tn": float(self.tn), "fn": float(self.fn)
        }

# ==============================================================================
# EgoVehicleManager
# ==============================================================================

class EgoVehicleManager(object):
    """Spawns and manages the single ego (heavy) vehicle."""

    def __init__(self, world, blueprint_library, map_instance):
        self.world = world
        self.blueprint_library = blueprint_library
        self.map_instance = map_instance
        self.actor = None
        self._spawn_point = None

    def _get_ego_blueprint(self):
        """Return first available heavy vehicle blueprint (no Cybertruck)."""
        for pref in EGO_VEHICLE_PREFERENCES:
            bps = self.blueprint_library.filter(pref)
            if bps:
                return bps[0]
        # Fallback: any vehicle that is not tesla.cybertruck
        all_vehicles = self.blueprint_library.filter("vehicle.*")
        for bp in all_vehicles:
            if "cybertruck" in bp.id.lower():
                continue
            # Prefer trucks/buses by name
            if any(x in bp.id for x in ["firetruck", "sprinter", "t2", "ambulance", "carlacola", "bus", "truck"]):
                return bp
        return all_vehicles[0] if all_vehicles else None

    def spawn(self):
        """Spawn ego at a random spawn point. Returns True on success."""
        spawn_points = self.map_instance.get_spawn_points()
        bp = self._get_ego_blueprint()
        if not bp:
            return False
        import random

        def _try_spawn(transform):
            self.actor = self.world.try_spawn_actor(bp, transform)
            return self.actor is not None

        # Try several map spawn transforms (PlayerStart); collisions often cause try_spawn to fail.
        if spawn_points:
            print("Trying %d map spawn points for ego..." % len(spawn_points), flush=True)
            order = list(spawn_points)
            random.shuffle(order)
            for sp in order[: min(40, len(order))]:
                self._spawn_point = sp
                if _try_spawn(sp):
                    return True
            print("All sampled spawn points failed; trying waypoint-based spawns...", flush=True)

        # OpenDRIVE-only / custom maps often have no PlayerStart actors, or spawns are blocked.
        print("Using driving waypoints for ego spawn...", flush=True)
        try:
            wps = self.map_instance.generate_waypoints(5.0)
            random.shuffle(wps)
            for wp in wps[: min(80, len(wps))]:
                loc = wp.transform.location
                loc.z += 0.5
                t = carla.Transform(loc, wp.transform.rotation)
                self._spawn_point = t
                if _try_spawn(t):
                    return True
        except Exception as e:
            print("Waypoint sweep failed: %s" % e, flush=True)

        try:
            wp = self.map_instance.get_waypoint(
                carla.Location(0.0, 0.0, 0.0),
                project_to_road=True,
                lane_type=carla.LaneType.Driving,
            )
            if wp is not None:
                loc = wp.transform.location
                loc.z += 1.0
                self._spawn_point = carla.Transform(loc, wp.transform.rotation)
                if _try_spawn(self._spawn_point):
                    return True
        except Exception:
            pass

        self._spawn_point = carla.Transform(carla.Location(0.0, 0.0, 3.0), carla.Rotation(yaw=0.0))
        return _try_spawn(self._spawn_point)

    def get_actor(self):
        return self.actor

    def destroy(self):
        if self.actor and self.actor.is_alive:
            self.actor.destroy()
            self.actor = None


# ==============================================================================
# TrafficManager (traffic + pedestrians)
# ==============================================================================

class TrafficManager(object):
    """Spawns and manages traffic vehicles and pedestrians with AI."""

    def __init__(self, client, world, blueprint_library, map_instance, ego_actor):
        self.client = client
        self.world = world
        self.blueprint_library = blueprint_library
        self.map_instance = map_instance
        self.ego_actor = ego_actor
        self.traffic_manager = client.get_trafficmanager(TM_PORT)
        self.vehicle_ids = []
        self.walker_ids = []
        self.walker_actors = []  # [{"id": id, "con": controller_id}, ...]
        self._all_walker_actor_ids = []

    def spawn_traffic_vehicles(self, count=NUM_TRAFFIC_VEHICLES):
        """Spawn count vehicles with autopilot, excluding ego spawn area."""
        from carla.command import SpawnActor, SetAutopilot, FutureActor

        spawn_points = list(self.map_instance.get_spawn_points())
        ego_loc = self.ego_actor.get_location() if self.ego_actor else None
        usable = [sp for sp in spawn_points if not ego_loc or sp.location.distance(ego_loc) > 15.0]
        if len(usable) < count:
            usable = spawn_points

        blueprints = list(self.blueprint_library.filter("vehicle.*"))
        # Filter out bicycles, motorcycles, and cybertruck
        blueprints = [b for b in blueprints if "cybertruck" not in b.id.lower() 
                    and "bike" not in b.id.lower() 
                    and "motorcycle" not in b.id.lower()
                    and "bicycle" not in b.id.lower()]
        if not blueprints:
            blueprints = list(self.blueprint_library.filter("vehicle.*"))

        batch = []
        np_random = __import__("random")
        for i in range(min(count, len(usable))):
            transform = usable[i % len(usable)]
            bp = np_random.choice(blueprints)
            if bp.has_attribute("color"):
                bp.set_attribute("color", np_random.choice(bp.get_attribute("color").recommended_values))
            batch.append(
                SpawnActor(bp, transform).then(
                    SetAutopilot(FutureActor, True, self.traffic_manager.get_port())
                )
            )

        for resp in self.client.apply_batch_sync(batch, do_tick=True):
            if not resp.error:
                self.vehicle_ids.append(resp.actor_id)

    def spawn_pedestrians(self, count=NUM_PEDESTRIANS):
        """Spawn pedestrians with AI walker controllers."""
        from carla.command import SpawnActor

        walker_bps = list(self.blueprint_library.filter("walker.pedestrian.*"))
        if not walker_bps:
            return
        np_random = __import__("random")
        spawn_points = []
        for _ in range(count):
            loc = self.world.get_random_location_from_navigation()
            if loc is not None:
                spawn_points.append(carla.Transform(loc))

        batch = []
        walker_speeds = []
        for sp in spawn_points:
            wb = np_random.choice(walker_bps)
            if wb.has_attribute("is_invincible"):
                wb.set_attribute("is_invincible", "false")
            if wb.has_attribute("speed"):
                walker_speeds.append(wb.get_attribute("speed").recommended_values[1])
            else:
                walker_speeds.append(1.4)
            batch.append(SpawnActor(wb, sp))

        results = self.client.apply_batch_sync(batch, do_tick=True)
        walkers = []
        speeds_ok = []
        for i, r in enumerate(results):
            if r.error:
                continue
            walkers.append({"id": r.actor_id})
            speeds_ok.append(walker_speeds[i])

        ctrl_bp = self.world.get_blueprint_library().find("controller.ai.walker")
        batch = []
        for w in walkers:
            batch.append(SpawnActor(ctrl_bp, carla.Transform(), w["id"]))
        ctrl_results = self.client.apply_batch_sync(batch, do_tick=True)
        for i, r in enumerate(ctrl_results):
            if not r.error and i < len(walkers):
                walkers[i]["con"] = r.actor_id

        all_id = []
        for w in walkers:
            if "con" in w:
                all_id.append(w["con"])
                all_id.append(w["id"])
        self._all_walker_actor_ids = list(all_id)
        self.walker_actors = walkers

        all_actors = self.world.get_actors(all_id)
        self.world.tick()
        for i in range(0, len(all_id), 2):
            if i + 1 < len(all_id):
                con = all_actors[i]
                idx = i // 2
                if idx < len(speeds_ok):
                    con.start()
                    con.go_to_location(self.world.get_random_location_from_navigation())
                    con.set_max_speed(float(speeds_ok[idx]))

    def configure_traffic_manager(self):
        """Set TM for synchronous mode and following distance."""
        self.traffic_manager.set_synchronous_mode(True)
        self.traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        self.traffic_manager.global_percentage_speed_difference(20.0)
        # Set random seed for consistent behavior
        self.traffic_manager.set_random_device_seed(0)
        self.traffic_manager.set_hybrid_physics_mode(True)  # Better physics
        # Note: Left-hand driving may need to be set in CARLA server settings

    def destroy_all(self):
        """Destroy all spawned traffic and walkers."""
        from carla.command import DestroyActor

        if self.vehicle_ids:
            self.client.apply_batch_sync([DestroyActor(aid) for aid in self.vehicle_ids], do_tick=True)
            self.vehicle_ids.clear()
        if self._all_walker_actor_ids:
            all_actors = self.world.get_actors(self._all_walker_actor_ids)
            for i in range(0, len(self._all_walker_actor_ids), 2):
                if i < len(all_actors):
                    all_actors[i].stop()
            self.client.apply_batch_sync(
                [DestroyActor(aid) for aid in self._all_walker_actor_ids], do_tick=True
            )
            self._all_walker_actor_ids.clear()
            self.walker_actors.clear()


# ==============================================================================
# SensorManager (RGB camera, lane invasion)
# ==============================================================================

class SensorManager(object):
    """Attaches RGB camera, ultrasonic/radar sensors, and lane invasion sensor to ego."""

    def __init__(self, world, ego_actor):
        self.world = world
        self.ego_actor = ego_actor
        self.camera = None
        self.lane_invasion_sensor = None
        self.radar_sensors = []  # Multiple forward-facing radar sensors (like ultrasonic)
        self._image_queue = None
        self._latest_image = None
        self._lane_invasion_occurred = False
        self._crossed_solid = False
        self._invasion_callback_data = {"solid": False, "occurred": False}
        self._radar_detections = []  # Latest radar detections: [{"distance": d, "velocity": v, "azimuth": a}, ...]
        self._sensor_detections_dict = {}  # Per-sensor detections dict: {sensor_idx: [detections]}

    def setup_camera(self, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, mouse_controlled=True):
        """
        Spawn RGB camera. If mouse_controlled, camera is free-floating and updated each frame
        based on vehicle position + mouse yaw/pitch (drag with left mouse to look around).
        """
        bp = self.world.get_blueprint_library().find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(width))
        bp.set_attribute("image_size_y", str(height))
        bp.set_attribute("fov", "90")

        # Initial transform; will be updated each frame in update_camera_transform()
        transform = carla.Transform(
            carla.Location(x=0, y=0, z=10),
            carla.Rotation(pitch=-20, yaw=0),
        )
        # Spawn WITHOUT attach_to so we can update transform manually for mouse look
        self.camera = self.world.spawn_actor(bp, transform)
        self._mouse_controlled = mouse_controlled
        self._camera_yaw_offset = 0.0
        self._camera_pitch_offset = -20.0  # Default pitch (degrees)
        import queue
        self._image_queue = queue.Queue()
        self.camera.listen(self._image_queue.put)
        return True

    def update_camera_transform(self, vehicle_transform, mouse_dx=0, mouse_dy=0):
        """
        Update camera position and rotation. Call each frame.
        vehicle_transform: ego vehicle transform
        mouse_dx, mouse_dy: mouse movement this frame (from pygame.mouse.get_rel)
        """
        if not self.camera or not self.camera.is_alive:
            return
        self._camera_yaw_offset += mouse_dx * CAMERA_MOUSE_SENSITIVITY
        self._camera_pitch_offset += mouse_dy * CAMERA_MOUSE_SENSITIVITY
        self._camera_pitch_offset = max(-89, min(30, self._camera_pitch_offset))

        loc = vehicle_transform.location
        rot = vehicle_transform.rotation
        yaw_rad = math.radians(rot.yaw + self._camera_yaw_offset)
        pitch_rad = math.radians(self._camera_pitch_offset)

        # Camera behind and above vehicle, offset by mouse yaw
        cam_x = loc.x - CAMERA_DISTANCE * math.cos(yaw_rad) * math.cos(pitch_rad)
        cam_y = loc.y - CAMERA_DISTANCE * math.sin(yaw_rad) * math.cos(pitch_rad)
        cam_z = loc.z + CAMERA_HEIGHT + CAMERA_DISTANCE * math.sin(pitch_rad)

        cam_location = carla.Location(x=cam_x, y=cam_y, z=cam_z)
        cam_rotation = carla.Rotation(
            pitch=-self._camera_pitch_offset,
            yaw=rot.yaw + self._camera_yaw_offset,
            roll=rot.roll,
        )
        self.camera.set_transform(carla.Transform(cam_location, cam_rotation))

    def setup_lane_invasion(self):
        """Spawn lane invasion sensor and listen for crossings."""
        if not self.ego_actor or not self.ego_actor.type_id.startswith("vehicle."):
            return False
        bp = self.world.get_blueprint_library().find("sensor.other.lane_invasion")
        self.lane_invasion_sensor = self.world.spawn_actor(
            bp, carla.Transform(), attach_to=self.ego_actor
        )
        weak_self = weakref.ref(self)
        self.lane_invasion_sensor.listen(
            lambda e: SensorManager._on_lane_invasion(weak_self, e)
        )
        return True

    def setup_ultrasonic_radar(self, num_sensors=5):
        """Setup multiple forward-facing radar sensors (like ultrasonic array) for obstacle detection."""
        if not self.ego_actor:
            return False
        
        bound_x = 0.5 + self.ego_actor.bounding_box.extent.x
        bound_z = 0.5 + self.ego_actor.bounding_box.extent.z
        
        bp = self.world.get_blueprint_library().find("sensor.other.radar")
        bp.set_attribute("horizontal_fov", "60")  # Wide FOV for forward detection
        bp.set_attribute("vertical_fov", "30")
        bp.set_attribute("range", "50")  # 50m range
        
        # Spawn multiple sensors: center, left, right, left-center, right-center
        positions = [
            (bound_x + 0.1, 0.0, bound_z + 0.1),      # Center
            (bound_x + 0.1, -0.5, bound_z + 0.1),     # Left-center
            (bound_x + 0.1, 0.5, bound_z + 0.1),      # Right-center
            (bound_x + 0.1, -1.0, bound_z + 0.1),     # Left
            (bound_x + 0.1, 1.0, bound_z + 0.1),      # Right
        ]
        
        weak_self = weakref.ref(self)
        for i, (x, y, z) in enumerate(positions[:num_sensors]):
            transform = carla.Transform(
                carla.Location(x=x, y=y, z=z),
                carla.Rotation(pitch=5.0),
            )
            sensor = self.world.spawn_actor(bp, transform, attach_to=self.ego_actor)
            sensor.listen(lambda data, idx=i: SensorManager._on_radar(weak_self, data, idx))
            self.radar_sensors.append(sensor)
        
        return True

    @staticmethod
    def _on_lane_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        self._invasion_callback_data["occurred"] = True
        solid_crossed = False
        for marking in event.crossed_lane_markings:
            t = marking.type
            if t == carla.LaneMarkingType.Solid or "Solid" in str(t):
                solid_crossed = True
                break
        self._invasion_callback_data["solid"] = solid_crossed

    @staticmethod
    def _on_radar(weak_self, radar_data, sensor_idx):
        """Process radar data: extract distance, velocity, azimuth for forward detection."""
        self = weak_self()
        if not self:
            return
        
        # Collect detections from this sensor
        sensor_detections = []
        for detect in radar_data:
            # Only consider forward-facing detections (azimuth near 0)
            azi_deg = math.degrees(detect.azimuth)
            if abs(azi_deg) < FRONT_RADAR_AZIMUTH_DEG:  # Front only, ignore sides
                sensor_detections.append({
                    "distance": detect.depth,
                    "velocity": detect.velocity,
                    "azimuth": azi_deg,
                })
        
        # Store detections (in sync mode, callbacks are sequential, so this is safe)
        # We'll combine all sensors' detections in tick()
        if not hasattr(self, '_sensor_detections_dict'):
            self._sensor_detections_dict = {}
        self._sensor_detections_dict[sensor_idx] = sensor_detections

    def tick(self):
        """Poll latest camera image and update lane invasion state from callback."""
        self._lane_invasion_occurred = self._invasion_callback_data["occurred"]
        self._crossed_solid = self._invasion_callback_data["solid"]
        # Reset for next frame; we only care about "ever happened" for overlay
        # (Alternatively keep True until cleared by logic; we keep True until next tick so HUD can show)
        while self._image_queue and not self._image_queue.empty():
            self._latest_image = self._image_queue.get_nowait()
        
        # Combine radar detections from all sensors
        if hasattr(self, '_sensor_detections_dict'):
            self._radar_detections = []
            for sensor_idx, detections in self._sensor_detections_dict.items():
                self._radar_detections.extend(detections)
            # Keep only closest detections (most relevant)
            self._radar_detections.sort(key=lambda x: x["distance"])
            self._radar_detections = self._radar_detections[:20]  # Top 20 closest

    def get_latest_image(self):
        return self._latest_image

    def get_lane_invasion_occurred(self):
        return self._lane_invasion_occurred

    def get_crossed_solid(self):
        return self._crossed_solid

    def get_radar_detections(self):
        """Return latest radar detections: list of {distance, velocity, azimuth}."""
        return self._radar_detections

    def reset_lane_invasion_display(self):
        """Call periodically if you want to clear the invasion flag for display."""
        self._invasion_callback_data["occurred"] = False
        self._invasion_callback_data["solid"] = False

    def destroy(self):
        if self.camera and self.camera.is_alive:
            self.camera.destroy()
            self.camera = None
        if self.lane_invasion_sensor and self.lane_invasion_sensor.is_alive:
            self.lane_invasion_sensor.destroy()
            self.lane_invasion_sensor = None
        for sensor in self.radar_sensors:
            if sensor and sensor.is_alive:
                sensor.destroy()
        self.radar_sensors.clear()


# ==============================================================================
# ADASController (FCW, ACC, lane/wrong-way logic)
# ==============================================================================

class ADASController(object):
    """Forward collision warning, emergency braking, ACC, and lane/wrong-way state."""

    def __init__(self, world, map_instance, ego_actor):
        self.world = world
        self.map_instance = map_instance
        self.ego_actor = ego_actor
        self.distance_to_lead_m = float("inf")
        self.relative_velocity_ms = 0.0
        self.ttc_s = float("inf")
        self.adas_status = "SAFE"  # SAFE | WARNING | BRAKE
        self.lane_status = "OK"    # OK | INVASION | WRONG WAY
        self.emergency_brake_active = False
        self._emergency_brake_value = 0.0
        self.emergency_reverse_active = False
        self._lead_vehicle = None
        self._last_lane_status = "OK"
        self._lane_invasion_display_until = 0.0  # time when to clear INVASION display
        self._lane_policy_enabled = False  # Toggle with L key: highway mode (strict lane policy)
        self._traffic_light_red = False     # True when we should stop for red light
        self._pedestrian_in_path = False    # True when pedestrian crossing in path
        self._lights_map = {}               # traffic_light id -> waypoint for trigger
        
        # Smoothing filters for stable readings
        self._distance_history = []
        self._velocity_history = []
        self._max_history = 5  # Keep last 5 readings for smoothing
        
        # Cruise control variables
        self.cruise_control_enabled = False
        self.cruise_target_speed_kmh = 50.0  # Default cruise speed
        self.cruise_control_active = False
        
        # Speed limit detection
        self.current_speed_limit_kmh = 50.0  # Default speed limit
        self.speed_limit_auto_mode = False  # Toggle with R key
        
        # Highway detection
        self.is_on_highway = False
        
        # Emergency brake override
        self.emergency_brake_override = False
        
        # Paper-aligned components
        self._fusion_engine = FusionEngine()
        self._decision_engine = DecisionEngine(mode="proposed")
        self._paper_state = AdasState.SAFE
        self._paper_brake_cmd = 0.0
        self._sensor_readings = SensorReadings()
        
        # Paper metrics logging
        script_dir = os.path.dirname(os.path.abspath(__file__))
        out_csv = os.path.join(script_dir, "adas_final1_paper_metrics.csv")
        self._paper_metrics = MetricsLogger(out_csv)

    def _get_ego_transform_velocity(self):
        if not self.ego_actor or not self.ego_actor.is_alive:
            return None, None
        return self.ego_actor.get_transform(), self.ego_actor.get_velocity()

    def _velocity_to_speed_ms(self, v):
        return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

    def _smooth_value(self, value, history_list, max_len):
        """Apply moving average smoothing to reduce fluctuations."""
        history_list.append(value)
        if len(history_list) > max_len:
            history_list.pop(0)
        return sum(history_list) / len(history_list) if history_list else value

    def update_forward_detection(self, radar_detections=None):
        """Simple and working forward detection."""
        self.distance_to_lead_m = float("inf")
        self.relative_velocity_ms = 0.0
        self.ttc_s = float("inf")
        self._lead_vehicle = None

        t, v_ego = self._get_ego_transform_velocity()
        if t is None or v_ego is None:
            return

        ego_loc = t.location
        ego_forward = t.get_forward_vector()
        ego_speed_ms = self._velocity_to_speed_ms(v_ego)

        # Simple transform-based detection
        vehicles = self.world.get_actors().filter("vehicle.*")
        closest_dist = float("inf")
        
        for actor in vehicles:
            if actor.id == self.ego_actor.id or not actor.is_alive:
                continue
            
            other_loc = actor.get_location()
            to_other = other_loc - ego_loc
            dist = math.sqrt(to_other.x**2 + to_other.y**2 + to_other.z**2)
            
            # Simple check - vehicles within 20m and ahead
            if 0.5 <= dist <= 20.0:
                if dist > 0:
                    dir_norm = to_other / dist
                    dot = ego_forward.x * dir_norm.x + ego_forward.y * dir_norm.y + ego_forward.z * dir_norm.z
                    
                    if dot > 0.7:  # Within ~45° cone
                        if dist < closest_dist:
                            closest_dist = dist
                            other_vel = actor.get_velocity()
                            other_speed = self._velocity_to_speed_ms(other_vel)
                            
                            # Simple relative velocity
                            rel_vel = ego_speed_ms - other_speed
                            if rel_vel > 0:  # Approaching
                                self.relative_velocity_ms = rel_vel
                                self.ttc_s = dist / max(rel_vel, 0.1)
                            else:
                                self.relative_velocity_ms = 0
                                self.ttc_s = float("inf")
                            
                            self.distance_to_lead_m = dist
                            self._lead_vehicle = actor
        
        # Update ADAS status
        if self.ttc_s < float("inf"):
            if self.ttc_s <= 2.0:
                self.adas_status = "BRAKE"
            elif self.ttc_s <= 4.0:
                self.adas_status = "WARNING"
            else:
                self.adas_status = "SAFE"
        else:
            self.adas_status = "SAFE"
        
        # Debug output
        if self.distance_to_lead_m <= 20.0:
            print(f"[DETECTION] Dist: {self.distance_to_lead_m:.1f}m, RelVel: {self.relative_velocity_ms:.1f}m/s, TTC: {self.ttc_s:.1f}s, Status: {self.adas_status}")
        else:
            print(f"[DETECTION] No vehicle ahead - Status: {self.adas_status}")

    def get_collision_avoidance_control(self, current_throttle, current_brake):
        """Return (throttle, brake, active) with improved logic and no overriding issues."""
        # Store original inputs for debugging
        orig_throttle, orig_brake = current_throttle, current_brake
        
        # Initialize output
        out_throttle = current_throttle
        out_brake = current_brake
        active = False
        
        # Check emergency brake override first - but allow emergency braking for safety
        if self.emergency_brake_override:
            # Still allow emergency braking for safety even with override
            if self.distance_to_lead_m < ACC_SAFE_DISTANCE_MIN_M and self.relative_velocity_ms > 0.5:
                out_throttle, out_brake, active = 0.0, 1.0, True
                self.emergency_reverse_active = True
                return out_throttle, out_brake, active
            return current_throttle, current_brake, False  # No ADAS intervention when override is active
        
        # Priority 1: Traffic light - immediate stop (highest priority)
        if self._traffic_light_red:
            # Get distance to traffic light for progressive braking
            try:
                ego_loc = self.ego_actor.get_location()
                ego_wp = self.map_instance.get_waypoint(ego_loc, project_to_road=True, lane_type=carla.LaneType.Driving)
                if ego_wp:
                    # Find the closest traffic light
                    lights = self.world.get_actors().filter("*traffic_light*")
                    min_distance = float("inf")
                    for tl in lights:
                        tl_loc = tl.get_location()
                        distance = tl_loc.distance(ego_loc)
                        if distance < min_distance and distance <= TLIGHT_STOP_DISTANCE_M:
                            min_distance = distance
                    
                    if min_distance < float("inf"):
                        # Progressive braking based on distance to stop line
                        if min_distance > 15.0:
                            out_throttle, out_brake, active = 0.0, 0.3, True  # Gentle braking
                        elif min_distance > 8.0:
                            out_throttle, out_brake, active = 0.0, 0.6, True  # Moderate braking
                        else:
                            out_throttle, out_brake, active = 0.0, 1.0, True  # Full emergency braking
                            
                        print(f"[TRAFFIC] Progressive braking for traffic light: {min_distance:.1f}m, brake: {out_brake:.1f}")
                    else:
                        out_throttle, out_brake, active = 0.0, 1.0, True  # Default full brake
                else:
                    out_throttle, out_brake, active = 0.0, 1.0, True  # Default full brake
            except Exception:
                out_throttle, out_brake, active = 0.0, 1.0, True  # Fallback full brake
                
            self.emergency_reverse_active = True
            return out_throttle, out_brake, active
        
        # Priority 2: Pedestrian in path - immediate stop
        if self._pedestrian_in_path:
            out_throttle, out_brake, active = 0.0, 1.0, True
            self.emergency_reverse_active = True
            return out_throttle, out_brake, active
        
        # Simple TTC-based braking
        ttc_braking = False
        if self.ttc_s < float("inf"):
            if self.ttc_s <= 2.0:  # Critical
                out_throttle, out_brake, active = 0.0, 1.0, True
                ttc_braking = True
                print(f"[BRAKE] CRITICAL: TTC {self.ttc_s:.1f}s - FULL BRAKE")
            elif self.ttc_s <= 3.0:  # Danger
                out_throttle, out_brake, active = 0.0, 0.8, True
                ttc_braking = True
                print(f"[BRAKE] DANGER: TTC {self.ttc_s:.1f}s - STRONG BRAKE")
            elif self.ttc_s <= 4.0:  # Warning
                out_throttle, out_brake, active = 0.0, 0.5, True
                ttc_braking = True
                print(f"[BRAKE] WARNING: TTC {self.ttc_s:.1f}s - MODERATE BRAKE")
            else:
                out_throttle, out_brake, active = current_throttle, current_brake, False
        
        # Reset emergency reverse when safe
        if self.emergency_reverse_active and self.distance_to_lead_m > 20.0:
            self.emergency_reverse_active = False
            print("Emergency brake automatically disabled - road ahead is clear")
        
        # Priority 5: Warning zone with context-aware braking (only if not already in TTC braking)
        if not ttc_braking:
            if self.distance_to_lead_m < 30.0 and self.relative_velocity_ms > 0.1:
                if self.is_on_highway:
                    # Highway: gentle speed reduction
                    if current_throttle > 0.3:
                        out_throttle, out_brake, active = 0.0, 0.3, True
                    else:
                        out_throttle, out_brake, active = current_throttle * 0.5, 0.2, True
                else:
                    # City: moderate braking
                    if current_throttle > 0.2:
                        out_throttle, out_brake, active = 0.0, 0.6, True
                    elif current_brake < 0.7:
                        out_throttle, out_brake, active = 0.0, 0.7, True
                    else:
                        out_throttle, out_brake, active = 0.0, current_brake, True
            
            # Priority 6: Caution zone - mild intervention
            elif self.distance_to_lead_m < ACC_SAFE_DISTANCE_CRUISE_M:
                if current_throttle > 0.4:
                    out_throttle, out_brake, active = current_throttle * 0.6, current_brake, True
                elif self.relative_velocity_ms > 1.0:  # Approaching too fast
                    out_throttle, out_brake, active = current_throttle * 0.3, 0.2, True
        
        # Always log control state for debugging
        if active:
            print(f"[CONTROL] ACTIVE - Throttle: {orig_throttle:.2f} -> {out_throttle:.2f}, "
                  f"Brake: {orig_brake:.2f} -> {out_brake:.2f}, "
                  f"Dist: {self.distance_to_lead_m:.1f}m, RelVel: {self.relative_velocity_ms:.1f}m/s, "
                  f"Emergency: {self.emergency_brake_active}")
        else:
            print(f"[CONTROL] INACTIVE - Throttle: {orig_throttle:.2f}, "
                  f"Brake: {orig_brake:.2f}, "
                  f"Dist: {self.distance_to_lead_m:.1f}m, RelVel: {self.relative_velocity_ms:.1f}m/s")
        
        return out_throttle, out_brake, active

    def update_paper_aligned_components(self, radar_detections=None):
        """Update paper-aligned fusion and decision engine."""
        try:
            # Update sensor readings from existing detection logic
            self._sensor_readings.lidar_distance_m = self.distance_to_lead_m
            self._sensor_readings.radar_relative_velocity_ms = self.relative_velocity_ms
            self._sensor_readings.ultrasonic_distance_m = self.distance_to_lead_m if self.distance_to_lead_m < 3.0 else float("inf")
            self._sensor_readings.timestamp = time.time()
            
            # Get ego speed
            t, v_ego = self._get_ego_transform_velocity()
            ego_speed_ms = self._velocity_to_speed_ms(v_ego) if v_ego else 0.0
            
            # Compute fused state
            fused = self._fusion_engine.compute_fused_state(
                self._sensor_readings, self._sensor_readings, 
                self._sensor_readings, self._sensor_readings, 
                ego_speed_ms
            )
            
            # Get paper decision
            self._paper_state, self._paper_brake_cmd = self._decision_engine.decide(fused)
            
            # Log metrics
            self._paper_metrics.log(
                ts=time.time(),
                mode="proposed",
                fused=fused,
                state=self._paper_state,
                brake_cmd=self._paper_brake_cmd,
                sensor_ts=self._sensor_readings.timestamp
            )
            
        except Exception as e:
            print(f"Warning: Paper component update failed: {e}")

    def update_lateral_warning(self):
        """Update lateral warning for paper-aligned system - detects vehicles too close on sides."""
        try:
            # Update paper sensor readings for lateral detection
            self._sensor_readings.ir_left_distance_m = float("inf")
            self._sensor_readings.ir_right_distance_m = float("inf")
            
            t, v_ego = self._get_ego_transform_velocity()
            if t is None or v_ego is None:
                return
                
            ego_loc = t.location
            ego_forward = t.get_forward_vector()
            ego_right = t.get_right_vector()
            
            vehicles = self.world.get_actors().filter("vehicle.*")
            for actor in vehicles:
                if actor.id == self.ego_actor.id or not actor.is_alive:
                    continue
                    
                other_loc = actor.get_location()
                dx = other_loc.x - ego_loc.x
                dy = other_loc.y - ego_loc.y
                dz = other_loc.z - ego_loc.z
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                
                # Only check vehicles within lateral detection range
                if dist > 10.0:  # 10m lateral detection range
                    continue
                    
                # Check if vehicle is to the left or right (not in front)
                forward_dot = (ego_forward.x * dx + ego_forward.y * dy + ego_forward.z * dz) / dist
                if abs(forward_dot) < 0.7:  # Vehicle is roughly to the side
                    lateral_dot = (ego_right.x * dx + ego_right.y * dy + ego_right.z * dz) / dist
                    lateral_dist = abs(lateral_dot * dist)
                    
                    if lateral_dot > 0:  # Vehicle is to the right
                        self._sensor_readings.ir_right_distance_m = min(self._sensor_readings.ir_right_distance_m, lateral_dist)
                    else:  # Vehicle is to the left
                        self._sensor_readings.ir_left_distance_m = min(self._sensor_readings.ir_left_distance_m, lateral_dist)
                        
        except Exception as e:
            print(f"Warning: Lateral warning update failed: {e}")

    def update_speed_limit(self):
        """Detect speed limit signs and update current speed limit."""
        try:
            ego_loc = self.ego_actor.get_location()
            ego_wp = self.map_instance.get_waypoint(ego_loc, project_to_road=True, lane_type=carla.LaneType.Driving)
            if ego_wp:
                # CARLA speed limits are in km/h
                speed_limit = ego_wp.get_speed_limit()
                if speed_limit > 0 and speed_limit != self.current_speed_limit_kmh:
                    self.current_speed_limit_kmh = speed_limit
                    print("Speed limit detected: %s km/h" % speed_limit)
                    
                    # Auto-adjust cruise speed to speed limit if in auto mode
                    if self.speed_limit_auto_mode and self.cruise_control_enabled:
                        self.cruise_target_speed_kmh = min(self.cruise_target_speed_kmh, speed_limit)
                        print("Cruise speed adjusted to: %s km/h" % self.cruise_target_speed_kmh)
                
                # Detect if on highway (speed limit >= 80 km/h)
                self.is_on_highway = speed_limit >= 80.0
        except Exception:
            pass

    def toggle_speed_limit_auto(self):
        """Toggle automatic speed limit following mode."""
        self.speed_limit_auto_mode = not self.speed_limit_auto_mode
        return self.speed_limit_auto_mode

    def toggle_emergency_brake_override(self):
        """Toggle emergency brake override to disable automatic braking."""
        self.emergency_brake_override = not self.emergency_brake_override
        status = "DISABLED" if self.emergency_brake_override else "ENABLED"
        print(f"🚨 Emergency brake override: {status}")
        return self.emergency_brake_override

    def get_cruise_control(self, current_speed_kmh):
        """Return throttle/brake for cruise control with speed limit awareness."""
        if not self.cruise_control_enabled or self.emergency_brake_override:
            return 0.0, 0.0, False
        
        # Apply speed limit if in auto mode
        target_speed = self.cruise_target_speed_kmh
        if self.speed_limit_auto_mode:
            target_speed = min(target_speed, self.current_speed_limit_kmh)
        
        speed_error = target_speed - current_speed_kmh
        
        if speed_error > 5.0:  # Need to accelerate
            return min(0.6, speed_error / 50.0), 0.0, True
        elif speed_error < -2.0:  # Need to brake
            return 0.0, min(0.4, abs(speed_error) / 30.0), True
        else:  # Maintain speed
            return 0.1, 0.0, True  # Light throttle to maintain

    def toggle_cruise_control(self):
        """Toggle cruise control on/off."""
        self.cruise_control_enabled = not self.cruise_control_enabled
        return self.cruise_control_enabled

    def set_cruise_speed(self, speed_kmh):
        """Set cruise control target speed."""
        self.cruise_target_speed_kmh = max(10.0, min(120.0, speed_kmh))

    def update_traffic_light(self):
        """Set traffic light status - simplified logic to prevent false signals."""
        old_state = "RED" if self._traffic_light_red else "GREEN"
        self._traffic_light_red = False
        
        try:
            ego_loc = self.ego_actor.get_location()
            ego_transform = self.ego_actor.get_transform()
            ego_wp = self.map_instance.get_waypoint(ego_loc, project_to_road=True, lane_type=carla.LaneType.Driving)
            if ego_wp is None:
                return
            
            # Get ego vehicle's forward direction
            ego_forward = ego_transform.get_forward_vector()
            
            lights = self.world.get_actors().filter("*traffic_light*")
            closest_light = None
            closest_distance = float("inf")
            
            for tl in lights:
                if not tl.is_alive:
                    continue
                
                tl_loc = tl.get_location()
                to_tl = tl_loc - ego_loc
                tl_distance = math.sqrt(to_tl.x**2 + to_tl.y**2 + to_tl.z**2)
                
                # Only consider lights within detection range
                if tl_distance > TLIGHT_STOP_DISTANCE_M:
                    continue
                
                # Check if light is directly ahead of us - very strict filtering
                if tl_distance > 0:
                    to_tl_normalized = carla.Vector3D(to_tl.x/tl_distance, to_tl.y/tl_distance, to_tl.z/tl_distance)
                    dot_product = ego_forward.x * to_tl_normalized.x + ego_forward.y * to_tl_normalized.y + ego_forward.z * to_tl_normalized.z
                    
                    # Very strict front-only check - must be almost directly ahead
                    if dot_product <= 0.9:  # Must be within ~25° cone (much stricter)
                        continue
                    
                    # Additional lateral check - ensure light is not to the side
                    ego_right = ego_transform.get_right_vector()
                    lateral_offset = abs(ego_right.x * to_tl.x + ego_right.y * to_tl.y + ego_right.z * to_tl.z)
                    if lateral_offset > 1.0:  # Must be within 1m lateral offset
                        continue
                
                # Find the closest light
                if tl_distance < closest_distance:
                    closest_distance = tl_distance
                    closest_light = tl
            
            # Process the closest light
            if closest_light:
                if closest_light.state == carla.TrafficLightState.Red:
                    self._traffic_light_red = True
                    print(f"[TRAFFIC] RED light at {closest_distance:.1f}m")
                elif closest_light.state == carla.TrafficLightState.Yellow:
                    if closest_distance < 10.0:  # Only stop for yellow if close
                        self._traffic_light_red = True
                        print(f"[TRAFFIC] YELLOW light at {closest_distance:.1f}m - STOPPING")
                    else:
                        print(f"[TRAFFIC] YELLOW light at {closest_distance:.1f}m - PROCEED")
                else:
                    print(f"[TRAFFIC] GREEN light at {closest_distance:.1f}m - PROCEED")
            else:
                self._traffic_light_red = False
                    
        except Exception as e:
            print(f"Warning: Traffic light detection failed: {e}")
        
        # Notify of traffic light state changes
        new_state = "RED" if self._traffic_light_red else "GREEN"
        if old_state != new_state:
            print(f"Traffic light: {new_state}")

    def update_pedestrian_crossing(self):
        """Set _pedestrian_in_path if a pedestrian is crossing in our path ahead."""
        self._pedestrian_in_path = False
        try:
            t = self.ego_actor.get_transform()
            ego_loc = t.location
            ego_forward = t.get_forward_vector()
            ego_right = t.get_right_vector()
            walkers = self.world.get_actors().filter("*walker*")
            for w in walkers:
                if not w.is_alive or "walker" not in w.type_id.lower():
                    continue
                wloc = w.get_location()
                dx = wloc.x - ego_loc.x
                dy = wloc.y - ego_loc.y
                dz = wloc.z - ego_loc.z
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                if dist < 1 or dist > PEDESTRIAN_FRONT_M:
                    continue
                # In front
                dot = (ego_forward.x * dx + ego_forward.y * dy + ego_forward.z * dz) / dist
                if dot < 0.7:
                    continue
                lateral = abs(ego_right.x * dx + ego_right.y * dy + ego_right.z * dz)
                if lateral < PEDESTRIAN_LATERAL_M:
                    self._pedestrian_in_path = True
                    return
        except Exception:
            pass

    def update_adas_status(self):
        """Set ADAS status and emergency brake flag with improved priority logic and consistency."""
        # Store previous state to detect changes
        previous_status = self.adas_status
        previous_emergency = self.emergency_brake_active
        
        # Reset emergency brake flag - will be set based on conditions
        self.emergency_brake_active = False
        
        # Priority 1: Traffic light - immediate brake (highest priority)
        if self._traffic_light_red:
            self.adas_status = "RED LIGHT"
            self.emergency_brake_active = True
        # Priority 2: Pedestrian in path - immediate brake
        elif self._pedestrian_in_path:
            self.adas_status = "PEDESTRIAN"
            self.emergency_brake_active = True
        # Priority 3: Critical distance - immediate brake regardless of TTC
        elif self.distance_to_lead_m < 3.0:  # Critical distance threshold
            self.adas_status = "CRITICAL"
            self.emergency_brake_active = True
        # Priority 4: Emergency TTC with relative velocity check
        elif (self.ttc_s < FCW_TTC_EMERGENCY_S and 
              self.relative_velocity_ms > 0.5 and  # Only if approaching
              self.distance_to_lead_m < 20.0):  # Reasonable distance range
            self.adas_status = "EMERGENCY"
            self.emergency_brake_active = True
        # Priority 5: Warning TTC
        elif (self.ttc_s < FCW_TTC_WARNING_S and 
              self.relative_velocity_ms > 0.1 and
              self.distance_to_lead_m < 30.0):
            self.adas_status = "WARNING"
            self.emergency_brake_active = False
        # Priority 6: Distance-based warnings with velocity consideration
        elif (self.distance_to_lead_m < ACC_SAFE_DISTANCE_MIN_M and 
              self.relative_velocity_ms > 0.2):
            self.adas_status = "BRAKE"
            self.emergency_brake_active = True
        elif self.distance_to_lead_m < ACC_SAFE_DISTANCE_MAX_M:
            self.adas_status = "WARNING"
            self.emergency_brake_active = False
        elif self.distance_to_lead_m < ACC_SAFE_DISTANCE_CRUISE_M:
            self.adas_status = "CAUTION"
            self.emergency_brake_active = False
        else:
            self.adas_status = "SAFE"
            self.emergency_brake_active = False
        
        # Log state changes for debugging
        if (previous_status != self.adas_status or 
            previous_emergency != self.emergency_brake_active):
            print(f"[ADAS] Status: {previous_status} -> {self.adas_status}, "
                  f"Emergency: {previous_emergency} -> {self.emergency_brake_active}, "
                  f"Dist: {self.distance_to_lead_m:.1f}m, TTC: {self.ttc_s:.2f}s, "
                  f"RelVel: {self.relative_velocity_ms:.1f}m/s")

    def update_wrong_way(self):
        """Set lane_status WRONG WAY if driving opposite to lane direction."""
        try:
            ego_loc = self.ego_actor.get_location()
            waypoint = self.map_instance.get_waypoint(
                ego_loc, project_to_road=True, lane_type=carla.LaneType.Driving
            )
            if waypoint is None:
                if self.lane_status == "WRONG WAY":
                    self.lane_status = "OK"
                return
            ego_forward = self.ego_actor.get_transform().get_forward_vector()
            wp_forward = waypoint.transform.get_forward_vector()
            dot = (
                ego_forward.x * wp_forward.x
                + ego_forward.y * wp_forward.y
                + ego_forward.z * wp_forward.z
            )
            if dot < WRONG_WAY_DOT_THRESHOLD:
                self.lane_status = "WRONG WAY"
            else:
                if self.lane_status == "WRONG WAY":
                    self.lane_status = "OK"
        except Exception:
            pass

    def set_lane_invasion(self, invasion_occurred, crossed_solid):
        """Called when lane invasion sensor fires. Sets INVASION for display duration."""
        # Only set lane invasion if vehicle is moving (speed > 5 km/h) to avoid false positives
        if invasion_occurred:
            t, v = self._get_ego_transform_velocity()
            if t and v:
                speed_kmh = self._velocity_to_speed_ms(v) * 3.6  # Convert m/s to km/h
                if speed_kmh > 5.0:  # Only count lane invasion when moving
                    if self.lane_status != "WRONG WAY":
                        self.lane_status = "INVASION"
                    self._lane_invasion_display_until = time.time() + 3.0  # show for 3 s

    def reset_lane_status_if_ok(self):
        """Clear INVASION display after timeout; WRONG WAY is updated each frame in update_wrong_way."""
        current_time = time.time()
        
        # Always handle wrong way reset first
        if self.lane_status == "WRONG WAY":
            # Wrong way status is handled in update_wrong_way, just continue
            return
        
        # Handle lane invasion timeout with improved logic
        if self.lane_status == "INVASION":
            if current_time > self._lane_invasion_display_until:
                old_status = self.lane_status
                self.lane_status = "OK"
                if old_status != "OK":
                    print(f"[LANE] Invasion cleared - Status: NORMAL (after {current_time - self._lane_invasion_display_until + 3.0:.1f}s)")
            else:
                # Still in invasion period, ensure it doesn't run indefinitely
                if current_time > self._lane_invasion_display_until + 5.0:  # Max 5 seconds
                    print("[LANE] Force clearing invasion - timeout exceeded")
                    self.lane_status = "OK"
        
        # Additional safety: ensure no stuck states
        elif self.lane_status not in ["OK", "WRONG WAY", "INVASION"]:
            print(f"[LANE] Invalid lane status '{self.lane_status}' - resetting to OK")
            self.lane_status = "OK"

    def get_lane_keeping_steer(self):
        """
        When lane invasion or wrong-way is detected AND lane policy is enabled,
        compute steering to pull vehicle back to lane. Decides left/right from lane geometry.
        Returns (steer_correction, active, turn_direction): turn_direction is "LEFT", "RIGHT", or None.
        """
        if not self._lane_policy_enabled:
            return 0.0, False, None
        if self.lane_status not in ("INVASION", "WRONG WAY"):
            return 0.0, False, None
        try:
            ego_transform = self.ego_actor.get_transform()
            ego_loc = ego_transform.location
            ego_yaw = math.radians(ego_transform.rotation.yaw)

            waypoint = self.map_instance.get_waypoint(
                ego_loc, project_to_road=True, lane_type=carla.LaneType.Driving
            )
            if waypoint is None:
                return 0.0, False, None

            wp_loc = waypoint.transform.location
            wp_forward = waypoint.transform.get_forward_vector()

            if self.lane_status == "WRONG WAY":
                # Steer to align with lane direction (turn toward lane forward)
                desired_yaw = math.atan2(wp_forward.y, wp_forward.x)
                yaw_diff = desired_yaw - ego_yaw
                while yaw_diff > math.pi:
                    yaw_diff -= 2 * math.pi
                while yaw_diff < -math.pi:
                    yaw_diff += 2 * math.pi
                steer = -yaw_diff * LANE_KEEPING_STEER_GAIN
            else:
                # INVASION: steer toward lane center (waypoint ahead) - direction from ego to lane
                next_wps = waypoint.next(5.0)
                if next_wps:
                    target_loc = next_wps[0].transform.location
                else:
                    target_loc = wp_loc
                dx = target_loc.x - ego_loc.x
                dy = target_loc.y - ego_loc.y
                to_target = math.atan2(dy, dx)
                yaw_diff = to_target - ego_yaw
                while yaw_diff > math.pi:
                    yaw_diff -= 2 * math.pi
                while yaw_diff < -math.pi:
                    yaw_diff += 2 * math.pi
                steer = yaw_diff * LANE_KEEPING_STEER_GAIN

            steer = max(-LANE_KEEPING_MAX_STEER, min(LANE_KEEPING_MAX_STEER, steer))
            # CARLA: steer > 0 = turn right, steer < 0 = turn left
            turn_direction = "RIGHT" if steer > 0.02 else ("LEFT" if steer < -0.02 else None)
            return steer, True, turn_direction
        except Exception:
            return 0.0, False, None

    def get_control_modification(self, throttle, brake, delta_time, speed_kmh=None):
        """
        Returns (throttle, brake) after FCW/ACC/emergency brake and distance-based speed limit.
        When distance to lead drops below threshold, speed is reduced and limited until safe distance.
        """
        out_throttle = throttle
        out_brake = brake
        d = self.distance_to_lead_m

        # Traffic light: stop on red
        if self._traffic_light_red:
            out_throttle = 0.0
            out_brake = max(out_brake, 0.6)

        # Pedestrian crossing: wait
        if self._pedestrian_in_path:
            out_throttle = 0.0
            out_brake = max(out_brake, 0.5)

        # Emergency brake: ramp up brake, zero throttle
        if self.emergency_brake_active:
            self._emergency_brake_value = min(
                1.0,
                self._emergency_brake_value + EMERGENCY_BRAKE_RAMP_UP * delta_time,
            )
            out_throttle = 0.0
            out_brake = max(out_brake, self._emergency_brake_value)
        else:
            self._emergency_brake_value = max(
                0.0,
                self._emergency_brake_value - EMERGENCY_BRAKE_RAMP_UP * delta_time,
            )

        # Warning: reduce throttle
        if self.adas_status == "WARNING":
            out_throttle = out_throttle * THROTTLE_REDUCTION_WARNING

        # Distance-based speed limit: when within threshold distance, reduce and cap speed
        # until vehicle is back to safe cruising distance
        if d < ACC_SAFE_DISTANCE_CRUISE_M and speed_kmh is not None and not self.emergency_brake_active:
            if d < ACC_DISTANCE_THRESHOLD_M:
                # Below threshold: limit speed to MAX_SPEED_WHEN_TOO_CLOSE_KMH
                max_allowed_kmh = MAX_SPEED_WHEN_TOO_CLOSE_KMH
            else:
                # Between threshold and safe cruise: interpolate allowed speed
                frac = (d - ACC_DISTANCE_THRESHOLD_M) / max(0.01, ACC_SAFE_DISTANCE_CRUISE_M - ACC_DISTANCE_THRESHOLD_M)
                max_allowed_kmh = MAX_SPEED_WHEN_TOO_CLOSE_KMH + frac * (80.0 - MAX_SPEED_WHEN_TOO_CLOSE_KMH)
            if speed_kmh > max_allowed_kmh:
                out_throttle = 0.0
                out_brake = max(out_brake, 0.5)

        # ACC: if vehicle ahead and too close, reduce throttle / gentle brake to keep 8--12 m
        if self._lead_vehicle and self._lead_vehicle.is_alive and not self.emergency_brake_active:
            if d < ACC_SAFE_DISTANCE_MIN_M:
                out_throttle = 0.0
                out_brake = max(out_brake, 0.4)
            elif d < ACC_SAFE_DISTANCE_MAX_M:
                # Proportional reduction
                out_throttle = out_throttle * max(0.0, (d - ACC_SAFE_DISTANCE_MIN_M) / (ACC_SAFE_DISTANCE_MAX_M - ACC_SAFE_DISTANCE_MIN_M))
                if out_brake < 0.2:
                    out_brake = 0.15

        return (out_throttle, out_brake)

    def get_display_data(self, lane_keeping_active=False, lane_keep_turn_direction=None):
        """Return dict for HUD. lane_keep_turn_direction: 'LEFT', 'RIGHT', or None."""
        return {
            "distance_to_lead_m": self.distance_to_lead_m,
            "ttc_s": self.ttc_s,
            "adas_status": self.adas_status,
            "lane_status": self.lane_status,
            "emergency_brake_active": self.emergency_brake_active,
            "lane_keeping_active": lane_keeping_active,
            "lane_keep_turn_direction": lane_keep_turn_direction,
            "lane_policy_enabled": self._lane_policy_enabled,
            "traffic_light_red": self._traffic_light_red,
            "pedestrian_in_path": self._pedestrian_in_path,
            "paper_state": self._paper_state.value if hasattr(self, '_paper_state') else "SAFE",
            "paper_brake_cmd": self._paper_brake_cmd if hasattr(self, '_paper_brake_cmd') else 0.0,
        }


# ==============================================================================
# HUD
# ==============================================================================

class HUD(object):
    """On-screen display: speed, distance to front, TTC, ADAS status. Top-right lane warning only (no red overlay)."""

    def __init__(self, width, height):
        self.dim = (width, height)
        self._font = pygame.font.Font(pygame.font.get_default_font(), HUD_FONT_SIZE)
        self._font_warning = pygame.font.Font(pygame.font.get_default_font(), 20)

    def render(self, display, speed_kmh, adas_data, reverse=False):
        """HUD: main details only - distance, TTC, speed, ADAS, lane, traffic light, pedestrian."""
        dist = adas_data.get("distance_to_lead_m", float("inf"))
        ttc = adas_data.get("ttc_s", float("inf"))
        dist_str = "%.1f m" % dist if dist < 1e5 else "--"
        ttc_str = "%.2f s" % ttc if ttc < 1e5 else "--"
        lines = [
            "Speed: %.0f km/h" % speed_kmh,
            "Distance to front: %s" % dist_str,
            "TTC: %s" % ttc_str,
            "ADAS: %s" % adas_data.get("adas_status", "SAFE"),
            "Lane: %s" % adas_data.get("lane_status", "OK"),
        ]
        if adas_data.get("traffic_light_red"):
            lines.append("Traffic: RED - Stopping")
        else:
            lines.append("Traffic: OK")
        if adas_data.get("pedestrian_in_path"):
            lines.append("Pedestrian: Crossing - Wait")
        else:
            lines.append("Pedestrian: Clear")
        if adas_data.get("emergency_brake_active"):
            lines.append("EMERGENCY BRAKE")
        elif adas_data.get("adas_status") == "WARNING":
            lines.append("WARNING: Vehicle Ahead")
        if adas_data.get("lane_keeping_active"):
            turn_dir = adas_data.get("lane_keep_turn_direction")
            if turn_dir:
                lines.append("Lane keep: Turn %s" % turn_dir)
        
        # Paper-aligned state display
        paper_state = adas_data.get("paper_state", "SAFE")
        paper_brake = adas_data.get("paper_brake_cmd", 0.0)
        lines.append(f"Paper: {paper_state} (Brake: {paper_brake:.2f})")

        y = 10
        for text in lines:
            color = (255, 255, 255)
            if "BRAKE" in text or "EMERGENCY" in text:
                color = (255, 80, 80)
            elif "WARNING" in text:
                color = (255, 200, 100)
            surf = self._font.render(text, True, color)
            display.blit(surf, (10, y))
            y += 28

        # Top-right lane warning only (no red overlay) - shows until back in lane
        lane_status = adas_data.get("lane_status", "OK")
        if lane_status != "OK":
            msg = "LANE WARNING" if lane_status == "INVASION" else "WRONG WAY"
            warn_surf = self._font_warning.render(msg, True, (255, 100, 100))
            pad = 15
            box_w = warn_surf.get_width() + pad * 2
            box_h = warn_surf.get_height() + pad * 2
            box = pygame.Surface((box_w, box_h))
            box.fill((40, 20, 20))
            box.set_alpha(200)
            rx, ry = self.dim[0] - box_w - 10, 10
            display.blit(box, (rx, ry))
            display.blit(warn_surf, (rx + pad, ry + pad))


# ==============================================================================
# Data Logger for CSV Output
# ==============================================================================

class DataLogger:
    """Handles CSV data logging for ADAS TTC analysis."""
    
    def __init__(self, filename=None):
        if filename is None:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            filename = os.path.join(script_dir, "adas_final1_ttc_data.csv")
        self.filename = filename
        self.csv_file = None
        self.csv_writer = None
        
    def initialize_logging(self):
        """Initialize CSV file with headers."""
        try:
            self.csv_file = open(self.filename, 'w', newline='', encoding='utf-8')
            fieldnames = [
                'timestamp', 'distance_to_lead_m', 'ego_speed_kmh', 'ttc_s', 
                'adas_status', 'lane_status', 'emergency_brake_active'
            ]
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=fieldnames)
            self.csv_writer.writeheader()
            print(f"CSV logging initialized: {self.filename}")
        except Exception as e:
            print(f"Failed to initialize CSV logging: {e}")
    
    def log_data_point(self, timestamp, distance, ego_speed_kmh, ttc, adas_status, lane_status, emergency_brake):
        """Log a single data point with clean numerical values."""
        try:
            if self.csv_writer:
                # Replace inf/nan with numerical values
                # Handle both float inf and string "inf"
                clean_distance = 0.0 if (
                    distance is None or 
                    distance == float('inf') or 
                    (isinstance(distance, str) and distance.lower() == 'inf') or
                    math.isnan(distance) if isinstance(distance, (int, float)) else False
                ) else float(distance)
                
                clean_ttc = 0.0 if (
                    ttc is None or 
                    ttc == float('inf') or 
                    (isinstance(ttc, str) and ttc.lower() == 'inf') or
                    math.isnan(ttc) if isinstance(ttc, (int, float)) else False
                ) else float(ttc)
                
                clean_speed = 0.0 if (
                    ego_speed_kmh is None or 
                    ego_speed_kmh == float('inf') or 
                    (isinstance(ego_speed_kmh, str) and ego_speed_kmh.lower() == 'inf') or
                    math.isnan(ego_speed_kmh) if isinstance(ego_speed_kmh, (int, float)) else False
                ) else float(ego_speed_kmh)
                
                row = {
                    'timestamp': timestamp,
                    'distance_to_lead_m': clean_distance,
                    'ego_speed_kmh': clean_speed,
                    'ttc_s': clean_ttc,
                    'adas_status': adas_status,
                    'lane_status': lane_status,
                    'emergency_brake_active': emergency_brake
                }
                self.csv_writer.writerow(row)
        except Exception as e:
            print(f"Failed to log data point: {e}")
    
    def close_logging(self):
        """Close CSV file."""
        try:
            if self.csv_file:
                self.csv_file.close()
                print(f"CSV logging closed: {self.filename}")
        except Exception as e:
            print(f"Failed to close CSV logging: {e}")

# ==============================================================================
# Main: connect, load map, spawn, run loop, cleanup
# ==============================================================================

def send_dashboard_telemetry(vehicle_id, speed_kmh, adas_data):
    """Optionally stream telemetry to local web dashboard if active."""
    import json
    import urllib.request
    try:
        dist = adas_data.get("distance_to_lead_m", 0.0)
        ttc = adas_data.get("ttc_s", 0.0)
        payload = json.dumps({
            "id": str(vehicle_id),
            "source": "carla_simulation",
            "speed_kmh": float(speed_kmh),
            "distance_m": float(dist) if dist < 1e5 else 999.0,
            "ttc_s": float(ttc) if ttc < 1e5 else 99.0,
            "adas_status": str(adas_data.get("adas_status", "SAFE")),
            "lane_status": str(adas_data.get("lane_status", "OK")),
            "emergency_brake": bool(adas_data.get("emergency_brake_active", False)),
            "paper_state": str(adas_data.get("paper_state", "SAFE")),
            "paper_brake_cmd": float(adas_data.get("paper_brake_cmd", 0.0)),
            "lane_keeping_active": bool(adas_data.get("lane_keeping_active", False)),
            "traffic_light_red": bool(adas_data.get("traffic_light_red", False)),
            "pedestrian_in_path": bool(adas_data.get("pedestrian_in_path", False)),
        }).encode("utf-8")
        req = urllib.request.Request(
            "http://localhost:5000/api/telemetry",
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST"
        )
        with urllib.request.urlopen(req, timeout=0.05):
            pass
    except Exception:
        pass


def load_map(client):
    """Load Town05 map or return current world if already Town05 / load fails."""
    try:
        world = client.get_world()
        current_map = world.get_map().name
        if "Town05" in current_map:
            print("Town05 is already loaded.", flush=True)
            return world
        print("Loading Town05 map...", flush=True)
        world = client.load_world("Town05")
        print("Loaded Town05 successfully!", flush=True)
        return world
    except Exception as e:
        print("Failed to load Town05, falling back to current world: %s" % e, flush=True)
        try:
            return client.get_world()
        except Exception:
            return None


def run_demo():
    # Help the pygame window appear centered and visible on Windows (not stuck behind other apps).
    os.environ.setdefault("SDL_VIDEO_CENTERED", "1")
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass

    print("Starting pygame HUD window...", flush=True)
    pygame.init()
    display = pygame.display.set_mode(
        (DISPLAY_WIDTH, DISPLAY_HEIGHT),
        pygame.HWSURFACE | pygame.DOUBLEBUF,
    )
    pygame.display.set_caption("CARLA ADAS Demo - FCW / ACC / Lane")
    pygame.display.flip()
    pygame.event.pump()
    clock = pygame.time.Clock()

    client = None
    world = None
    ego_manager = None
    traffic_mgr = None
    sensor_mgr = None
    adas = None
    hud = None
    original_settings = None

    try:
        print("Connecting to CARLA at %s:%s ..." % (CARLA_HOST, CARLA_PORT), flush=True)
        client = carla.Client(CARLA_HOST, CARLA_PORT)
        client.set_timeout(10.0)
        world = load_map(client)
        if world is None:
            print("Error: Could not connect to CARLA world.", flush=True)
            return
        original_settings = world.get_settings()
        world.apply_settings(
            carla.WorldSettings(
                synchronous_mode=True,
                fixed_delta_seconds=FIXED_DELTA_SECONDS,
                no_rendering_mode=False,
                max_substeps=2,
            )
        )

        blueprint_library = world.get_blueprint_library()
        map_instance = world.get_map()

        # Ego
        ego_manager = EgoVehicleManager(world, blueprint_library, map_instance)
        if not ego_manager.spawn():
            print("Failed to spawn ego vehicle.")
            return
        ego_actor = ego_manager.get_actor()
        print("Ego vehicle spawned: %s" % ego_actor.type_id)
        # In synchronous mode, new actors must be ticked before attach_to sensors works reliably.
        for _ in range(15):
            world.tick()

        # Traffic + pedestrians
        traffic_mgr = TrafficManager(client, world, blueprint_library, map_instance, ego_actor)
        traffic_mgr.configure_traffic_manager()
        traffic_mgr.spawn_traffic_vehicles()
        traffic_mgr.spawn_pedestrians()
        print("Traffic and pedestrians spawned (traffic is moving with autopilot).")
        for _ in range(10):
            world.tick()
        refreshed = world.get_actor(ego_actor.id)
        if refreshed is not None and refreshed.is_alive:
            ego_actor = refreshed
            ego_manager.actor = ego_actor

        # Sensors
        sensor_mgr = SensorManager(world, ego_actor)
        sensor_mgr.setup_camera(mouse_controlled=True)  # Free camera with mouse drag to look around
        lane_ok = False
        radar_ok = False
        try:
            lane_ok = bool(sensor_mgr.setup_lane_invasion())
        except Exception as e:
            print("Lane invasion sensor unavailable (continuing without it): %s" % e, flush=True)
        try:
            radar_ok = bool(sensor_mgr.setup_ultrasonic_radar(num_sensors=5))
        except Exception as e:
            print("Radar sensors unavailable (continuing without them): %s" % e, flush=True)
        print(
            "Sensors: lane_invasion=%s radar=%s"
            % ("on" if lane_ok else "off", "on" if radar_ok else "off"),
            flush=True,
        )

        # ADAS and HUD
        adas = ADASController(world, map_instance, ego_actor)
        hud = HUD(DISPLAY_WIDTH, DISPLAY_HEIGHT)
        
        # Initialize CSV logging
        data_logger = DataLogger()
        data_logger.initialize_logging()

        # Manual control state (matching manual_control.py)
        throttle = 0.0
        brake = 0.0
        steer = 0.0
        handbrake = False
        steer_cache = 0.0
        STEER_SPEED = 0.003
        reverse_gear = False  # Q key toggles reverse
        gear = 1  # 1 = forward, -1 = reverse

        running = True
        while running:
            delta_time_ms = clock.get_time()
            delta_time_s = delta_time_ms / 1000.0

            # Events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN:
                    if event.key == K_ESCAPE:
                        running = False
                    if event.key == K_q:
                        # Toggle reverse gear (like manual_control.py)
                        if reverse_gear:
                            gear = 1  # Forward
                            reverse_gear = False
                        else:
                            gear = -1  # Reverse
                            reverse_gear = True
                        print("Gear: %s" % ("REVERSE" if reverse_gear else "FORWARD"))
                    if event.key == K_l:
                        # Toggle lane policy: ON = highway mode (strict lane following), OFF = city mode (free turns)
                        adas._lane_policy_enabled = not adas._lane_policy_enabled
                        print("Lane policy: %s (Highway mode)" % ("ON" if adas._lane_policy_enabled else "OFF (City mode)"))
                    if event.key == K_c:
                        # Toggle cruise control
                        enabled = adas.toggle_cruise_control()
                        print("Cruise control: %s (Target: %d km/h)" % ("ON" if enabled else "OFF", adas.cruise_target_speed_kmh))
                    if event.key == K_r:
                        # Toggle speed limit auto mode
                        auto_mode = adas.toggle_speed_limit_auto()
                        print("Speed limit auto: %s (Current limit: %d km/h)" % ("ON" if auto_mode else "OFF", adas.current_speed_limit_kmh))
                    if event.key == K_e:
                        # Toggle emergency brake override
                        override = adas.toggle_emergency_brake_override()
                        print("🚨 Emergency brake override: %s" % ("DISABLED" if override else "ENABLED"))
                    if event.key == K_UP:
                        # Increase cruise speed
                        adas.set_cruise_speed(adas.cruise_target_speed_kmh + 5)
                        print("Cruise speed set to: %d km/h" % adas.cruise_target_speed_kmh)
                    if event.key == K_DOWN:
                        # Decrease cruise speed
                        adas.set_cruise_speed(adas.cruise_target_speed_kmh - 5)
                        print("Cruise speed set to: %d km/h" % adas.cruise_target_speed_kmh)

            keys = pygame.key.get_pressed()
            if keys[K_w]:
                throttle = min(1.0, throttle + 0.05)
            else:
                throttle = max(0.0, throttle - 0.05)
            if keys[K_s]:
                brake = min(1.0, brake + 0.15)
            else:
                brake = max(0.0, brake - 0.1)
            if keys[K_a]:
                steer_cache -= STEER_SPEED * delta_time_ms
            elif keys[K_d]:
                steer_cache += STEER_SPEED * delta_time_ms
            else:
                steer_cache *= 0.9
            steer_cache = max(-0.7, min(0.7, steer_cache))
            steer = steer_cache
            handbrake = keys[K_SPACE]

            # ADAS: forward detection using radar sensors
            radar_detections = sensor_mgr.get_radar_detections()
            adas.update_forward_detection(radar_detections)
            adas.update_adas_status()
            adas.update_traffic_light()
            adas.update_pedestrian_crossing()
            adas.update_wrong_way()  # so wrong-way takes precedence over invasion
            adas.update_speed_limit()  # Detect and update speed limits
            adas.set_lane_invasion(
                sensor_mgr.get_lane_invasion_occurred(),
                sensor_mgr.get_crossed_solid(),
            )
            
            # Reset lane invasion warning when cleared
            adas.reset_lane_status_if_ok()
            
            # Paper-aligned components update
            adas.update_paper_aligned_components(radar_detections)

            # Update lateral warning for paper-aligned system (detect vehicles too close on sides)
            adas.update_lateral_warning()

            # Lane-keeping: auto steer back to lane when invasion or wrong-way; get turn direction for indicator
            lane_steer, lane_keeping_active, lane_keep_turn_direction = adas.get_lane_keeping_steer()
            if lane_keeping_active:
                steer = lane_steer  # Override driver steer to pull back to track (left or right as needed)
            # else keep steer from keys (steer_cache)

            # Current speed for distance-based speed limit
            v = ego_actor.get_velocity()
            speed_kmh = 3.6 * math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

            # Apply collision avoidance first (highest priority)
            throttle, brake, collision_active = adas.get_collision_avoidance_control(throttle, brake)
            
            # Apply cruise control if enabled and no collision avoidance
            if not collision_active:
                cruise_throttle, cruise_brake, cruise_active = adas.get_cruise_control(speed_kmh)
                if cruise_active:
                    throttle = cruise_throttle
                    brake = cruise_brake

            # Emergency braking control
            emergency_brake_stop = adas.emergency_reverse_active or (adas.adas_status in ("CRITICAL", "RED LIGHT", "PEDESTRIAN"))
            
            if emergency_brake_stop:
                control = carla.VehicleControl(
                    throttle=0.0,
                    steer=steer,
                    brake=1.0,
                    hand_brake=True,
                    reverse=False,
                    gear=1,
                )
            else:
                control = carla.VehicleControl(
                    throttle=throttle,
                    steer=steer,
                    brake=brake,
                    hand_brake=handbrake,
                    reverse=reverse_gear,
                    gear=-1 if reverse_gear else 1,
                )
            
            # Debug: Print control values
            if brake > 0.5:
                print(f"APPLYING CONTROL: Throttle={throttle:.2f}, Brake={brake:.2f}", flush=True)
            
            ego_actor.apply_control(control)

            # One-time console warning for lane violation (HUD shows top-right warning until back in lane)
            if adas.lane_status != "OK":
                if adas.lane_status != adas._last_lane_status:
                    print("Lane violation: %s" % adas.lane_status)
                    adas._last_lane_status = adas.lane_status
            else:
                adas._last_lane_status = "OK"

            # Mouse for camera: get relative movement (drag to look around)
            mouse_dx, mouse_dy = 0, 0
            if pygame.mouse.get_pressed()[0]:  # Left button held
                mouse_dx, mouse_dy = pygame.mouse.get_rel()
            else:
                pygame.mouse.get_rel()  # Reset rel to avoid jump on next drag

            # Update camera transform (free camera with mouse control)
            sensor_mgr.update_camera_transform(ego_actor.get_transform(), mouse_dx, mouse_dy)

            # Tick world and sensors
            world.tick()
            sensor_mgr.tick()
            adas.reset_lane_status_if_ok()

            # Camera image
            img = sensor_mgr.get_latest_image()
            if img is not None:
                array = np.frombuffer(img.raw_data, dtype=np.uint8)
                array = array.reshape((img.height, img.width, 4))[:, :, :3][:, :, ::-1]
                surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
                display.blit(surface, (0, 0))
            else:
                display.fill((30, 30, 30))

            # HUD
            v = ego_actor.get_velocity()
            speed_kmh_hud = 3.6 * math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
            adas_data = adas.get_display_data(lane_keeping_active=lane_keeping_active, lane_keep_turn_direction=lane_keep_turn_direction)
            hud.render(display, speed_kmh_hud, adas_data, reverse=reverse_gear)
            
            # Log data to CSV
            data_logger.log_data_point(
                timestamp=time.time(),
                distance=adas_data.get("distance_to_lead_m", 0.0),
                ego_speed_kmh=speed_kmh_hud,
                ttc=adas_data.get("ttc_s", 0.0),
                adas_status=adas_data.get("adas_status", "SAFE"),
                lane_status=adas_data.get("lane_status", "OK"),
                emergency_brake=adas_data.get("emergency_brake_active", False)
            )
            
            # Terminal display - Show ADAS readings every 10 frames
            if hasattr(data_logger, '_frame_count'):
                data_logger._frame_count += 1
            else:
                data_logger._frame_count = 1
                
            if data_logger._frame_count % 5 == 0:
                send_dashboard_telemetry(ego_actor.id, speed_kmh_hud, adas_data)

            if data_logger._frame_count % 10 == 0:
                dist = adas_data.get("distance_to_lead_m", float("inf"))
                ttc = adas_data.get("ttc_s", float("inf"))
                status = adas_data.get("adas_status", "SAFE")
                lane = adas_data.get("lane_status", "OK")
                emergency = adas_data.get("emergency_brake_active", False)
                
                dist_str = "%.1f m" % dist if dist < 1e5 else "--"
                ttc_str = "%.2f s" % ttc if ttc < 1e5 else "--"
                
                print(f"[ADAS] Speed: {speed_kmh_hud:.1f} km/h | Distance: {dist_str} | TTC: {ttc_str} | Status: {status} | Lane: {lane} | Emergency: {emergency}", flush=True)

            pygame.display.flip()
            clock.tick_busy_loop(int(1.0 / FIXED_DELTA_SECONDS))

    except Exception as e:
        print("Error: %s" % e)
        import traceback
        traceback.print_exc()
    finally:
        print("Shutting down and cleaning up...")
        # Close CSV logging
        if 'data_logger' in locals():
            data_logger.close_logging()
        
        # Close paper metrics logging
        if 'adas' in locals() and hasattr(adas, '_paper_metrics'):
            try:
                summary = adas._paper_metrics.summary()
                print("Paper metrics summary:")
                for k, v in summary.items():
                    print(f"  {k}: {v}")
                adas._paper_metrics.close()
            except Exception as e:
                print(f"Warning: Could not close paper metrics: {e}")
        
        # Restore world settings immediately when shutdown begins.
        if world and original_settings is not None:
            try:
                world.apply_settings(original_settings)
            except Exception:
                pass
        if sensor_mgr:
            sensor_mgr.destroy()
        if traffic_mgr:
            traffic_mgr.destroy_all()
        if ego_manager:
            ego_manager.destroy()
        if world and original_settings is not None:
            try:
                world.apply_settings(original_settings)
            except Exception:
                pass
        pygame.quit()
        print("Done.")


if __name__ == "__main__":
    try:
        run_demo()
    except KeyboardInterrupt:
        print("\nCancelled by user.")
    finally:
        pass
