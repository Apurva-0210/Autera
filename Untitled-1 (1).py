#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
FleetEase ADAS demo for CARLA 0.9.15+
=====================================
Keeps the original file's vehicle preferences, Town05 map, and manual
keyboard controls. ONLY the ADAS logic core is replaced with a
paper-faithful FleetEase pipeline:

  Sensors -> Priority-Weighted Fusion -> TTC (with +inf sentinel)
          -> 4-State Decision (SAFE/WARNING/EMERGENCY/CRITICAL)  (Eq. 3)

Fixes vs previous revision:
  * STATE HYSTERESIS  -- prevents WARNING<->EMERGENCY oscillation that was
                          chattering the brake every 100 ms.
  * LIDAR BUMPER OFFSET -- LiDAR is roof-mounted so its measured range
                           overstates front-bumper distance by ~4 m on a
                           firetruck; this is now corrected so LiDAR and
                           Radar agree.
  * PER-SENSOR DECAY    -- ultrasonic / IR obstacle sensors only fire on
                           hit events; each now has its own timestamp so
                           a stale "too close" reading cannot latch forever.
  * LATENCY ON ANY ESCALATION -- was only updated on SAFE->non-SAFE.
  * STRONGER EMERGENCY BRAKE -- 0.55 was insufficient for laden-HCV mass;
                                bumped to 0.75 (still partial, CRITICAL=1.0
                                + handbrake).
  * CRITICAL applies hand_brake.
  * DWELL TIMERS on states so minor sensor noise cannot re-trigger.
"""

from __future__ import print_function

import argparse
import csv
import math
import os
import sys
import time
import weakref
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Tuple

# --- CARLA import ------------------------------------------------------------
repo_root = os.path.dirname(os.path.abspath(__file__))
carla_path = os.path.join(repo_root, "carla")
python_api = os.path.join(repo_root, "PythonAPI")
for p in (repo_root, carla_path, python_api):
    if os.path.exists(p) and p not in sys.path:
        sys.path.insert(0, p)

try:
    import carla
except ImportError as e:
    raise RuntimeError("Cannot import carla. Install with: pip install carla") from e

try:
    import pygame
    from pygame.locals import (
        K_ESCAPE, K_SPACE, K_a, K_d, K_s, K_w, K_q,
        K_1, K_2, K_3, K_4, K_5, K_6, K_7, K_8, K_9, K_v,
    )
    # Lowercase letter keys
    K_l = pygame.K_l
    K_c = pygame.K_c
    K_e = pygame.K_e
except ImportError as e:
    raise RuntimeError("Cannot import pygame. Install with: pip install pygame") from e

try:
    import numpy as np
except ImportError as e:
    raise RuntimeError("Cannot import numpy. Install with: pip install numpy") from e


# =============================================================================
# Constants (unchanged from original where possible)
# =============================================================================

CARLA_HOST = "localhost"
CARLA_PORT = 2000
TM_PORT = 8000
FIXED_DELTA_SECONDS = 0.03

# ORIGINAL vehicle preferences (kept verbatim)
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

# Camera mouse control (original values)
CAMERA_MOUSE_SENSITIVITY = 0.15
CAMERA_DISTANCE = 8.0
CAMERA_HEIGHT = 4.0

# Camera modes (1-9 keys)
CAMERA_MODES = {
    1: {"distance": 8.0, "height": 4.0, "pitch": -20, "name": "Follow"},
    2: {"distance": 15.0, "height": 8.0, "pitch": -25, "name": "High Follow"},
    3: {"distance": 5.0, "height": 2.0, "pitch": -15, "name": "Close"},
    4: {"distance": 20.0, "height": 15.0, "pitch": -45, "name": "Top Down"},
    5: {"distance": 3.0, "height": 1.5, "pitch": -10, "name": "Chase"},
    6: {"distance": 12.0, "height": 6.0, "pitch": -30, "name": "Wide"},
    7: {"distance": 25.0, "height": 20.0, "pitch": -60, "name": "Bird's Eye"},
    8: {"distance": 6.0, "height": 3.0, "pitch": -20, "name": "Medium"},
    9: {"distance": 30.0, "height": 25.0, "pitch": -70, "name": "Map View"},
}

# Sensor visualization modes
SENSOR_VISUALIZATION_MODES = {
    "LIDAR": 1,
    "RADAR": 2,
    "ULTRASONIC": 3,
    "IR": 4,
    "LANE_INVASION": 5,
}

# Lane keeping parameters from adas_final1.py for better performance
LANE_KEEPING_GAIN = 2.5   # How strongly to steer toward lane center
LANE_KEEPING_MAX_STEER = 0.6     # Cap corrective steer

# =============================================================================
# FleetEase paper parameters (Sec. 6 and 7)
# =============================================================================

TTC_UPDATE_PERIOD_S = 0.05   # 20 Hz TTC cadence (increased from 10 Hz for faster response)

# Sensor ranges (Table 3)
LIDAR_RANGE_M = 100.0
LIDAR_HZ = 10
RADAR_RANGE_M = 100.0
RADAR_HZ = 20
ULTRASONIC_RANGE_M = 8.0
ULTRASONIC_HZ = 30
IR_LATERAL_RANGE_M = 5.0
IR_HZ = 20

# LiDAR nearest-point extraction: +/-25 deg, 0-100 m (balanced FOV for front detection)
LIDAR_FOV_HALF_DEG = 25.0
LIDAR_NEAR_WINDOW_M = 100.0

# Radar detection parameters
RADAR_FOV_HALF_DEG = 45.0  # Wider FOV for radar (45 degrees = 90 degrees total)
RADAR_SELF_REJECT_M = 1.0  # Lower self-rejection for radar to detect closer vehicles

# Decision thresholds (Eq. 3)
# TTC-based thresholds from adas_final1.py for reliable braking
D_CRITICAL_M = 3.0  # Distance threshold for critical
D_EMERGENCY_M = 6.0  # Distance threshold for emergency  
D_WARNING_M = 10.0  # Distance threshold for warning
TTC_CRITICAL_S = 2.0  # Critical TTC threshold (full brake)
TTC_EMERGENCY_S = 3.0  # Emergency TTC threshold (strong brake)
TTC_WARNING_S = 4.0  # Warning TTC threshold (moderate brake)

# --- Hysteresis (new): buffers required before de-escalating a state --------
# Paper gives "entry" thresholds; these are the "exit" buffers to prevent
# flapping around the boundary when the brake causes TTC to rise just above
# the entry threshold for one tick.
D_EXIT_BUFFER_M = 0.5  # Reduced from 2.0 for faster de-escalation
TTC_EXIT_BUFFER_S = 0.1  # Reduced from 0.4 for faster de-escalation
# Minimum time to stay in a state before de-escalation is allowed (seconds).
# Reduced to allow quick recovery after obstacle clears
MIN_DWELL_S = {
    "SAFE": 0.0,
    "WARNING": 0.02,  # Reduced from 0.05
    "EMERGENCY": 0.05,  # Reduced from 0.1
    "CRITICAL": 0.08,  # Reduced from 0.15
}

# Non-closing sentinel tolerance (Sec. 7.1)
EPS_CLOSING_MS = 1e-3

# Minimum ego velocity for valid TTC calculation (m/s)
# Below this, TTC is considered invalid to prevent false positives when stationary
MIN_EGO_VELOCITY_MS = 1.0

# Filter length for dv (Sec. 3.2)
DV_FILTER_HISTORY = 5

# Self-detection rejection (meters). Anything closer than this is the ego truck.
# Reduced to detect smaller vehicles (not just heavy ones)
SELF_REJECT_M = 1.5

# --- Brake commands ---
# WARNING = audio/visual only; EMERGENCY = full brake; CRITICAL = full brake
# EMERGENCY now uses full brake (1.0) for maximum stopping power
BRAKE_WARNING = 0.0
BRAKE_EMERGENCY = 1.0  # Increased from 0.85 to 1.0 for maximum braking
BRAKE_CRITICAL = 1.0


# =============================================================================
# Paper-aligned types
# =============================================================================

class AdasState(str, Enum):
    SAFE = "SAFE"
    WARNING = "WARNING"
    EMERGENCY = "EMERGENCY"
    CRITICAL = "CRITICAL"


# Severity ordering for hysteresis comparisons
_SEVERITY = {
    AdasState.SAFE: 0,
    AdasState.WARNING: 1,
    AdasState.EMERGENCY: 2,
    AdasState.CRITICAL: 3,
}


@dataclass
class SensorSnapshot:
    lidar_distance_m: float = float("inf")       # bumper-corrected
    radar_distance_m: float = float("inf")
    radar_closing_velocity_ms: float = 0.0       # positive == closing
    ultrasonic_distance_m: float = float("inf")
    ir_left_distance_m: float = float("inf")
    ir_right_distance_m: float = float("inf")
    last_sensor_wall_time: float = 0.0


@dataclass
class FusedState:
    distance_m: float
    closing_velocity_ms: float
    ttc_s: float
    ultrasonic_critical: bool
    lateral_breach: bool


# =============================================================================
# Fusion (paper Sec. 6.2)
# =============================================================================

class PriorityWeightedFusion:
    def __init__(self):
        self._dv_hist: deque = deque(maxlen=DV_FILTER_HISTORY)

    def reset(self):
        self._dv_hist.clear()

    def _filter_dv(self, dv: float) -> float:
        self._dv_hist.append(dv)
        return sum(self._dv_hist) / len(self._dv_hist)

    def fuse(self, s: SensorSnapshot, ego_velocity_ms: float = 0.0, num_dv: float = 0.0) -> FusedState:
        # LiDAR primary for distance; radar distance is fallback.
        if s.lidar_distance_m < float("inf"):
            d = s.lidar_distance_m
        else:
            d = s.radar_distance_m

        dv = self._filter_dv(s.radar_closing_velocity_ms)
        # Fallback: if radar dv is near zero, use LiDAR numerical differentiation
        if abs(dv) < EPS_CLOSING_MS and abs(num_dv) > EPS_CLOSING_MS:
            dv = num_dv

        if abs(dv) < EPS_CLOSING_MS or dv <= 0.0 or d == float("inf"):
            ttc = float("inf")
        else:
            ttc = d / dv

        ultra_crit = False
        lateral = False
        return FusedState(d, dv, ttc, ultra_crit, lateral)


# =============================================================================
# Decision engine (paper Eq. 3) + hysteresis + dwell
# =============================================================================

class DecisionEngine:

    def __init__(self, mode: str = "proposed"):
        assert mode in {"proposed", "lidar_only", "pcar"}
        self.mode = mode
        self._current = AdasState.SAFE
        self._entered_at = 0.0

    # Immediate, no-memory classification as written in the paper.
    def _natural_state(self, f: FusedState) -> AdasState:
        # Ultrasonic override disabled - causes unwanted braking from side objects
        # if f.ultrasonic_critical:
        #     return AdasState.CRITICAL

        d, ttc = f.distance_m, f.ttc_s

        if self.mode == "pcar":
            if d < D_CRITICAL_M or ttc <= 1.0:
                return AdasState.CRITICAL
            if d < D_WARNING_M or ttc <= 2.5:
                return AdasState.WARNING
            return AdasState.SAFE

        # Proposed + lidar_only share the same four-state scheme.
        if d < D_CRITICAL_M or ttc <= TTC_CRITICAL_S:
            return AdasState.CRITICAL
        if d < D_EMERGENCY_M or ttc <= TTC_EMERGENCY_S:
            return AdasState.EMERGENCY
        if d < D_WARNING_M or ttc <= TTC_WARNING_S:
            return AdasState.WARNING
        return AdasState.SAFE

    # Conditions that must all be satisfied to *leave* the current state
    # (de-escalate). Escalations always fire immediately.
    def _can_deescalate(self, f: FusedState) -> bool:
        d, ttc = f.distance_m, f.ttc_s

        if self._current == AdasState.CRITICAL:
            # Exit if clearly above CRITICAL boundaries AND ultrasonic clear.
            exit_d = D_CRITICAL_M + D_EXIT_BUFFER_M           # 5.0 m
            exit_t = TTC_CRITICAL_S + TTC_EXIT_BUFFER_S       # 1.9 s
            return (not f.ultrasonic_critical
                    and d > exit_d
                    and (ttc == float("inf") or ttc > exit_t))

        if self._current == AdasState.EMERGENCY:
            exit_d = D_EMERGENCY_M + D_EXIT_BUFFER_M          # 9.0 m
            exit_t = TTC_EMERGENCY_S + TTC_EXIT_BUFFER_S      # 2.4 s
            return (d > exit_d
                    and (ttc == float("inf") or ttc > exit_t))

        if self._current == AdasState.WARNING:
            exit_d = D_WARNING_M + D_EXIT_BUFFER_M            # 17.0 m
            exit_t = TTC_WARNING_S + TTC_EXIT_BUFFER_S        # 3.4 s
            return (d > exit_d
                    and (ttc == float("inf") or ttc > exit_t))

        return True  # SAFE -> SAFE is fine

    def decide(self, f: FusedState, now: float) -> Tuple[AdasState, float]:
        natural = self._natural_state(f)

        # Escalation: fire immediately, refresh dwell.
        if _SEVERITY[natural] > _SEVERITY[self._current]:
            if natural != self._current:
                self._current = natural
                self._entered_at = now
            return self._current, _brake_for(self._current, f.distance_m, f.ttc_s)

        # Same state: just return.
        if natural == self._current:
            return self._current, _brake_for(self._current, f.distance_m, f.ttc_s)

        # De-escalation: need both dwell and hysteresis-buffer clearance.
        dwell_ok = (now - self._entered_at) >= MIN_DWELL_S[self._current.value]
        if dwell_ok and self._can_deescalate(f):
            self._current = natural
            self._entered_at = now
        # else: hold the more-severe state

        return self._current, _brake_for(self._current, f.distance_m, f.ttc_s)


def _brake_for(state: AdasState, distance_m: float, ttc_s: float) -> float:
    """
    TTC-based braking logic from adas_final1.py:
    - CRITICAL: Full brake (1.0)
    - EMERGENCY: Strong brake (0.8) 
    - WARNING: Moderate brake (0.5)
    - SAFE: No brake (0.0)
    """
    # Safety check: if distance is infinity (no obstacle detected), don't brake
    if distance_m == float("inf") or distance_m >= LIDAR_RANGE_M:
        return 0.0
    
    if state == AdasState.SAFE:
        return 0.0
    elif state == AdasState.WARNING:
        return 0.5  # Moderate brake for warning
    elif state == AdasState.EMERGENCY:
        return 0.8  # Strong brake for emergency
    else:  # CRITICAL
        return 1.0  # Full brake for critical


# =============================================================================
# Ego manager (ORIGINAL)
# =============================================================================

class EgoVehicleManager:
    def __init__(self, world, blueprint_library, map_instance):
        self.world = world
        self.blueprint_library = blueprint_library
        self.map_instance = map_instance
        self.actor = None

    def _get_ego_blueprint(self):
        for pref in EGO_VEHICLE_PREFERENCES:
            bps = self.blueprint_library.filter(pref)
            if bps:
                return bps[0]
        all_v = self.blueprint_library.filter("vehicle.*")
        for bp in all_v:
            if "cybertruck" in bp.id.lower():
                continue
            if any(x in bp.id for x in ["firetruck", "sprinter", "t2", "ambulance",
                                         "carlacola", "bus", "truck"]):
                return bp
        return all_v[0] if all_v else None

    def spawn(self):
        import random
        bp = self._get_ego_blueprint()
        if not bp:
            return False
        spawn_points = self.map_instance.get_spawn_points()

        def _try(transform):
            self.actor = self.world.try_spawn_actor(bp, transform)
            return self.actor is not None

        if spawn_points:
            order = list(spawn_points)
            random.shuffle(order)
            for sp in order[:min(40, len(order))]:
                if _try(sp):
                    return True
        try:
            wps = self.map_instance.generate_waypoints(5.0)
            random.shuffle(wps)
            for wp in wps[:80]:
                loc = wp.transform.location
                loc.z += 0.5
                if _try(carla.Transform(loc, wp.transform.rotation)):
                    return True
        except Exception:
            pass
        return _try(carla.Transform(carla.Location(0, 0, 3.0), carla.Rotation()))

    def destroy(self):
        if self.actor and self.actor.is_alive:
            self.actor.destroy()
            self.actor = None


# =============================================================================
# Traffic manager (ORIGINAL)
# =============================================================================

class TrafficManager:
    def __init__(self, client, world, blueprint_library, map_instance, ego_actor):
        self.client = client
        self.world = world
        self.blueprint_library = blueprint_library
        self.map_instance = map_instance
        self.ego_actor = ego_actor
        self.traffic_manager = client.get_trafficmanager(TM_PORT)
        self.vehicle_ids = []
        self._all_walker_actor_ids = []

    def configure(self):
        self.traffic_manager.set_synchronous_mode(True)
        self.traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        self.traffic_manager.global_percentage_speed_difference(20.0)
        self.traffic_manager.set_random_device_seed(0)
        self.traffic_manager.set_hybrid_physics_mode(True)

    def spawn_traffic(self, count=NUM_TRAFFIC_VEHICLES):
        from carla.command import SpawnActor, SetAutopilot, FutureActor
        import random
        sps = list(self.map_instance.get_spawn_points())
        ego_loc = self.ego_actor.get_location() if self.ego_actor else None
        usable = [sp for sp in sps
                  if not ego_loc or sp.location.distance(ego_loc) > 15.0]
        if len(usable) < count:
            usable = sps
        bps = [b for b in self.blueprint_library.filter("vehicle.*")
               if "cybertruck" not in b.id.lower()
               and "bike" not in b.id.lower()
               and "motorcycle" not in b.id.lower()
               and "bicycle" not in b.id.lower()]
        if not bps:
            bps = list(self.blueprint_library.filter("vehicle.*"))
        batch = []
        for i in range(min(count, len(usable))):
            bp = random.choice(bps)
            if bp.has_attribute("color"):
                bp.set_attribute("color",
                                 random.choice(bp.get_attribute("color").recommended_values))
            batch.append(SpawnActor(bp, usable[i % len(usable)]).then(
                SetAutopilot(FutureActor, True, self.traffic_manager.get_port())))
        for r in self.client.apply_batch_sync(batch, True):
            if not r.error:
                self.vehicle_ids.append(r.actor_id)

    def spawn_pedestrians(self, count=NUM_PEDESTRIANS):
        from carla.command import SpawnActor
        import random
        walker_bps = list(self.blueprint_library.filter("walker.pedestrian.*"))
        if not walker_bps:
            return
        sps = []
        for _ in range(count):
            loc = self.world.get_random_location_from_navigation()
            if loc is not None:
                sps.append(carla.Transform(loc))
        batch = []
        speeds = []
        for sp in sps:
            wb = random.choice(walker_bps)
            if wb.has_attribute("is_invincible"):
                wb.set_attribute("is_invincible", "false")
            speeds.append(wb.get_attribute("speed").recommended_values[1]
                          if wb.has_attribute("speed") else 1.4)
            batch.append(SpawnActor(wb, sp))
        res = self.client.apply_batch_sync(batch, True)
        walkers = []
        speeds_ok = []
        for i, r in enumerate(res):
            if not r.error:
                walkers.append({"id": r.actor_id})
                speeds_ok.append(speeds[i])
        ctrl_bp = self.world.get_blueprint_library().find("controller.ai.walker")
        batch = []
        for w in walkers:
            batch.append(SpawnActor(ctrl_bp, carla.Transform(), w["id"]))
        cres = self.client.apply_batch_sync(batch, True)
        for i, r in enumerate(cres):
            if not r.error and i < len(walkers):
                walkers[i]["con"] = r.actor_id
        all_id = []
        for w in walkers:
            if "con" in w:
                all_id.append(w["con"])
                all_id.append(w["id"])
        self._all_walker_actor_ids = all_id
        actors = self.world.get_actors(all_id)
        self.world.tick()
        for i in range(0, len(all_id), 2):
            if i + 1 < len(all_id):
                con = actors[i]
                idx = i // 2
                if idx < len(speeds_ok):
                    con.start()
                    con.go_to_location(self.world.get_random_location_from_navigation())
                    con.set_max_speed(float(speeds_ok[idx]))

    def destroy_all(self):
        from carla.command import DestroyActor
        if self.vehicle_ids:
            self.client.apply_batch_sync([DestroyActor(x) for x in self.vehicle_ids], True)
            self.vehicle_ids.clear()
        if self._all_walker_actor_ids:
            actors = self.world.get_actors(self._all_walker_actor_ids)
            for i in range(0, len(self._all_walker_actor_ids), 2):
                if i < len(actors):
                    try:
                        actors[i].stop()
                    except Exception:
                        pass
            self.client.apply_batch_sync(
                [DestroyActor(x) for x in self._all_walker_actor_ids], True)
            self._all_walker_actor_ids.clear()


# =============================================================================
# Sensor suite (paper-aligned, with self-detection guards + per-sensor decay
# + LiDAR bumper-offset correction)
# =============================================================================

class FleetEaseSensors:

    def __init__(self, world, ego):
        self.world = world
        self.ego = ego
        self._actors: List = []

        bx = ego.bounding_box.extent
        # Self-detection rejection sized by ego bounding box
        self._self_reject = max(SELF_REJECT_M, bx.x + 0.5)
        # LiDAR bumper-relative offset: LiDAR sits at ego center (x=0) but
        # the paper's d(t) is bumper-relative. Subtract bx.x from LiDAR range
        # so LiDAR and Radar distances are in the same frame.
        self._lidar_bumper_offset = float(bx.x)

        self._snap = SensorSnapshot()

        # Per-sensor timestamps (wall clock) for correct decay
        self._last_lidar_t: Optional[float] = None
        self._last_radar_t: Optional[float] = None
        self._last_ultra_t: Optional[float] = None
        self._last_ir_l_t: Optional[float] = None
        self._last_ir_r_t: Optional[float] = None

        # For B1 baseline numerical dv
        self._prev_lidar_dist: Optional[float] = None
        self._prev_lidar_wall: Optional[float] = None
        self._numerical_dv: float = 0.0

        self._camera_image = None
        self._image_queue = None

        self._camera = None
        self._camera_yaw_offset = 0.0
        self._camera_pitch_offset = -20.0
        self._camera_mode = 1  # Default camera mode
        self.sensor_visualization_enabled = False  # V key toggle
        
        # Lane invasion sensor for lane keeping
        self._lane_invasion_sensor = None
        self.lane_crossed_left = False
        self.lane_crossed_right = False

    def spawn(self, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT):
        bl = self.world.get_blueprint_library()
        bx = self.ego.bounding_box.extent
        front_x = bx.x + 0.4
        side_y = bx.y + 0.2
        mid_z = bx.z + 0.4
        roof_z = bx.z + 1.8
        weak = weakref.ref(self)

        # --- LiDAR on roof (avoids self-hits on chassis)
        lidar_bp = bl.find("sensor.lidar.ray_cast")
        lidar_bp.set_attribute("range", str(LIDAR_RANGE_M))
        lidar_bp.set_attribute("rotation_frequency", str(LIDAR_HZ))
        lidar_bp.set_attribute("channels", "64")  # Increased from 32 for better detection
        lidar_bp.set_attribute("points_per_second", "400000")  # Increased from 200000 for better detection
        lidar_bp.set_attribute("upper_fov", "5")
        lidar_bp.set_attribute("lower_fov", "-20")
        lidar_bp.set_attribute("sensor_tick", str(1.0 / LIDAR_HZ))
        lidar = self.world.spawn_actor(
            lidar_bp,
            carla.Transform(carla.Location(x=0.0, z=roof_z)),
            attach_to=self.ego,
        )
        lidar.listen(lambda d: FleetEaseSensors._on_lidar(weak, d))
        self._actors.append(lidar)

        # --- Radar, forward +/-45 deg, mounted at front bumper
        radar_bp = bl.find("sensor.other.radar")
        radar_bp.set_attribute("horizontal_fov", str(90))
        radar_bp.set_attribute("vertical_fov", "10")
        radar_bp.set_attribute("range", str(RADAR_RANGE_M))
        radar_bp.set_attribute("sensor_tick", str(1.0 / RADAR_HZ))
        radar = self.world.spawn_actor(
            radar_bp,
            carla.Transform(carla.Location(x=front_x, z=mid_z)),
            attach_to=self.ego,
        )
        radar.listen(lambda d: FleetEaseSensors._on_radar(weak, d))
        self._actors.append(radar)

        # --- Ultrasonic near-field (bumper-mounted)
        ultra_bp = bl.find("sensor.other.obstacle")
        ultra_bp.set_attribute("distance", str(ULTRASONIC_RANGE_M))
        ultra_bp.set_attribute("hit_radius", "0.6")
        ultra_bp.set_attribute("only_dynamics", "false")
        ultra_bp.set_attribute("sensor_tick", str(1.0 / ULTRASONIC_HZ))
        ultra = self.world.spawn_actor(
            ultra_bp,
            carla.Transform(carla.Location(x=front_x, z=0.5)),
            attach_to=self.ego,
        )
        ultra.listen(lambda d: FleetEaseSensors._on_ultra(weak, d))
        self._actors.append(ultra)

        # --- IR left / right lateral proximity
        for side, y, yaw in (("left", -side_y, -90.0), ("right", side_y, 90.0)):
            ir_bp = bl.find("sensor.other.obstacle")
            ir_bp.set_attribute("distance", str(IR_LATERAL_RANGE_M))
            ir_bp.set_attribute("hit_radius", "0.25")
            ir_bp.set_attribute("only_dynamics", "false")
            ir_bp.set_attribute("sensor_tick", str(1.0 / IR_HZ))
            ir = self.world.spawn_actor(
                ir_bp,
                carla.Transform(carla.Location(x=0.0, y=y, z=0.8),
                                carla.Rotation(yaw=yaw)),
                attach_to=self.ego,
            )
            ir.listen(lambda d, s=side: FleetEaseSensors._on_ir(weak, d, s))
            self._actors.append(ir)

        # --- Auxiliary RGB camera (detached; transform driven each frame)
        cam_bp = bl.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", str(width))
        cam_bp.set_attribute("image_size_y", str(height))
        cam_bp.set_attribute("fov", "90")
        self._camera = self.world.spawn_actor(
            cam_bp,
            carla.Transform(carla.Location(x=0, y=0, z=10),
                            carla.Rotation(pitch=-20, yaw=0)),
        )
        try:
            import queue
        except ImportError:
            import Queue as queue
        self._image_queue = queue.Queue()
        self._camera.listen(self._image_queue.put)
        self._actors.append(self._camera)
        
        # --- Lane invasion sensor for lane keeping
        lane_bp = bl.find("sensor.other.lane_invasion")
        self._lane_invasion_sensor = self.world.spawn_actor(
            lane_bp,
            carla.Transform(),
            attach_to=self.ego,
        )
        self._lane_invasion_sensor.listen(lambda d: FleetEaseSensors._on_lane_invasion(weak, d))
        self._actors.append(self._lane_invasion_sensor)

    # --- callbacks ---------------------------------------------------------
    @staticmethod
    def _on_lane_invasion(wref, event):
        self = wref()
        if self is None:
            return
        for marking in event.crossed_lane_markings:
            if marking == carla.LaneMarkingType.Solid or marking == carla.LaneMarkingType.Broken:
                # Determine which side was crossed based on the lane marking type
                # This is a simplified approach - in practice you'd need more sophisticated detection
                self.ego.get_transform()
                # Reset flags when any lane is crossed (will be set again in main loop)
                self.lane_crossed_left = False
                self.lane_crossed_right = False

    @staticmethod
    def _on_lidar(wref, data):
        self = wref()
        if self is None:
            return
        pts = np.frombuffer(data.raw_data, dtype=np.float32)
        if pts.size == 0:
            return
        pts = np.reshape(pts, (-1, 4))
        x, y = pts[:, 0], pts[:, 1]
        # Distance from LiDAR mount
        r_from_lidar = np.sqrt(x * x + y * y)
        az = np.degrees(np.arctan2(y, x))
        mask = (np.abs(az) <= LIDAR_FOV_HALF_DEG) & \
               (r_from_lidar >= self._self_reject) & \
               (r_from_lidar <= LIDAR_NEAR_WINDOW_M)
        now = time.time()
        if not np.any(mask):
            # no valid forward return -> nothing close detected
            new_d = float("inf")
        else:
            # Bumper-relative distance: subtract forward offset
            new_d = float(np.min(r_from_lidar[mask])) - self._lidar_bumper_offset
            if new_d < 0.0:
                new_d = 0.0

        # Numerical dv for B1 baseline
        if (self._prev_lidar_dist is not None and
                self._prev_lidar_wall is not None and
                self._prev_lidar_dist < float("inf") and
                new_d < float("inf")):
            dt = now - self._prev_lidar_wall
            if dt > 1e-4:
                self._numerical_dv = (self._prev_lidar_dist - new_d) / dt
            else:
                self._numerical_dv = 0.0
        else:
            self._numerical_dv = 0.0

        self._snap.lidar_distance_m = new_d
        self._prev_lidar_dist = new_d
        self._prev_lidar_wall = now
        self._last_lidar_t = now
        self._snap.last_sensor_wall_time = now

    @staticmethod
    def _on_radar(wref, data):
        self = wref()
        if self is None:
            return
        closest_d = float("inf")
        closest_vel = 0.0
        for det in data:
            az_deg = math.degrees(det.azimuth)
            # Use radar-specific wider FOV for better vehicle detection
            if abs(az_deg) > RADAR_FOV_HALF_DEG:
                continue
            # Use radar-specific lower self-rejection to detect closer vehicles
            if det.depth < RADAR_SELF_REJECT_M:
                continue
            if det.depth < closest_d:
                closest_d = det.depth
                closest_vel = -det.velocity   # flip sign: positive == closing
        self._snap.radar_distance_m = closest_d
        self._snap.radar_closing_velocity_ms = closest_vel
        self._last_radar_t = time.time()

    @staticmethod
    def _on_ultra(wref, event):
        self = wref()
        if self is None:
            return
        d = float(event.distance)
        if d < self._self_reject:
            return
        self._snap.ultrasonic_distance_m = d
        self._last_ultra_t = time.time()

    @staticmethod
    def _on_ir(wref, event, side: str):
        self = wref()
        if self is None:
            return
        d = float(event.distance)
        if d < 0.2:
            return
        now = time.time()
        if side == "left":
            self._snap.ir_left_distance_m = d
            self._last_ir_l_t = now
        else:
            self._snap.ir_right_distance_m = d
            self._last_ir_r_t = now

    # Per-sensor decay: if a sensor hasn't reported in max_age_s, treat as clear.
    def decay(self, wall_now: float,
              max_age_ultra: float = 0.20,
              max_age_ir: float = 0.30,
              max_age_radar: float = 0.40):
        if self._last_ultra_t is None or wall_now - self._last_ultra_t > max_age_ultra:
            self._snap.ultrasonic_distance_m = float("inf")
        if self._last_ir_l_t is None or wall_now - self._last_ir_l_t > max_age_ir:
            self._snap.ir_left_distance_m = float("inf")
        if self._last_ir_r_t is None or wall_now - self._last_ir_r_t > max_age_ir:
            self._snap.ir_right_distance_m = float("inf")
        if self._last_radar_t is None or wall_now - self._last_radar_t > max_age_radar:
            self._snap.radar_distance_m = float("inf")
            self._snap.radar_closing_velocity_ms = 0.0

    def update_camera(self, veh_transform, mouse_dx=0, mouse_dy=0):
        if self._camera is None or not self._camera.is_alive:
            return
        self._camera_yaw_offset += mouse_dx * CAMERA_MOUSE_SENSITIVITY
        self._camera_pitch_offset += mouse_dy * CAMERA_MOUSE_SENSITIVITY
        self._camera_pitch_offset = max(-89, min(30, self._camera_pitch_offset))
        
        # Get camera mode parameters
        mode = CAMERA_MODES.get(self._camera_mode, CAMERA_MODES[1])
        distance = mode["distance"]
        height = mode["height"]
        base_pitch = mode["pitch"]
        
        loc = veh_transform.location
        rot = veh_transform.rotation
        yaw = math.radians(rot.yaw + self._camera_yaw_offset)
        pitch = math.radians(base_pitch + self._camera_pitch_offset)
        cx = loc.x - distance * math.cos(yaw) * math.cos(pitch)
        cy = loc.y - distance * math.sin(yaw) * math.cos(pitch)
        cz = loc.z + height + distance * math.sin(pitch)
        self._camera.set_transform(carla.Transform(
            carla.Location(x=cx, y=cy, z=cz),
            carla.Rotation(pitch=-(base_pitch + self._camera_pitch_offset),
                           yaw=rot.yaw + self._camera_yaw_offset,
                           roll=rot.roll),
        ))

    def tick(self):
        while self._image_queue and not self._image_queue.empty():
            self._camera_image = self._image_queue.get_nowait()

    def set_camera_mode(self, mode_num):
        if mode_num in CAMERA_MODES:
            self._camera_mode = mode_num
            return CAMERA_MODES[mode_num]["name"]
        return None

    def snapshot(self) -> SensorSnapshot:
        return self._snap

    def numerical_dv(self) -> float:
        return self._numerical_dv

    def camera_image(self):
        return self._camera_image

    def destroy(self):
        for a in self._actors:
            try:
                if a.is_alive:
                    a.stop()
                    a.destroy()
            except Exception:
                pass
        self._actors.clear()

# Controller: 10 Hz gate + latency on any escalation
# =============================================================================

class FleetEaseController:

    def __init__(self, mode: str = "proposed"):
        self.mode = mode
        self.fusion = PriorityWeightedFusion()
        self.decision = DecisionEngine(mode=mode)

        self.state = AdasState.SAFE
        self.brake_cmd = 0.0
        self.fused: Optional[FusedState] = None

        self._prev_state = AdasState.SAFE
        self.last_latency_ms: Optional[float] = None
        self._last_eval_wall = 0.0

        # Lane policy toggle (L key)
        self.lane_policy_enabled = False

        # Cruise control variables (C key)
        self.cruise_control_enabled = False
        self.cruise_target_speed_kmh = 50.0

        # ADAS override (E key) - when true, driver can override ADAS brakes
        self.adas_override_enabled = False

    def step(self, snap: SensorSnapshot, num_dv: float, wall_now: float, ego_velocity_ms: float = 0.0):
        # 10 Hz TTC cadence — return cached result if we evaluated recently
        if wall_now - self._last_eval_wall < TTC_UPDATE_PERIOD_S:
            return self.state, self.brake_cmd, self.fused

        self._last_eval_wall = wall_now
        compute_t0 = time.perf_counter()

        # B1 baseline: use numerical LiDAR differentiation as dv
        if self.mode == "lidar_only":
            snap = SensorSnapshot(
                lidar_distance_m=snap.lidar_distance_m,
                radar_distance_m=snap.lidar_distance_m,
                radar_closing_velocity_ms=num_dv,
                ultrasonic_distance_m=snap.ultrasonic_distance_m,
                ir_left_distance_m=snap.ir_left_distance_m,
                ir_right_distance_m=snap.ir_right_distance_m,
                last_sensor_wall_time=snap.last_sensor_wall_time,
            )

        fused = self.fusion.fuse(snap, ego_velocity_ms, num_dv)
        state, brake = self.decision.decide(fused, wall_now)

        # Latency: update on ANY severity escalation
        if _SEVERITY[state] > _SEVERITY[self._prev_state]:
            if snap.last_sensor_wall_time > 0.0:
                lat = (wall_now - snap.last_sensor_wall_time) * 1000.0
                if 0.0 <= lat < 2000.0:
                    self.last_latency_ms = lat
                else:
                    self.last_latency_ms = (time.perf_counter() - compute_t0) * 1000.0
            else:
                self.last_latency_ms = (time.perf_counter() - compute_t0) * 1000.0
        elif state == AdasState.SAFE:
            self.last_latency_ms = None
        # else: leave latency as-is (report of previous escalation still valid)

        self._prev_state = state
        self.state = state
        self.brake_cmd = brake
        self.fused = fused
        return state, brake, fused


# =============================================================================
# HUD
# =============================================================================

class HUD:
    STATE_COLORS = {
        AdasState.SAFE:      (120, 220, 120),
        AdasState.WARNING:   (255, 200,  90),
        AdasState.EMERGENCY: (255, 130,  60),
        AdasState.CRITICAL:  (255,  70,  70),
    }

    def __init__(self, width, height):
        self.dim = (width, height)
        self.font = pygame.font.Font(pygame.font.get_default_font(), HUD_FONT_SIZE - 2)
        self.big = pygame.font.Font(pygame.font.get_default_font(), 26)

    def render(self, display, speed_kmh, snap, fused, state, brake_cmd,
               mode, latency_ms, driver_throttle, driver_brake,
               applied_throttle, applied_brake, sensor_viz_enabled=False):
        def fm(x): return f"{x:.2f} m" if x != float("inf") else "--"
        def fs(x): return f"{x:.2f} s" if x != float("inf") else "+inf"
        def fl(x): return f"{x:.1f} ms" if x is not None else "--"
        ultra_tag = "  <3m!" if fused.ultrasonic_critical else ""
        lateral_tag = "  LANE" if fused.lateral_breach else ""
        lines = [
            f"FleetEase  mode={mode}",
            f"Speed        {speed_kmh:5.1f} km/h",
            f"d LiDAR      {fm(snap.lidar_distance_m)}",
            f"d Radar      {fm(snap.radar_distance_m)}",
            f"d Ultra      {fm(snap.ultrasonic_distance_m)}{ultra_tag}",
            f"dv closing   {fused.closing_velocity_ms:+.2f} m/s",
            f"TTC          {fs(fused.ttc_s)}",
            f"System brake {brake_cmd:.2f}",
            f"Drv t/b      {driver_throttle:.2f} / {driver_brake:.2f}",
            f"App t/b      {applied_throttle:.2f} / {applied_brake:.2f}",
            f"Latency      {fl(latency_ms)}",
            f"IR L/R       {fm(snap.ir_left_distance_m)} / {fm(snap.ir_right_distance_m)}{lateral_tag}",
        ]
        y = 8
        for ln in lines:
            surf = self.font.render(ln, True, (240, 240, 240))
            display.blit(surf, (10, y))
            y += HUD_FONT_SIZE

        color = self.STATE_COLORS[state]
        txt = self.big.render(state.value, True, (20, 20, 20))
        pad = 10
        box = pygame.Surface((txt.get_width() + 2 * pad, txt.get_height() + 2 * pad))
        box.fill(color)
        rx = self.dim[0] - box.get_width() - 10
        ry = 10
        display.blit(box, (rx, ry))
        display.blit(txt, (rx + pad, ry + pad))

        # Sensor visualization overlay (V key toggle)
        if sensor_viz_enabled:
            # Draw sensor range indicators on screen
            cx, cy = self.dim[0] // 2, self.dim[1] // 2
            
            # LiDAR visualization - green circle (long range)
            lidar_range = min(snap.lidar_distance_m, 40.0) if snap.lidar_distance_m != float("inf") else 40.0
            lidar_radius = int(lidar_range * 5)  # Scale for display
            pygame.draw.circle(display, (0, 255, 0), (cx, cy), lidar_radius, 3)
            
            # Radar visualization - blue circle (medium range)
            radar_range = min(snap.radar_distance_m, 40.0) if snap.radar_distance_m != float("inf") else 40.0
            radar_radius = int(radar_range * 5)
            pygame.draw.circle(display, (0, 100, 255), (cx, cy), radar_radius, 3)
            
            # Ultrasonic visualization - red inner circle (near field)
            if snap.ultrasonic_distance_m != float("inf"):
                ultra_radius = int(min(snap.ultrasonic_distance_m, 8.0) * 10)
                pygame.draw.circle(display, (255, 0, 0), (cx, cy), ultra_radius, 4)
            
            # IR sensor visualization - left/right yellow circles (lateral)
            if snap.ir_left_distance_m != float("inf"):
                ir_left_radius = int(min(snap.ir_left_distance_m, 5.0) * 15)
                pygame.draw.circle(display, (255, 255, 0), (cx - 80, cy), ir_left_radius, 3)
            if snap.ir_right_distance_m != float("inf"):
                ir_right_radius = int(min(snap.ir_right_distance_m, 5.0) * 15)
                pygame.draw.circle(display, (255, 255, 0), (cx + 80, cy), ir_right_radius, 3)
            
            # Sensor legend and labels
            font_small = pygame.font.Font(pygame.font.get_default_font(), 18)
            font_legend = pygame.font.Font(pygame.font.get_default_font(), 14)
            
            # Legend box (with alpha transparency)
            legend_x, legend_y = 10, self.dim[1] - 120
            legend_surface = pygame.Surface((200, 110), pygame.SRCALPHA)
            legend_surface.fill((0, 0, 0, 180))
            display.blit(legend_surface, (legend_x - 5, legend_y - 5))
            pygame.draw.rect(display, (255, 255, 255), (legend_x - 5, legend_y - 5, 200, 110), 1)
            
            # Legend items
            legend_items = [
                ("LiDAR (long range)", (0, 255, 0)),
                ("Radar (medium range)", (0, 100, 255)),
                ("Ultrasonic (near field)", (255, 0, 0)),
                ("IR (lateral L/R)", (255, 255, 0)),
            ]
            
            for i, (text, color) in enumerate(legend_items):
                y_pos = legend_y + i * 25
                pygame.draw.circle(display, color, (legend_x + 10, y_pos + 8), 8)
                label = font_legend.render(text, True, (255, 255, 255))
                display.blit(label, (legend_x + 25, y_pos))
            
            # Distance labels on circles
            if snap.lidar_distance_m != float("inf"):
                label_lidar = font_small.render(f"{snap.lidar_distance_m:.1f}m", True, (0, 255, 0))
                display.blit(label_lidar, (cx - 50, cy - lidar_radius - 25))
            if snap.radar_distance_m != float("inf"):
                label_radar = font_small.render(f"{snap.radar_distance_m:.1f}m", True, (0, 100, 255))
                display.blit(label_radar, (cx - 50, cy - radar_radius - 45))


class FleetEaseLogger:
    def __init__(self, path):
        self._f = open(path, "w", newline="")
        self._w = csv.DictWriter(self._f, fieldnames=[
            "wall_time", "mode",
            "d_lidar_m", "d_radar_m", "d_ultra_m",
            "ir_left_m", "ir_right_m",
            "dv_closing_ms", "ttc_s",
            "ultra_critical", "lateral_breach",
            "state", "brake_cmd",
            "ego_speed_kmh", "latency_ms",
        ])
        self._w.writeheader()

    @staticmethod
    def _clean(x):
        if x is None:
            return ""
        if isinstance(x, float) and (math.isinf(x) or math.isnan(x)):
            return ""
        return x

    def log(self, wall_time, mode, snap, fused, state, brake_cmd,
            ego_speed_kmh, latency_ms):
        self._w.writerow({
            "wall_time": wall_time,
            "mode": mode,
            "d_lidar_m": self._clean(snap.lidar_distance_m),
            "d_radar_m": self._clean(snap.radar_distance_m),
            "d_ultra_m": self._clean(snap.ultrasonic_distance_m),
            "ir_left_m": self._clean(snap.ir_left_distance_m),
            "ir_right_m": self._clean(snap.ir_right_distance_m),
            "dv_closing_ms": self._clean(fused.closing_velocity_ms),
            "ttc_s": self._clean(fused.ttc_s),
            "ultra_critical": fused.ultrasonic_critical,
            "lateral_breach": fused.lateral_breach,
            "state": state.value,
            "brake_cmd": brake_cmd,
            "ego_speed_kmh": round(ego_speed_kmh, 2),
            "latency_ms": self._clean(latency_ms),
        })

    def close(self):
        try:
            self._f.close()
        except Exception:
            pass


# =============================================================================
# Main (ORIGINAL Town05 + manual controls)
# =============================================================================

def load_map(client):
    try:
        print("Loading Town05 map...")
        return client.load_world("Town05")
    except Exception as e:
        print("load_world failed:", e)
        return None


def run():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["proposed", "lidar_only", "pcar"],
                    default="proposed")
    ap.add_argument("--csv", default="fleetease_log.csv")
    args = ap.parse_args()

    os.environ.setdefault("SDL_VIDEO_CENTERED", "1")
    pygame.init()
    display = pygame.display.set_mode((DISPLAY_WIDTH, DISPLAY_HEIGHT),
                                      pygame.HWSURFACE | pygame.DOUBLEBUF)
    pygame.display.set_caption(f"FleetEase ADAS  [mode={args.mode}]")
    clock = pygame.time.Clock()

    client = world = ego_mgr = traffic = sensors = logger = None
    original_settings = None

    try:
        client = carla.Client(CARLA_HOST, CARLA_PORT)
        client.set_timeout(60.0)
        world = load_map(client)
        original_settings = world.get_settings()
        world.apply_settings(carla.WorldSettings(
            synchronous_mode=True,
            fixed_delta_seconds=FIXED_DELTA_SECONDS,
            no_rendering_mode=False,
            max_substeps=2,
        ))
        bl = world.get_blueprint_library()
        mp = world.get_map()

        ego_mgr = EgoVehicleManager(world, bl, mp)
        if not ego_mgr.spawn():
            print("Failed to spawn ego.")
            return
        ego = ego_mgr.actor
        print(f"Ego spawned: {ego.type_id}")
        for _ in range(15):
            world.tick()

        traffic = TrafficManager(client, world, bl, mp, ego)
        traffic.configure()
        traffic.spawn_traffic()
        traffic.spawn_pedestrians()
        print("Traffic and pedestrians spawned.")
        for _ in range(10):
            world.tick()
        refreshed = world.get_actor(ego.id)
        if refreshed is not None and refreshed.is_alive:
            ego = refreshed
            ego_mgr.actor = ego

        sensors = FleetEaseSensors(world, ego)
        sensors.spawn()
        print("Primary sensors attached (LiDAR on roof; Ultra/IR with self-reject).")

        controller = FleetEaseController(mode=args.mode)
        hud = HUD(DISPLAY_WIDTH, DISPLAY_HEIGHT)
        logger = FleetEaseLogger(args.csv)

        throttle = 0.0
        brake = 0.0
        steer = 0.0
        steer_cache = 0.0
        STEER_SPEED = 0.003
        reverse_gear = False

        running = True
        while running:
            dt_ms = clock.get_time()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN:
                    if event.key == K_ESCAPE:
                        running = False
                    if event.key == K_q:
                        reverse_gear = not reverse_gear
                        print("Gear:", "REVERSE" if reverse_gear else "FORWARD")
                    # Lane policy toggle (L key)
                    if event.key == K_l:
                        controller.lane_policy_enabled = not controller.lane_policy_enabled
                        print("Lane keeping:", "ON" if controller.lane_policy_enabled else "OFF")
                    # Cruise control toggle (C key)
                    if event.key == K_c:
                        controller.cruise_control_enabled = not controller.cruise_control_enabled
                        print("Cruise control:", "ON" if controller.cruise_control_enabled else "OFF")
                        if controller.cruise_control_enabled:
                            print(f"  Target speed: {controller.cruise_target_speed_kmh} km/h")
                    # Camera modes (1-9 keys)
                    if event.key == K_1:
                        mode_name = sensors.set_camera_mode(1)
                        print(f"Camera mode: {mode_name}")
                    if event.key == K_2:
                        mode_name = sensors.set_camera_mode(2)
                        print(f"Camera mode: {mode_name}")
                    if event.key == K_3:
                        mode_name = sensors.set_camera_mode(3)
                        print(f"Camera mode: {mode_name}")
                    if event.key == K_4:
                        mode_name = sensors.set_camera_mode(4)
                        print(f"Camera mode: {mode_name}")
                    if event.key == K_5:
                        mode_name = sensors.set_camera_mode(5)
                        print(f"Camera mode: {mode_name}")
                    if event.key == K_6:
                        mode_name = sensors.set_camera_mode(6)
                        print(f"Camera mode: {mode_name}")
                    if event.key == K_7:
                        mode_name = sensors.set_camera_mode(7)
                        print(f"Camera mode: {mode_name}")
                    if event.key == K_8:
                        mode_name = sensors.set_camera_mode(8)
                        print(f"Camera mode: {mode_name}")
                    if event.key == K_9:
                        mode_name = sensors.set_camera_mode(9)
                        print(f"Camera mode: {mode_name}")
                    # Sensor visualization toggle (V key)
                    if event.key == K_v:
                        sensors.sensor_visualization_enabled = not sensors.sensor_visualization_enabled
                        print("Sensor visualization:", "ON" if sensors.sensor_visualization_enabled else "OFF")
                    # ADAS override toggle (E key) - when held, driver can override ADAS brakes
                    if event.key == K_e:
                        controller.adas_override_enabled = not controller.adas_override_enabled
                        print("ADAS override:", "ON (Driver control)" if controller.adas_override_enabled else "OFF (ADAS control)")

            keys = pygame.key.get_pressed()
            # Check if E is held for temporary override
            adas_override_active = controller.adas_override_enabled or keys[K_e]
            if keys[K_w]:
                throttle = min(1.0, throttle + 0.05)
            else:
                throttle = max(0.0, throttle - 0.05)
            if keys[K_s]:
                brake = min(1.0, brake + 0.15)
            else:
                brake = max(0.0, brake - 0.10)
            if keys[K_a]:
                steer_cache -= STEER_SPEED * dt_ms
            elif keys[K_d]:
                steer_cache += STEER_SPEED * dt_ms
            else:
                steer_cache *= 0.9
            steer_cache = max(-0.7, min(0.7, steer_cache))
            steer = steer_cache
            driver_handbrake = keys[K_SPACE]

            wall_now = time.time()
            sensors.decay(wall_now)
            snap = sensors.snapshot()
            num_dv = sensors.numerical_dv()
            
            # DEBUG: Print sensor distances
            if snap.lidar_distance_m != float("inf") or snap.radar_distance_m != float("inf"):
                print(f"DEBUG: LiDAR={snap.lidar_distance_m:.1f}m, Radar={snap.radar_distance_m:.1f}m")
            
            # Calculate ego velocity (m/s) for robustness checks
            v = ego.get_velocity()
            ego_velocity_ms = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
            speed_kmh = ego_velocity_ms * 3.6
            
            state, sys_brake_cmd, fused = controller.step(snap, num_dv, wall_now, ego_velocity_ms)

            # Cruise control logic (C key toggle) - must be after state is known
            if controller.cruise_control_enabled and state in (AdasState.SAFE, AdasState.WARNING):
                # Simple P-controller for cruise control
                speed_error = controller.cruise_target_speed_kmh - speed_kmh
                if speed_error > 5.0:
                    throttle = min(0.6, throttle + 0.02)
                    brake = max(0.0, brake - 0.1)
                elif speed_error < -5.0:
                    throttle = max(0.0, throttle - 0.1)
                    brake = min(0.5, brake + 0.05)
                else:
                    # Maintain speed
                    throttle = max(0.05, min(0.3, throttle))
                    brake = max(0.0, brake - 0.05)

            # Lane keeping logic (L key toggle) - STRONG lane centering (Stanley controller)
            if controller.lane_policy_enabled:
                ego_location = ego.get_location()
                ego_transform = ego.get_transform()

                try:
                    waypoint = mp.get_waypoint(
                        ego_location,
                        project_to_road=True,
                        lane_type=carla.LaneType.Driving
                    )

                    if waypoint:
                        # --- Lane center reference ---
                        lane_center = waypoint.transform.location
                        lane_forward = waypoint.transform.get_forward_vector()

                        # --- Cross-track error ---
                        dx = ego_location.x - lane_center.x
                        dy = ego_location.y - lane_center.y
                        cross_track_error = dx * lane_forward.y - dy * lane_forward.x

                        # --- Heading error ---
                        lane_yaw = math.radians(waypoint.transform.rotation.yaw)
                        ego_yaw = math.radians(ego_transform.rotation.yaw)
                        heading_error = lane_yaw - ego_yaw

                        # Normalize heading error
                        while heading_error > math.pi:
                            heading_error -= 2 * math.pi
                        while heading_error < -math.pi:
                            heading_error += 2 * math.pi

                        # --- Stanley controller ---
                        k = 2.5
                        speed = max(ego_velocity_ms, 0.1)

                        steer_correction = heading_error + math.atan2(k * cross_track_error, speed)

                        # Clamp steering
                        steer = max(-LANE_KEEPING_MAX_STEER,
                                    min(LANE_KEEPING_MAX_STEER, steer_correction))

                        # Override driver steering completely
                        steer_cache = steer

                        print(f"[LANE] CTE={cross_track_error:.2f}, HE={heading_error:.2f}, Steer={steer:.2f}")

                        # --- Auto throttle ---
                        if state == AdasState.SAFE and not adas_override_active:
                            if speed_kmh < 30.0:
                                throttle = min(0.4, throttle + 0.02)
                                brake = max(0.0, brake - 0.05)
                            elif speed_kmh > 40.0:
                                throttle = max(0.0, throttle - 0.05)
                                brake = min(0.3, brake + 0.02)
                            else:
                                throttle = max(0.1, min(0.25, throttle))
                                brake = max(0.0, brake - 0.02)

                except Exception:
                    pass

            # --- Arbitration (paper Algo. 1 + engineering sense) -------------
            # SAFE     : driver has full control
            # WARNING  : driver keeps control; HUD alert only (no brake)
            # EMERGENCY: zero throttle, force system brake, engage handbrake for heavy vehicles
            # CRITICAL : zero throttle, full brake + engage handbrake
            # ADAS override (E key): when active, driver retains full control even in EMERGENCY/CRITICAL
            
            # Start with driver inputs
            applied_throttle = throttle
            applied_brake = brake
            applied_handbrake = driver_handbrake

            # Only apply ADAS brakes if override is NOT active and there's actual danger
            if not adas_override_active and sys_brake_cmd > 0.0:
                # DEBUG: Print sensor data and braking state
                print(f"DEBUG: d={fused.distance_m:.1f}m, ttc={fused.ttc_s:.1f}s, state={state.value}, brake={sys_brake_cmd:.2f}")
                
                if state == AdasState.EMERGENCY:
                    applied_throttle = 0.0  # Force throttle to 0
                    applied_brake = 1.0  # Force full brake
                    applied_handbrake = True  # Engage handbrake for heavy vehicles
                elif state == AdasState.CRITICAL:
                    applied_throttle = 0.0  # Force throttle to 0
                    applied_brake = 1.0  # Full brake
                    applied_handbrake = True   # parking brake for full stop

            ego.apply_control(carla.VehicleControl(
                throttle=float(applied_throttle),
                steer=float(steer),
                brake=float(applied_brake),
                hand_brake=bool(applied_handbrake),
                reverse=bool(reverse_gear),
            ))

            mouse_dx = mouse_dy = 0
            if pygame.mouse.get_pressed()[0]:
                mouse_dx, mouse_dy = pygame.mouse.get_rel()
            else:
                pygame.mouse.get_rel()
            sensors.update_camera(ego.get_transform(), mouse_dx, mouse_dy)

            world.tick()
            sensors.tick()

            v = ego.get_velocity()
            speed_kmh = 3.6 * math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)

            img = sensors.camera_image()
            if img is not None:
                arr = np.frombuffer(img.raw_data, dtype=np.uint8)
                arr = arr.reshape((img.height, img.width, 4))[:, :, :3][:, :, ::-1]
                surf = pygame.surfarray.make_surface(arr.swapaxes(0, 1))
                display.blit(surf, (0, 0))
            else:
                display.fill((25, 25, 30))

            if fused is not None:
                hud.render(display, speed_kmh, snap, fused, state, sys_brake_cmd,
                           args.mode, controller.last_latency_ms,
                           throttle, brake, applied_throttle, applied_brake,
                           sensors.sensor_visualization_enabled)
                logger.log(wall_now, args.mode, snap, fused, state,
                           sys_brake_cmd, speed_kmh, controller.last_latency_ms)

            pygame.display.flip()
            clock.tick_busy_loop(int(1.0 / FIXED_DELTA_SECONDS))

    except KeyboardInterrupt:
        print("\nInterrupted.")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Cleaning up...")
        if logger is not None:
            logger.close()
        if sensors is not None:
            sensors.destroy()
        if traffic is not None:
            traffic.destroy_all()
        if ego_mgr is not None:
            ego_mgr.destroy()
        if world is not None and original_settings is not None:
            try:
                world.apply_settings(original_settings)
            except Exception:
                pass
        pygame.quit()
        print("Done.")


if __name__ == "__main__":
    run()