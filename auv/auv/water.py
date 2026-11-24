"""Fresh vs salt water: buoyancy, ballast, barometer spec-grav and DVL sound-speed corrections.

Sign convention (explicit): NET BUOYANCY is positive when the vehicle FLOATS (upward force).
The plant's `net_weight_N` is the opposite sign (positive sinks): net_weight_N = -net_buoyancy_N.

All functions are pure.  Vehicle: mass `mass_kg`, displaced volume `volume_L` (hull, with no ballast),
ballast is lead of `ballast_kg` carried externally (its own small displacement is included).
"""
from __future__ import annotations
from dataclasses import dataclass
import math

G = 9.80665
LEAD_DENSITY = 11340.0          # kg/m^3
ASSUMED_SOUND_SPEED = 1500.0    # m/s, DVL factory default


@dataclass(frozen=True)
class WaterType:
    name: str
    density: float              # kg/m^3
    sound_speed: float          # m/s
    baro_spec_grav: float       # ArduPilot BARO_SPEC_GRAV for this water

    @property
    def dvl_scale_factor_correction(self) -> float:
        """Multiply a DVL velocity computed with the assumed 1500 m/s by this to correct it.
        Doppler velocity is proportional to the assumed sound speed, so reading = true*1500/c."""
        return self.sound_speed / ASSUMED_SOUND_SPEED


FRESH = WaterType("fresh", 1000.0, 1482.0, 1.0)
SALT = WaterType("salt", 1025.0, 1521.0, 1.024)


def custom_water(density: float, sound_speed: float | None = None, name: str = "custom") -> WaterType:
    """Brackish / other water from density (kg/m^3).  Sound speed is interpolated between the
    fresh and salt presets when not given (linear in density, clamped extrapolation not applied)."""
    if sound_speed is None:
        f = (density - FRESH.density) / (SALT.density - FRESH.density)
        sound_speed = FRESH.sound_speed + f * (SALT.sound_speed - FRESH.sound_speed)
    return WaterType(name, density, sound_speed, density / 1000.0)


def get_water(w) -> WaterType:
    if isinstance(w, WaterType):
        return w
    return {"fresh": FRESH, "salt": SALT}[w]


def net_buoyancy_N(mass_kg: float, volume_L: float, ballast_kg: float, water) -> float:
    """Upward force (N): >0 floats, <0 sinks."""
    w = get_water(water)
    lead_vol = ballast_kg / LEAD_DENSITY                       # m^3
    return G * (w.density * (volume_L / 1000.0 + lead_vol) - mass_kg - ballast_kg)


def net_weight_N(mass_kg: float, volume_L: float, ballast_kg: float, water) -> float:
    """Plant convention: positive sinks (= -net_buoyancy_N)."""
    return -net_buoyancy_N(mass_kg, volume_L, ballast_kg, water)


def required_ballast_kg(mass_kg: float, volume_L: float, target_buoyancy_N: float, water) -> float:
    """Ballast (lead) mass giving the target net buoyancy (N, + floats).  Can be negative
    (= remove mass) when the dry vehicle is too heavy for the target."""
    w = get_water(water)
    b0 = G * (w.density * volume_L / 1000.0 - mass_kg)
    per_kg = G * (w.density / LEAD_DENSITY - 1.0)              # N per kg of lead (negative)
    return (target_buoyancy_N - b0) / per_kg


def depth_from_pressure(gauge_pa: float, spec_grav: float) -> float:
    """Depth (m) the autopilot reports: gauge pressure / (spec_grav * 1000 * g)."""
    return gauge_pa / (spec_grav * 1000.0 * G)


def pressure_at_depth(depth_m: float, water) -> float:
    return get_water(water).density * G * depth_m


def depth_reading_scale(true_water, spec_grav_setting: float) -> float:
    """Reported depth / true depth for a given BARO_SPEC_GRAV setting (1.0 = correct)."""
    return get_water(true_water).density / (spec_grav_setting * 1000.0)


def dvl_velocity_scale(true_water, sound_speed_setting: float = ASSUMED_SOUND_SPEED) -> float:
    """Reported / true DVL velocity when the DVL is configured with `sound_speed_setting`."""
    return sound_speed_setting / get_water(true_water).sound_speed


def dvl_velocity_correction(true_water, sound_speed_setting: float = ASSUMED_SOUND_SPEED) -> float:
    """Factor to multiply reported velocity by to remove the sound-speed error."""
    return 1.0 / dvl_velocity_scale(true_water, sound_speed_setting)


def predive_checklist(mass_kg: float, volume_L: float, water, target_buoyancy_N: float = 1.0) -> dict:
    w = get_water(water)
    bal = required_ballast_kg(mass_kg, volume_L, target_buoyancy_N, w)
    return {
        "water": w.name,
        "BARO_SPEC_GRAV": round(w.baro_spec_grav, 3),
        "ballast_kg": round(bal, 3),
        "target_net_buoyancy_N": target_buoyancy_N,
        "dvl_sound_speed_mps": round(w.sound_speed, 1),
        "dvl_uncorrected_error_pct": round((dvl_velocity_scale(w) - 1.0) * 100.0, 2),
    }
