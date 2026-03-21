package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class TurretConstants {

    // ── CAN IDs ────────────────────────────────────────────────────────────────
    public static final int flywheelCanId      = 28;
    public static final int hoodCanId          = 27;
    public static final int rotationMotorCanId = 26;

    // ── Gear ratio ─────────────────────────────────────────────────────────────
    // (50/12) * (82/10) — floating-point literals required (integer division = 32, wrong!)
    public static final double rotationMotorGearRatio = (50.0 / 12.0) * (82.0 / 10.0); // = 34.1667

    public static final boolean rotationMotorInverted = false;

    // ── PID / Feedforward (Slot0) ──────────────────────────────────────────────
    //
    //  All gains operate in MECHANISM rotations (after gear ratio).
    //  Phoenix 6 MotionMagic uses kV + kS for the feedforward profile,
    //  then kP + kD correct the remaining error.
    //
    //  ┌──────┬──────────────────────────────────────────┬───────┬───────┬─────────────┐
    //  │ Gain │ What it does                             │  Min  │  Max  │ Recommended  │
    //  ├──────┼──────────────────────────────────────────┼───────┼───────┼─────────────┤
    //  │ kP   │ Volts per rotation of position error.    │  5.0  │ 60.0  │   20.0       │
    //  │      │ Too low → sluggish. Too high → vibrates. │       │       │              │
    //  ├──────┼──────────────────────────────────────────┼───────┼───────┼─────────────┤
    //  │ kI   │ Volts per rot·sec of accumulated error.  │  0.0  │  0.5  │    0.0       │
    //  │      │ Usually 0 — MotionMagic handles steady   │       │       │              │
    //  │      │ state via feedforward. Only add if the    │       │       │              │
    //  │      │ turret consistently undershoots.          │       │       │              │
    //  ├──────┼──────────────────────────────────────────┼───────┼───────┼─────────────┤
    //  │ kD   │ Volts per rot/s of velocity error.       │  0.0  │  3.0  │    0.8       │
    //  │      │ Dampens overshoot/oscillation.            │       │       │              │
    //  ├──────┼──────────────────────────────────────────┼───────┼───────┼─────────────┤
    //  │ kS   │ Volts to overcome static friction.       │  0.1  │  0.8  │    0.3       │
    //  │      │ Increase until motor just barely twitches │       │       │              │
    //  │      │ when you command a tiny velocity.         │       │       │              │
    //  ├──────┼──────────────────────────────────────────┼───────┼───────┼─────────────┤
    //  │ kV   │ Volts per mechanism rot/s.               │  3.0  │  6.0  │    4.2       │
    //  │      │ = 12V / (motorFreeSpeed / gearRatio)     │       │       │              │
    //  │      │ Kraken X44: 5800 RPM / 34.17 = 2.83 r/s │       │       │              │
    //  │      │ kV ≈ 12 / 2.83 ≈ 4.24                   │       │       │              │
    //  │      │ THIS IS CRITICAL FOR MOTIONMAGIC SPEED.  │       │       │              │
    //  │      │ Old value 0.12 was ~35x too low!         │       │       │              │
    //  └──────┴──────────────────────────────────────────┴───────┴───────┴─────────────┘
    //
    //  TUNING ORDER: kV first (get speed right) → kS (overcome friction) → kP (snap to target) → kD (dampen)
    //
    public static final Slot0Configs rotationMotorGains =
        new Slot0Configs()
            .withKP(50.0)    // [5.0 – 60.0]  Start here, increase if sluggish, decrease if vibrating
            .withKI(0.1)     // [0.0 –  0.5]  Leave at 0 unless turret consistently undershoots
            .withKD(2.0)     // [0.0 –  3.0]  Increase if overshooting/oscillating
            .withKS(0.35)    // [0.1 –  0.8]  Increase until motor barely twitches from rest
            .withKV(4.2);    // [3.0 –  6.0]  ≈ 12V / (5800RPM / 60 / 34.17) — DO NOT go below 3.0

    // ── Motion Magic speed limiter ─────────────────────────────────────────────
    //
    //  These are in MECHANISM rotations/second (after gear ratio).
    //  The turret mechanism can physically reach ~2.83 rot/s (no load).
    //  Never command more than ~85% of free speed.
    //
    //  ┌────────────────┬───────┬───────┬─────────────┬───────────────────────────┐
    //  │ Constant       │  Min  │  Max  │ Recommended │ Notes                     │
    //  ├────────────────┼───────┼───────┼─────────────┼───────────────────────────┤
    //  │ Cruise Velocity│  0.3  │  2.4  │    1.5      │ 1.5 rot/s ≈ 540°/s       │
    //  │ (rot/s)        │       │       │             │ Fast snap, still safe     │
    //  ├────────────────┼───────┼───────┼─────────────┼───────────────────────────┤
    //  │ Acceleration   │  0.5  │  5.0  │    3.0      │ How fast it ramps up.     │
    //  │ (rot/s²)       │       │       │             │ Lower = smoother, higher  │
    //  │                │       │       │             │ = more responsive.        │
    //  └────────────────┴───────┴───────┴─────────────┴───────────────────────────┘
    //
    public static final double maxVelocityRotPerSec        = 2.0;   // [0.3 – 2.4] rot/s — cruise speed
    public static final double maxAccelerationRotPerSecSec = 5.0;   // [0.5 – 5.0] rot/s² — ramp rate

    // ── Soft limits ───────────────────────────────────────────────────────────
    //
    //  Measured from physical turret zero (the position at robot enable).
    //  CCW = positive, CW = negative (WPILib / Phoenix 6 standard).
    //  Total range = 243.6 + 116.4 = 360°, so every field direction is reachable.
    //
    //  IMPORTANT: These are enforced BOTH in Phoenix firmware (TalonFX soft limits)
    //  AND in the tracking algorithm (angle normalization). Both must agree.
    //
    public static final double maxRotationDeg =  243.6;  // CCW limit (positive)
    public static final double minRotationDeg = -116.4;  // CW  limit (negative)
    public static final double maxRotationRad = Math.toRadians(maxRotationDeg);
    public static final double minRotationRad = Math.toRadians(minRotationDeg);

    // ── Turret zero-angle offset ───────────────────────────────────────────────
    //
    //  Turret physical zero faces BACKWARD (toward driver station / shooter side).
    //  Robot forward is the intake side. Offset = π to bridge that 180° gap.
    //  If tracking is consistently off by a small fixed angle, tweak around Math.PI.
    //
    public static final double turretZeroOffsetRad = Math.PI;

    // ═══════════════════════════════════════════════════════════════════════════
    //  TARGET CONFIGURATION
    // ═══════════════════════════════════════════════════════════════════════════
    //
    //  Coordinates are relative to the robot's POSITION AT ENABLE TIME.
    //  Robot frame: +X = forward (intake side), +Y = left.
    //
    // ── Target A: HUB ─────────────────────────────────────────────────────────
    //  HUB is 1.184 m forward (toward intake) and 0.854 m to the RIGHT.
    //  In robot frame, right = negative Y.
    //
    public static final double hubOffsetX =  1.184;   // meters, robot +X (forward / intake side)
    public static final double hubOffsetY = -0.854;   // meters, robot +Y (left); negative = right

    // ── Target B: Driver Station ───────────────────────────────────────────────
    //  The turret at 0° points directly toward the driver station.
    //  Just set the straight-line distance from starting position to DS.
    //
    public static final double driverStationDistance = 3.43;  // meters

    // ═══════════════════════════════════════════════════════════════════════════
    //  TURRET SHOT MAP — Hood + Flywheel interpolation by distance
    // ═══════════════════════════════════════════════════════════════════════════
    //
    //  HOW IT WORKS:
    //  You provide two (or more) reference points: a distance and the hood angle
    //  + flywheel power that work well at that distance. InterpolatingDoubleTreeMap
    //  linearly interpolates between them for any distance in between.
    //  Outside the range, values clamp to the nearest endpoint.
    //
    //  TO ADD MORE POINTS: just add more .put() calls with measured data.
    //  The more points you add, the more accurate the interpolation becomes.
    //
    //  HOOD MAP: distance (meters) → hood offset (degrees from starting position)
    //    0° offset   = 17.5° physical hood angle (minimum, most flat)
    //    16.5° offset = 34° physical hood angle (maximum, most elevated)
    //
    //  FLYWHEEL MAP: distance (meters) → flywheel duty cycle (0.0 to 1.0)
    //    0.40 = 40% power,  0.50 = 50% power, etc.
    //
    //  ┌──────────┬──────────────────────┬───────────────────┐
    //  │ Distance │ Hood Offset (°)      │ Flywheel Power    │
    //  ├──────────┼──────────────────────┼───────────────────┤
    //  │  1.5 m   │   0.0° (17.5° phys) │  40%              │
    //  │  6.0 m   │  16.5° (34.0° phys) │  50%              │
    //  └──────────┴──────────────────────┴───────────────────┘
    //
    //  Example: at 3.75 m (midpoint), the turret will auto-set:
    //    hood     = 8.25° offset (25.75° physical)
    //    flywheel = 45% power
    //

    /** Distance (meters) → hood angle offset in degrees from starting position. */
    public static final InterpolatingDoubleTreeMap kTurretHoodMap = new InterpolatingDoubleTreeMap();
    static {
        kTurretHoodMap.put(1.5, 0.0);    // CLOSE: 17.5° physical, minimum elevation
        kTurretHoodMap.put(6.0, 16.5);   // FAR:   34.0° physical, maximum elevation
        // Add more measured points here as you tune:
        // kTurretHoodMap.put(3.0, 8.0);  // example mid-range point
    }

    /** Distance (meters) → flywheel duty cycle (0.0–1.0). */
    public static final InterpolatingDoubleTreeMap kTurretFlywheelMap = new InterpolatingDoubleTreeMap();
    static {
        kTurretFlywheelMap.put(1.5, 0.32);  // CLOSE: 40% power
        kTurretFlywheelMap.put(3.75, 0.41); // MID:   36% power — tune this down if still too hot
        kTurretFlywheelMap.put(6.0, 0.54);  // FAR:   50% power
        // Add more measured points here as you tune:
        // kTurretFlywheelMap.put(3.0, 0.44);  // example mid-range point
    }

}