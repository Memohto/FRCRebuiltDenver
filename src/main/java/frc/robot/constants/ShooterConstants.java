package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {

    // ── Flywheel ───────────────────────────────────────────────────────────────
    public static final int flywheelCanId = 30;

    public static final Slot0Configs flywheelGains =
        new Slot0Configs()
            .withKV(0.12)  // Feedforward: volts per RPS. This does ~95% of the work.
                           // If flywheel overshoots target speed → lower kV slightly.
                           // If flywheel falls short and never catches up → raise kV.
            .withKP(0.10)  // Proportional: trims steady-state error left by kV.
                           // Raise slowly if there's still a small gap at target speed.
                           // Lower if you hear oscillation/hunting at steady state.
            .withKI(0.0)
            .withKD(0.0);

    public static final double flywheelGearRatio = 1.0;

    // Default shooting speed used when Drive.mode == NORMAL.
    // Previously 0.42 duty cycle ≈ 42 RPS on a Kraken (100 RPS free speed).
    public static final double flywheelDefaultSpeedRPS = 42.0;

    // At-speed tolerance for isFlywheelAtSpeed().
    // ±2 RPS = ~120 RPM — tight enough for consistent shots, forgiving enough
    // that you're not waiting too long to feed. Tune tighter if shots vary.
    public static final double flywheelToleranceRadPerSec = Units.rotationsToRadians(2.0);

    public static final double flywheelStatorCurrentLimitAmps = 80;
    public static final double flywheelSupplyCurrentLimitAmps = 40;
    public static final boolean flywheelInverted = false;

    // ── Hood ───────────────────────────────────────────────────────────────────
    public static final int hoodCanId = 29;
    public static final Slot0Configs hoodGains =
        new Slot0Configs()
            .withKP(100.0)
            .withKI(0.0)
            .withKD(1.5)
            .withKS(0.4);
    public static final double hoodGearRatio = (48.0 / 12.0) * (30.0 / 15.0) * (178.0 / 10.0);
    public static final double hoodSpeed = 0.1;
    public static final double hoodStatorCurrentLimitAmps = 60;
    public static final double hoodSupplyCurrentLimitAmps = 30;
    public static final boolean hoodInverted = true;

    // Hood soft limits (unchanged)
    public static final double maxHoodAngleRad = Math.toRadians(0.5);
    public static final double minHoodAngleRad = Math.toRadians(17.0);

    // ── Shot Maps ──────────────────────────────────────────────────────────────

    // Distance (meters) → hood angle offset in degrees from starting position (unchanged)
    public static final InterpolatingDoubleTreeMap kShooterHoodMap = new InterpolatingDoubleTreeMap();
    static {
        kShooterHoodMap.put(1.0,  0.0);
        kShooterHoodMap.put(4.0, 16.5);
    }

    // Distance (meters) → flywheel speed in RPS (rotations per second).
    //
    // CONVERTED from the original 0.0–1.0 duty cycle values by multiplying by 100
    // (Kraken X60 free speed ≈ 100 RPS at 12V).
    //
    // These are starting points — re-tune on the real robot by:
    //   1. Commanding a distance and watching AdvantageScope "Shooter/FlywheelVelocityErrorRadPerSec"
    //   2. If shots are going short → increase the RPS for that distance
    //   3. If shots are going long  → decrease the RPS for that distance
    public static final InterpolatingDoubleTreeMap kShooterFlywheelMap = new InterpolatingDoubleTreeMap();
    static {
        kShooterFlywheelMap.put(1.0, 33.0);  // was 0.33
        kShooterFlywheelMap.put(2.0, 33.0);  // was 0.37
        kShooterFlywheelMap.put(3.0, 39.0);  // was 0.41
        kShooterFlywheelMap.put(4.0, 42.0);  // was 0.46
        kShooterFlywheelMap.put(5.0, 45.0);  // was 0.52
        // kShooterFlywheelMap.put(6.0, 100.0); // was 1.0 — uncomment when ready
    }
}
