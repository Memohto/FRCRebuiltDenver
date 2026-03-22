package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;

public class TurretConstants {
    // ── Flywheel ───────────────────────────────────────────────────────────────
    public static final int flywheelCanId      = 28;

    // ── Hood ───────────────────────────────────────────────────────────────────
    public static final int hoodCanId          = 27;

    // ── Rotation Motor ─────────────────────────────────────────────────────────
    public static final int rotationMotorCanId = 26;
    public static final double rotationMotorGearRatio = (50.0 / 12.0) * (82.0 / 10.0);

    public static final boolean rotationMotorInverted = false;
    public static final Slot0Configs rotationMotorGains =
        new Slot0Configs()
            .withKP(50.0)
            .withKI(0.1)
            .withKD(2.0)
            .withKS(0.35)
            .withKV(4.2);

    // Motion magic speed limiter
    public static final double maxVelocityRotPerSec = 2.0;   // [0.3 – 2.4] rot/s — cruise speed
    public static final double maxAccelerationRotPerSecSec = 5.0;   // [0.5 – 5.0] rot/s² — ramp rate

    // Soft limits
    public static final double maxRotationDeg =  243.6;
    public static final double minRotationDeg = -116.4;
    public static final double maxRotationRad = Math.toRadians(maxRotationDeg);
    public static final double minRotationRad = Math.toRadians(minRotationDeg);

    // Turret zero-angle offset
    public static final double turretZeroOffsetRad = Math.PI;
}