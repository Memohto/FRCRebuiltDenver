package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterConstants {
    public static final double shootingSpeedRadPerSec = 100;

    // ── Flywheel ───────────────────────────────────────────────────────────────
    public static final int flywheelCanId = 30;
    public static final Slot0Configs flywheelGains =
        new Slot0Configs().withKP(0.1).withKI(0).withKD(0);
    public static final double flywheelGearRatio = 1;
    public static final double flywheelDefaultSpeed = 0.42;
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

    // Hood soft limits
    public static final double maxHoodAngleRad = Math.toRadians(0.5);
    public static final double minHoodAngleRad = Math.toRadians(17.0);

    // Distance (meters) → hood angle offset in degrees from starting position
    public static final InterpolatingDoubleTreeMap kShooterHoodMap = new InterpolatingDoubleTreeMap();
    static {
        kShooterHoodMap.put(1.0, 0.0);
        kShooterHoodMap.put(4.0, 16.5);
    }

    // Distance (meters) → flywheel speed (0.0–1.0)
    public static final InterpolatingDoubleTreeMap kShooterFlywheelMap = new InterpolatingDoubleTreeMap();
    static {
        kShooterFlywheelMap.put(1.0,0.33);
        kShooterFlywheelMap.put(2.0, 0.37);
        kShooterFlywheelMap.put(3.0, 0.41);
        kShooterFlywheelMap.put(4.0, 0.46);
        kShooterFlywheelMap.put(5.0, 0.52);
        //kShooterFlywheelMap.put(6.0, 1.0);
    }
}