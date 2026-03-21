package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {
    public static final double shootingSpeedRadPerSec = 100;

    // ── Flywheel ───────────────────────────────────────────────────────────────
    public static final int flywheelCanId = 30;
    public static final Slot0Configs flywheelGains =
        new Slot0Configs().withKP(0.1).withKI(0).withKD(0);
    public static final double flywheelGearRatio = 1;
    public static final double flywheelSpeed = 0.42;
    public static final double flywheelStatorCurrentLimitAmps = 80;
    public static final double flywheelSupplyCurrentLimitAmps = 40;
    public static final boolean flywheelInverted = false;

    // ── Hood ───────────────────────────────────────────────────────────────────
    //
    //  Gear train: (48/12) × (30/15) × (178/10) = 142.4 : 1
    //  Physical hood range: 17.5° (starting/min) to 34° (max elevation)
    //  Motor CW = hood goes up.  hoodInverted = true → positive command = CCW.
    //  So hood UP = negative mechanism position in Phoenix.
    //
    //  Motor position 0 = hood at 17.5° physical (starting position).
    //  Full travel = 16.5° of mechanism rotation = 0.04583 rotations.
    //
    //  WHY THE OLD LIMITS DIDN'T WORK:
    //  At 142.4:1, the motor needs to push through a LOT of gear friction.
    //  The old 10A supply / 20A stator limits capped the motor at ~15% of its
    //  available torque. The PID was computing the right voltage but the current
    //  limiter was clipping it before the motor could actually move.
    //
    public static final int hoodCanId = 29;
    public static final Slot0Configs hoodGains =
        new Slot0Configs()
            .withKP(100.0)   // [50 – 200] High because 142.4:1 → tiny mechanism rotations
            .withKI(0.0)
            .withKD(1.5)     // [0.5 – 3.0] Dampen oscillation
            .withKS(0.4);    // [0.1 – 0.8] Overcome static friction — raised from 0.15
    public static final double hoodGearRatio = (48.0 / 12.0) * (30.0 / 15.0) * (178.0 / 10.0); // = 142.4
    public static final double hoodSpeed = 0.1;
    public static final double hoodStatorCurrentLimitAmps = 60;   // was 20 — too low for 142.4:1
    public static final double hoodSupplyCurrentLimitAmps = 30;   // was 10 — too low for 142.4:1
    public static final boolean hoodInverted = false;

    // Hood soft limits (mechanism radians — negative = more elevation)
    public static final double maxHoodAngleRad = Math.toRadians(0.5);     // forward soft limit
    public static final double minHoodAngleRad = Math.toRadians(-17.0);   // reverse soft limit

    // ── Fixed shooter shot map ─────────────────────────────────────────────────
    public static final InterpolatingDoubleTreeMap kShotMap = new InterpolatingDoubleTreeMap();
    static {
        kShotMap.put(1.5, 0.0);    // close: hood at minimum (17.5° physical)
        kShotMap.put(6.0, 16.5);   // far:   hood at maximum (34° physical)
    }

    public static final double hoodToleranceRotations = Units.degreesToRotations(2.0);
}