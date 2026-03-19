package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {
    public static final double shootingSpeedRadPerSec = 100;

    public static final int flywheelCanId = 30;
    public static final Slot0Configs flywheelGains =
        new Slot0Configs().withKP(0.1).withKI(0).withKD(0);
    public static final double flywheelGearRatio = 1;
    public static final double flywheelSpeed = 0.42;
    public static final double flywheelStatorCurrentLimitAmps = 80;
    public static final double flywheelSupplyCurrentLimitAmps = 40;
    public static final boolean flywheelInverted = false;

     public static final int hoodCanId = 29;
    public static final Slot0Configs hoodGains =
        new Slot0Configs().withKP(0.1).withKI(0).withKD(0);
    public static final double hoodGearRatio = (48.0/12.0) * (30.0/15.0);
    public static final double hoodSpeed = 0.1;
    public static final double hoodStatorCurrentLimitAmps = 20;
    public static final double hoodSupplyCurrentLimitAmps = 10;
    public static final boolean hoodInverted = true;

    public static final double minHoodAngleRad = 0.05;
    public static final double maxHoodAngleRad = 5;

    public static final InterpolatingDoubleTreeMap kShotMap = new InterpolatingDoubleTreeMap();
    static {
        // Distance in meters, Angle in degrees
        kShotMap.put(1.0, minHoodAngleRad);
        kShotMap.put(5.0, maxHoodAngleRad);
    }
    public static final double hoodToleranceRotations = Units.degreesToRotations(2.0);
}
