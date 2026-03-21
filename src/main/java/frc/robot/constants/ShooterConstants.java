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
        // Distancia en metros → ángulo del hood en grados
        // Medir en robot real y ajustar estos valores
        kShotMap.put(1.0, minHoodAngleRad);
        kShotMap.put(2.0, 1.5);
        kShotMap.put(3.0, 2.5);
        kShotMap.put(4.0, 3.5);
        kShotMap.put(5.0, maxHoodAngleRad);
    }

    public static final InterpolatingDoubleTreeMap kFlywheelMap = new InterpolatingDoubleTreeMap();
    static {
        // Distancia en metros → velocidad flywheel (open loop 0.0-1.0)
        // Medir en robot real y ajustar estos valores
        kFlywheelMap.put(1.0, 0.35);
        kFlywheelMap.put(2.0, 0.40);
        kFlywheelMap.put(3.0, 0.45);
        kFlywheelMap.put(4.0, 0.50);
        kFlywheelMap.put(5.0, 0.55);
    }
    public static final double hoodToleranceRotations = Units.degreesToRotations(2.0);
}