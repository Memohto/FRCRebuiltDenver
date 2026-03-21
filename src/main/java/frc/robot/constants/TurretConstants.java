package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;

public class TurretConstants {
    public static final int flywheelCanId = 28;
    public static final int hoodCanId = 27;
    public static final int rotationMotorCanId = 26;

    public static final Slot0Configs rotationMotorGains =
        new Slot0Configs().withKP(8.0).withKI(0).withKD(0.2);
    public static final double rotationMotorGearRatio = (50/12) * (82/10);
    public static final boolean rotationMotorInverted = false;

    public static final double minRotationRad = -Math.PI * 0.75; // -135 grados
    public static final double maxRotationRad = Math.PI * 0.75;  // +135 grados

    // MotionMagic
    public static final double maxVelocityRotPerSec = 2.0;        // rotaciones/seg
    public static final double maxAccelerationRotPerSecSec = 4.0; // rotaciones/seg²
}