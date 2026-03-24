package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;

public class IndexerConstants {
    // ── Rollers ────────────────────────────────────────────────────────────────
    public static final int rollersCanId = 22;
    public static final Slot0Configs rollersGains =
        new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKS(0).withKV(0.124);
    public static final double rollersGearRatio = 40/12;
    public static final double rollersSpeed = 0.4;
    public static final double rollersStatorCurrentLimitAmps = 60;
    public static final double rollersSupplyCurrentLimitAmps = 30;
    public static final boolean rollersInverted = false;

    // ── ShooterWheels ──────────────────────────────────────────────────────────
    public static final int shooterWheelsCanId = 23;
    public static final Slot0Configs shooterWheelsGains =
        new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKS(0).withKV(0.124);
    public static final double shooterWheelsGearRatio = 48/12;
    public static final double shooterWheelsSpeed = 0.4;
    public static final double shooterWheelsStatorCurrentLimitAmps = 60;
    public static final double shooterWheelsSupplyCurrentLimitAmps = 30;
    public static final boolean shooterWheelsInverted = false;

    // ── TurretWheels ───────────────────────────────────────────────────────────
    public static final int turretWheelsCanId = 24;
    public static final Slot0Configs turretWheelsGains =
        new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKS(0).withKV(0.124);
    public static final double turretWheelsGearRatio = 48/12;
    public static final double turretWheelsSpeed = 0.4;
    public static final double turretWheelsStatorCurrentLimitAmps = 60;
    public static final double turretWheelsSupplyCurrentLimitAmps = 30;
    public static final boolean turretWheelsInverted = true;

    // ── Feeder ─────────────────────────────────────────────────────────────────
    public static final int feederCanId = 25;
    public static final Slot0Configs feederGains =
        new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKS(0).withKV(0.124);
    public static final double feederGearRatio = 50/12;
    public static final double feederSpeed = 1;
    public static final double feederStatorCurrentLimitAmps = 60;
    public static final double feederSupplyCurrentLimitAmps = 30;
    public static final boolean feederInverted = true;
}