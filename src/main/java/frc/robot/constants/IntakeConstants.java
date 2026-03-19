package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.util.Units;

public class IntakeConstants {
    public static final double pinionRadiusMeters = Units.inchesToMeters(0.5);

    public static final int rollersCanId = 21;
    public static final Slot0Configs rollersGains =
        new Slot0Configs().withKP(0.1).withKI(0).withKD(0);
    public static final double rollersGearRatio = (20/12);
    public static final double rollersSpeed = 0.8;
    public static final double rollersStatorCurrentLimitAmps = 60;
    public static final double rollersSupplyCurrentLimitAmps = 30;
    public static final boolean rollersInverted = false;

    public static final int extensorCanId = 20;
    public static final Slot0Configs extensorGains =
        new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKS(0).withKV(0.124);
    public static final double extensorGearRatio = (54/12) * (36/18);
    public static final double extensorSpeed = 0.75;
    public static final double extensorStatorCurrentLimitAmps = 40;
    public static final double extensorSupplyCurrentLimitAmps = 20;
    public static final boolean extensorInverted = false;
}
