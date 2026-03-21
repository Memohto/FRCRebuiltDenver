package frc.robot.constants;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class RobotConstants {
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;
    public static final ChasisMode chasisMode = ChasisMode.NORMAL;
    public static final TurretMode turretMode = TurretMode.NORMAL;

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }
    public static enum ChasisMode {
        ORBIT,
        FEEDER,
        NORMAL
    }
    public static enum TurretMode {
        HUB_TRACKER,
        DS_TRACKER,
        NORMAL
    }

    public static final double robotLoopFrequencyHz = 50.0;
    public static final double highPriorityFrequencyHz = robotLoopFrequencyHz;
    public static final double lowPriorityFrequencyHz = robotLoopFrequencyHz / 2;

    public static final Pose2d blueHub = new Pose2d(4.625, 4.035, new Rotation2d());
    public static final Pose2d redHub = new Pose2d(11.917, 4.035, new Rotation2d());
}