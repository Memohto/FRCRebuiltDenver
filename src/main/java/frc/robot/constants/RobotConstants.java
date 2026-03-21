package frc.robot.constants;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.geometry.Translation2d;

public class RobotConstants {
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }
    public static enum RobotMode {
        BOMBER,
        STRIKER
    }
    public static enum DriveMode {
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

    public static final Translation2d  blueHub = new Translation2d(4.625, 4.035);
    public static final Translation2d  redHub = new Translation2d(11.917, 4.035);
}