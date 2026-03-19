package frc.robot.constants;

import edu.wpi.first.wpilibj.RobotBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class RobotConstants {
    // ################## General ##################### //
    public static final GameMode currentMode = RobotBase.isReal() ? GameMode.REAL : GameMode.SIM;
    public static final DriveMode currentDrive = DriveMode.SWERVE;
    public static final double robotLoopFrequencyHz = 50.0;

    public static final double highPriorityFrequencyHz = robotLoopFrequencyHz;
    public static final double lowPriorityFrequencyHz = robotLoopFrequencyHz / 2;

    // ################### Enums ###################### //
    public static enum GameMode {
        REAL,
        SIM
    }

    public static enum DriveMode {
        SWERVE,
        DIFFERENTIAL
    }

    public static final Pose2d blueInitialPose = new Pose2d(3, 4, Rotation2d.kZero);
    public static final Pose2d redInitialPose = new Pose2d(13.5, 4, Rotation2d.k180deg);

    public static final Pose2d blueOutpost = new Pose2d(4.625, 4.035, new Rotation2d());
    public static final Pose2d redOutpost = new Pose2d(11.917, 4.035, new Rotation2d());
}