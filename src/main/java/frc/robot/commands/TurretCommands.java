package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.constants.RobotConstants.DriveMode;
import frc.robot.constants.RobotConstants.RobotMode;
import frc.robot.constants.RobotConstants.TurretMode;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.Turret;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class TurretCommands {

  private TurretCommands() {
  }

  private static Translation2d getBomberTarget(Pose2d robotPose) {
    boolean isRedAlliance = DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;

    Translation2d target = new Translation2d();
    switch (Drive.mode) {
      case ORBIT:
        target = isRedAlliance ? RobotConstants.redHub : RobotConstants.blueHub;
        break;
      case FEEDER:
        target = isRedAlliance ? new Translation2d(14, robotPose.getY()) : new Translation2d(2, robotPose.getY());
        break;
      case NORMAL:
      default:
        break;
    }
    return target;
  }

  private static Translation2d getStrikerTarget(Pose2d robotPose) {
    boolean isRedAlliance = DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
    
    Translation2d target = new Translation2d();
    switch (Turret.mode) {
      case HUB_TRACKER:
        target = isRedAlliance ? RobotConstants.redHub : RobotConstants.blueHub;
        break;
      case DS_TRACKER:
        target = isRedAlliance ? new Translation2d(14, robotPose.getY()) : new Translation2d(2, robotPose.getY());
        break;
      case NORMAL:
      default:
        break;
    }
    return target;
  }

  public static Command joystickTurretCmd(
      Turret turret,
      BooleanSupplier shootSupplier,
      Supplier<Pose2d> robotPoseSupplier) {
    return Commands.run(() -> {
      if (shootSupplier.getAsBoolean()) {
        Pose2d robotPose = robotPoseSupplier.get();
        Translation2d target = new Translation2d();
        double distanceMeters = 0.0;
        double angleRad = 0.0;

        if (Robot.mode == RobotMode.BOMBER) {
          target = getBomberTarget(robotPose);
          angleRad = 0.0;
          distanceMeters = Drive.getDistanceToTargetMeters(robotPose, target);
        } else {
          target = getStrikerTarget(robotPose);
          angleRad = Turret.computeTurretAngleRad(robotPose, target);
          distanceMeters = Drive.getDistanceToTargetMeters(robotPose, target);
        }

        if (Drive.mode == DriveMode.NORMAL && Turret.mode == TurretMode.NORMAL) {
          turret.rotateToAngle(0.0);
          turret.setHoodAtInitialPosition();
          // FIX 1: renamed flywheelDefaultSpeed → flywheelDefaultSpeedRPS
          // FIX 2: map now returns RPS, so use setFlywheelVelocity instead of setFlywheelSpeed
          turret.setFlywheelVelocity(
              Units.rotationsToRadians(ShooterConstants.flywheelDefaultSpeedRPS));
        } else {
          turret.rotateToAngle(angleRad);
          turret.setHoodForDistance(distanceMeters);
          // FIX 2: kShooterFlywheelMap now returns RPS — convert to rad/s for velocity control
          double flyWheelSpeedRPS = ShooterConstants.kShooterFlywheelMap.get(distanceMeters);
          turret.setFlywheelVelocity(Units.rotationsToRadians(flyWheelSpeedRPS));
        }
      } else {
        turret.rotateToAngle(0.0);
        turret.setHoodAtInitialPosition();
        turret.stopFlywheel();
      }
    }, turret);
  }
}
