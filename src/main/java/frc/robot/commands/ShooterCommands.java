package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.RobotConstants.DriveMode;
import frc.robot.constants.RobotConstants.RobotMode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ShooterCommands {
  private ShooterCommands() {
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

  public static Command joystickShooterCmd(
      Shooter shooter,
      Turret turret,
      BooleanSupplier shootSupplier,
      Supplier<Pose2d> robotPoseSupplier) {
    return Commands.run(
        () -> {
          if (shootSupplier.getAsBoolean() && Robot.mode == RobotMode.BOMBER) {
            Pose2d robotPose = robotPoseSupplier.get();
            Translation2d target = getBomberTarget(robotPose);
            double distanceMeters = Drive.getDistanceToTargetMeters(robotPose, target);

            if (Drive.mode != DriveMode.NORMAL) {
              // ── Closed-loop velocity control (battery-independent) ────────────
              // setHoodForDistance is unchanged — hood still uses position PID
              shooter.setHoodForDistance(distanceMeters);

              // Use velocity control: looks up RPS from the shot map and commands
              // the TalonFX to chase that speed using its onboard PID + feedforward.
              // The motor will add more voltage as the battery sags to maintain speed.
              shooter.setFlywheelVelocityForDistance(distanceMeters);

            } else {
              // NORMAL drive mode: spin at default speed, centered hood
              shooter.setHoodAtInitialPosition();
              // Default speed is stored as RPS in ShooterConstants
              shooter.setFlywheelVelocity(
                  Units.rotationsToRadians(ShooterConstants.flywheelDefaultSpeedRPS)
              );
            }
          } else {
            // Trigger released or wrong mode — stop everything
            shooter.stopFlywheel();
            shooter.setHoodAtInitialPosition();
          }
        },
        shooter);
  }
}
