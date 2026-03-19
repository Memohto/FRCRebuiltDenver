package frc.robot.commands;

import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class TurretCommands {
    private TurretCommands() {}

    public static Command autoAim(
        Turret turret,
        Drive swerve) {
        return Commands.run(
            () -> {
                boolean isFlipped =
                    DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red;

                Pose2d hubPose = isFlipped ? RobotConstants.redOutpost : RobotConstants.blueOutpost;
                Pose2d robotPose = swerve.getPose();
                double dx = hubPose.getX() - robotPose.getX();
                double dy = hubPose.getY() - robotPose.getY();
                Rotation2d targetRotation = Rotation2d.fromRadians(Math.atan2(dy, dx)).plus(Rotation2d.k180deg);
                Logger.recordOutput("Simulation/Debug/TurretTargetAngle", targetRotation);
                turret.rotateToAngle(targetRotation);
            },
            turret
        );
    }

    public static Command joystickTurretCmd(
        Drive swerve,
        Turret turret,
        Shooter shooter,
        Indexer indexer,
        BooleanSupplier warmUpSupplier,
        BooleanSupplier aimTurretSupplier,
        BooleanSupplier shootSupplier) {
        return Commands.run(
            () -> {
                boolean autoAim = aimTurretSupplier.getAsBoolean();
                boolean bomber = false;
                Rotation2d targetRotation = Rotation2d.kZero;
                if (autoAim) {
                    Pose2d hubPose = new Pose2d(4.625, 4.035, new Rotation2d());
                    Pose2d robotPose = swerve.getPose();
                    Transform2d deltaPose = hubPose.minus(robotPose);
                    targetRotation = Rotation2d.fromRadians(Math.atan2(deltaPose.getY(), deltaPose.getX()));
                } 
                // else {
                //     targetRotation = Rotation2d.kZero;
                //     bomber = true;
                // }

                Logger.recordOutput("Debug/TargetTurretAngle", targetRotation);
                turret.rotateToAngle(targetRotation);

                if(warmUpSupplier.getAsBoolean()) {
                    if(bomber) {
                        shooter.startFlywheel();
                    }
                    turret.startFlywheel();
                } else {
                    shooter.stopFlywheel();
                    turret.stopFlywheel();
                }

                if (shootSupplier.getAsBoolean()) {
                    if(turret.isFlywheelAtSpeed() && shooter.isFlywheelAtSpeed()) {
                        indexer.intake();
                        indexer.indexBoth();
                    } else if(turret.isFlywheelAtSpeed()) {
                        indexer.intake();
                        indexer.indexTurret();
                    } else if(shooter.isFlywheelAtSpeed()){
                        indexer.intake();
                        indexer.indexShooter();
                    } else {
                        indexer.stopIndexer();
                        indexer.stopRollers();
                    }
                } else {
                    indexer.stopIndexer();
                    indexer.stopRollers();
                }

            },
        turret, shooter, indexer);
    }
}
