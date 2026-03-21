package frc.robot.commands;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.constants.RobotConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.turret.Turret;

public class shootingSecuenceCommand {


    public static Command shootingFixed(Shooter shooter) {
        return Commands.runEnd(
            () -> {
                shooter.startFlywheel();
                shooter.setHoodInitialPosition();
            },
            () -> {
                shooter.stopFlywheel();
                shooter.setHoodResetPosition();
            },
            shooter
        );
    }

    public static Command shootingturretCommand(
        Drive swerve,
        Turret turret,
        Shooter shooter,
        Indexer indexer,
        BooleanSupplier warmUpSupplier,
        BooleanSupplier aimTurretSupplier,
        BooleanSupplier shootSupplier) {
        
        return Commands.run(() -> {
            double targetRotationRad = 0.0;
            double distanceMeters = 0;

            if (aimTurretSupplier.getAsBoolean()) {
                boolean isFlipped =
                    DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red;
                Pose2d hubPose = isFlipped ? RobotConstants.redOutpost : RobotConstants.blueOutpost;
                Pose2d robotPose = swerve.getPose();
                double dx = hubPose.getX() - robotPose.getX();
                double dy = hubPose.getY() - robotPose.getY();

                Rotation2d fieldAngleToHub = Rotation2d.fromRadians(Math.atan2(dy, dx));
                targetRotationRad = fieldAngleToHub.minus(robotPose.getRotation()).getRadians();
                distanceMeters = Math.hypot(dx, dy);

                Logger.recordOutput("Debug/Turret/TargetRotationRad", targetRotationRad);
                Logger.recordOutput("Debug/Turret/DistanceToHub", distanceMeters);
            }

            turret.rotateToAngle(targetRotationRad);

            // Ajustar flywheel y hood según distancia
            if (warmUpSupplier.getAsBoolean() && distanceMeters > 0) {
                double flywheelSpeed = ShooterConstants.kFlywheelMap.get(distanceMeters);
                turret.startFlywheelAtSpeed(flywheelSpeed);
                if (!aimTurretSupplier.getAsBoolean()) {
                    shooter.startFlywheel();
                }
                turret.setHoodPosition(distanceMeters);
                shooter.setHoodPosition(distanceMeters);
            } else if (warmUpSupplier.getAsBoolean()) {
                turret.startFlywheel();
                if (!aimTurretSupplier.getAsBoolean()) shooter.startFlywheel();
            } else {
                turret.stopFlywheel();
                shooter.stopFlywheel();
            }

            // Solo disparar si torreta está apuntando Y flywheel a velocidad
            boolean turretReady = !aimTurretSupplier.getAsBoolean() || turret.isAtAngle(targetRotationRad);
            if (shootSupplier.getAsBoolean() && turretReady) {
                indexer.intake();
                if (turret.isFlywheelAtSpeed() && shooter.isFlywheelAtSpeed()) {
                    indexer.indexBoth();
                } else if (turret.isFlywheelAtSpeed()) {
                    indexer.indexTurret();
                } else if (shooter.isFlywheelAtSpeed()) {
                    indexer.indexShooter();
                }
            } else {
                indexer.stopIndexer();
                indexer.stopRollers();
            }
        }, turret, shooter, indexer);
    }


    // --- COMANDOS DE INDEXER (Encapsulados correctamente) ---

    public static Command indexerShooterCommand(Indexer indexer) {
        return Commands.runEnd(indexer::indexShooter, indexer::stopIndexer, indexer);
    }

    public static Command indexerTurretCommand(Indexer indexer) {
        return Commands.runEnd(indexer::indexTurret, indexer::stopIndexer, indexer);
    }

    public static Command indexerBothCommand(Indexer indexer) {
        return Commands.runEnd(indexer::indexBoth, indexer::stopIndexer, indexer);
    }


    // --- SECUENCIAS FUSIONADAS CORREGIDAS ---

/**
 * Dispara con ambos sistemas a la vez (Turret + Fixed Shooter).
 */
public static Command shootingSequenceBoth(Drive swerve, Turret turret, Shooter shooter, Indexer indexer) {
    return shootingturretCommand(
        swerve, 
        turret, 
        shooter, 
        indexer, 
        () -> true, // warmUpSupplier: Siempre encendido
        () -> true, // aimTurretSupplier: Auto-aim activo
        () -> true  // shootSupplier: Disparar (Indexer activo)
    ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
}

/**
     * Dispara solo con el Shooter fijo.
     */
    public static Command shootingSequenceFixedCommand(Drive swerve, Turret turret, Shooter shooter, Indexer indexer) {
        return shootingturretCommand(
            swerve, 
            turret, 
            shooter, 
            indexer, 
            () -> true,  // warmUpSupplier
            () -> false, // aimTurretSupplier: Al ser false, la lógica interna usa el shooter fijo
            () -> true   // shootSupplier
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    /**
     * Dispara solo con la Torreta.
     */
    public static Command shootingSequenceTurretCommand(Drive swerve, Turret turret, Shooter shooter, Indexer indexer) {
        return shootingturretCommand(
            swerve, 
            turret, 
            shooter, 
            indexer, 
            () -> true, // warmUpSupplier
            () -> true, // aimTurretSupplier
            () -> true  // shootSupplier
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
  
}