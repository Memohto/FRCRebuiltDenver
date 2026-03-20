package frc.robot.commands;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.drive.swerve.Swerve;
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
        Swerve swerve,
        Turret turret,
        Shooter shooter,
        Indexer indexer,
        BooleanSupplier warmUpSupplier,
        BooleanSupplier aimTurretSupplier,
        BooleanSupplier shootSupplier) {
        
        return Commands.run(() -> {
            // Lógica de cálculo de ángulo (raw radians — no Rotation2d wrap)
            double targetRad;
            if (aimTurretSupplier.getAsBoolean()) {
                Pose2d outpostPose = new Pose2d(4.625, 4.035, new Rotation2d());
                Pose2d robotPose = swerve.getPose();
                // .minus() returns robot-relative deltas — heading is already factored out
                Transform2d deltaPose = outpostPose.minus(robotPose);
                double angleInRobotFrame = Math.atan2(deltaPose.getY(), deltaPose.getX());
                double rawAngle = angleInRobotFrame - frc.robot.constants.TurretConstants.turretZeroOffsetRad;

                // Normalize into turret's valid range [minRotationRad, maxRotationRad]
                double min = frc.robot.constants.TurretConstants.minRotationRad;
                double shifted = rawAngle - min;
                shifted = shifted - Math.floor(shifted / (2.0 * Math.PI)) * (2.0 * Math.PI);
                targetRad = shifted + min;

                Logger.recordOutput("Debug/Turret/TargetRotationDeg", Math.toDegrees(targetRad));
            } else {
                targetRad = 0.0;
            }

            turret.rotateToAngle(targetRad);

            // Lógica de Flywheels
            if (warmUpSupplier.getAsBoolean()) {
                turret.startFlywheel();
                if (!aimTurretSupplier.getAsBoolean()) shooter.startFlywheel(); 
            } else {
                turret.stopFlywheel();
                shooter.stopFlywheel();
            }

            // Lógica de Indexer con validación de velocidad
            if (shootSupplier.getAsBoolean()) {
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
public static Command shootingSequenceBoth(Swerve swerve, Turret turret, Shooter shooter, Indexer indexer) {
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
    public static Command shootingSequenceFixedCommand(Swerve swerve, Turret turret, Shooter shooter, Indexer indexer) {
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
    public static Command shootingSequenceTurretCommand(Swerve swerve, Turret turret, Shooter shooter, Indexer indexer) {
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
