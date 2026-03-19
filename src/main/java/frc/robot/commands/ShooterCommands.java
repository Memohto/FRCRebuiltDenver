package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;

public class ShooterCommands {
    private ShooterCommands() {}

    private static boolean flywheelRunning = false;

    public static Command joystickShooterCmd(
        Shooter shooter,
        Turret turret,
        BooleanSupplier shootSupplier,
        BooleanSupplier hoodUpSupplier,
        BooleanSupplier hoodDownSupplier,
        BooleanSupplier rotateUpSupplier,
        BooleanSupplier rotateDownSupplier) {
        return Commands.run(
            () -> {
                if (shootSupplier.getAsBoolean()) {
                    shooter.startFlywheel();
                    turret.startFlywheel();
                } else {
                    shooter.stopFlywheel();
                    turret.stopFlywheel();
                }

                if (hoodUpSupplier.getAsBoolean()) {
                    shooter.setHoodOpenLoop(ShooterConstants.hoodSpeed);
                    turret.setHoodOpenLoop(ShooterConstants.hoodSpeed);
                } else if (hoodDownSupplier.getAsBoolean()) {
                    shooter.setHoodOpenLoop(-ShooterConstants.hoodSpeed);
                    turret.setHoodOpenLoop(-ShooterConstants.hoodSpeed);
                } else {
                    shooter.setHoodOpenLoop(0);
                    turret.setHoodOpenLoop(0);
                }

                if (rotateUpSupplier.getAsBoolean()) {
                    turret.rotate(ShooterConstants.hoodSpeed);
                } else if (rotateDownSupplier.getAsBoolean()) {
                    turret.rotate(-ShooterConstants.hoodSpeed);
                } else {
                    turret.rotate(0);
                }
            },
            shooter, turret
        );
    }

    //manual commands
    public static Command flywheelCommand(
        Shooter shooter,
        BooleanSupplier toggleSupplier) {

        return Commands.run(
            () -> {
                if (toggleSupplier.getAsBoolean()) {
                    flywheelRunning = !flywheelRunning;
                }

                if (flywheelRunning) {
                    shooter.startFlywheel();
                } else {
                    shooter.stopFlywheel();
                }
            },
        shooter);
    }

    public static Command manualHood(
        Shooter shooter,
        DoubleSupplier voltageSupplier) {

        return Commands.run(
            () -> shooter.setHoodOpenLoop(voltageSupplier.getAsDouble()),
        shooter);
    }

    //automated commands
    public static Command hoodInitialPosition(
        Shooter shooter) {

        return Commands.run(
            () -> {
                Rotation2d target    = Rotation2d.fromRotations(-ShooterConstants.minHoodAngleRad);
                Rotation2d tolerance = Rotation2d.fromRotations(ShooterConstants.hoodToleranceRotations);

                if (!shooter.isHoodAtPosition(target, tolerance)) {
                    shooter.setHoodInitialPosition();
                } else {
                    shooter.setHoodOpenLoop(0);
                }
            },
        shooter);
    }

    public static Command hoodResetPosition(
        Shooter shooter) {

        return Commands.run(
            () -> {
                if (!shooter.isHoodAtResetPosition()) {
                    shooter.setHoodResetPosition();
                } else {
                    shooter.setHoodOpenLoop(0);
                }
            },
        shooter);
    
}
}