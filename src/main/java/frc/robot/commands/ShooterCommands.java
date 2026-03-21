package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;

public class ShooterCommands {
    private ShooterCommands() {}

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
                    if (TurretCommands.isBomberMode()) {
                        double distance = TurretCommands.getDistanceToTarget();
                        double dutyCycle = TurretConstants.kTurretFlywheelMap.get(distance);
                        shooter.setFlywheelSpeed(dutyCycle);
                        turret.startFlywheelForDistance(distance);
                    } else if (TurretCommands.isTracking()) {
                        shooter.startFlywheel();
                        turret.startFlywheelForDistance(TurretCommands.getDistanceToTarget());
                    } else {
                        shooter.startFlywheel();
                        turret.startFlywheel();
                    }
                } else {
                    shooter.stopFlywheel();
                    turret.stopFlywheel();
                }

                // ── Hoods ──────────────────────────────────────────────────
                if (TurretCommands.isBomberMode()) {
                    // BOMBER: fixed shooter hood auto-adjusts based on distance
                    // (turret hood is managed by TurretCommands.bomberHub)
                    double distance = TurretCommands.getDistanceToTarget();
                    double hoodOffsetDeg = TurretConstants.kTurretHoodMap.get(distance);
                    shooter.setHoodRotation(Rotation2d.fromDegrees(-hoodOffsetDeg));
                } else if (hoodUpSupplier.getAsBoolean()) {
                    shooter.setHoodOpenLoop(ShooterConstants.hoodSpeed);
                    turret.setHoodOpenLoop(ShooterConstants.hoodSpeed);
                } else if (hoodDownSupplier.getAsBoolean()) {
                    shooter.setHoodOpenLoop(-ShooterConstants.hoodSpeed);
                    turret.setHoodOpenLoop(-ShooterConstants.hoodSpeed);
                } else {
                    shooter.setHoodOpenLoop(0);
                    turret.setHoodOpenLoop(0);
                }
            },
            shooter
        );
    }
}