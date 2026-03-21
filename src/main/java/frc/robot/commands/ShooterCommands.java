package frc.robot.commands;

import static frc.robot.constants.VisionConstants.robotToCamera1;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants.RobotMode;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;

public class ShooterCommands {
    private ShooterCommands() {}

    public static Command joystickShooterCmd(
        Shooter shooter,
        Turret turret,
        BooleanSupplier shootSupplier,
        DoubleSupplier distanceToHubSupplier) {
        return Commands.run(
            () -> {
                if (shootSupplier.getAsBoolean()) {
                    double distanceMeters = distanceToHubSupplier.getAsDouble();
                    double hoodOffsetDeg = TurretConstants.kTurretHoodMap.get(distanceMeters);
                    // [TODO] Move set Hood and flywheel fow distance to shooter
                    shooter.setHoodRotation(Rotation2d.fromDegrees(-hoodOffsetDeg));

                    if (Robot.mode == RobotMode.BOMBER) {
                        double dutyCycle = TurretConstants.kTurretFlywheelMap.get(distanceMeters);
                        shooter.setFlywheelSpeed(dutyCycle);
                        turret.startFlywheelForDistance(distanceMeters);
                    } else if (Robot.mode == RobotMode.STRIKER) {
                        shooter.stopFlywheel();
                        turret.startFlywheelForDistance(distanceMeters);
                    } else {
                        shooter.startFlywheel();
                        turret.startFlywheel();
                    }
                } else {
                    shooter.stopFlywheel();
                    turret.stopFlywheel();
                }

                // // ── Hoods ──────────────────────────────────────────────────
                // if (TurretCommands.isBomberMode()) {
                //     // BOMBER: fixed shooter hood auto-adjusts based on distance
                //     // (turret hood is managed by TurretCommands.bomberHub)
                //     double distance = TurretCommands.getDistanceToTarget();
                //     double hoodOffsetDeg = TurretConstants.kTurretHoodMap.get(distance);
                //     shooter.setHoodRotation(Rotation2d.fromDegrees(-hoodOffsetDeg));
                // } else if (hoodUpSupplier.getAsBoolean()) {
                //     shooter.setHoodOpenLoop(ShooterConstants.hoodSpeed);
                //     turret.setHoodOpenLoop(ShooterConstants.hoodSpeed);
                // } else if (hoodDownSupplier.getAsBoolean()) {
                //     shooter.setHoodOpenLoop(-ShooterConstants.hoodSpeed);
                //     turret.setHoodOpenLoop(-ShooterConstants.hoodSpeed);
                // } else {
                //     shooter.setHoodOpenLoop(0);
                //     turret.setHoodOpenLoop(0);
                // }
            },
            shooter
        );
    }
}