package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputsAutoLogged;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    public final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public boolean isFlywheelAtSpeed() {
        return Math.abs(inputs.flywheelVelocityRadPerSec) > ShooterConstants.shootingSpeedRadPerSec;
    }

    public void startFlywheel() {
        io.setFlywheelOpenLoop(ShooterConstants.flywheelSpeed);
    }

    public void startFlywheelAtSpeed(double speed) {
        io.setFlywheelOpenLoop(speed);
    }

    public void debugFlywheel() {
        io.setFlywheelOpenLoop(0.25);
    }

    public void stopFlywheel() {
        io.setFlywheelOpenLoop(0.0);
    }

    public void setHoodOpenLoop(double speed) {
        io.setHoodOpenLoop(speed);
    }

    public void setHoodPosition(double distanceToTargetMeters) {
        double hoodAngleDegrees = ShooterConstants.kShotMap.get(distanceToTargetMeters);
        io.setHoodPosition(Rotation2d.fromDegrees(-hoodAngleDegrees));
    }

    public void setHoodRotation(Rotation2d rotation) {
        io.setHoodPosition(rotation);
    }

    public boolean isHoodAtPosition(Rotation2d target, Rotation2d tolerance) {
        return Math.abs(inputs.hoodPosition.getRadians() - target.getRadians()) < tolerance.getRadians();
    }

    public void setHoodInitialPosition() {
        io.setHoodPosition(Rotation2d.fromRotations(-ShooterConstants.minHoodAngleRad));
    }

    public void setHoodResetPosition() {
        io.setHoodPosition(new Rotation2d());
    }

    public boolean isHoodAtResetPosition() {
        Rotation2d tolerance = Rotation2d.fromRotations(ShooterConstants.hoodToleranceRotations);
        return Math.abs(inputs.hoodPosition.getRadians()) < tolerance.getRadians();
    }
}