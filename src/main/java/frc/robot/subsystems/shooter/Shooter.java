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

    /** Spin flywheel at the default fixed speed from ShooterConstants. */
    public void startFlywheel() {
        io.setFlywheelOpenLoop(ShooterConstants.flywheelSpeed);
    }

    /** Spin flywheel at a specific duty cycle (0.0–1.0). Used by Turret for distance-based speed. */
    public void setFlywheelSpeed(double dutyCycle) {
        io.setFlywheelOpenLoop(dutyCycle);
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

    /**
     * Set hood position based on distance using the fixed shooter's shot map.
     * The shot map returns degrees of offset from starting position (0 = 17.5° physical).
     * The value is negated because CW (negative) = hood UP due to motor inversion.
     */
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

    /** Move hood to starting position (17.5° physical = mechanism position 0). */
    public void setHoodInitialPosition() {
        io.setHoodPosition(new Rotation2d());
    }

    /** Return hood to starting position (17.5° physical = mechanism position 0). */
    public void setHoodResetPosition() {
        io.setHoodPosition(new Rotation2d());
    }

    public boolean isHoodAtResetPosition() {
        Rotation2d tolerance = Rotation2d.fromRotations(ShooterConstants.hoodToleranceRotations);
        return Math.abs(inputs.hoodPosition.getRadians()) < tolerance.getRadians();
    }
}