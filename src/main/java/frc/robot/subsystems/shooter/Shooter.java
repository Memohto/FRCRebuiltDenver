package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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

        // Log error between target and actual for tuning kV/kP
        double velocityError = inputs.flywheelTargetVelocityRadPerSec - inputs.flywheelVelocityRadPerSec;
        Logger.recordOutput("Shooter/FlywheelVelocityErrorRadPerSec", velocityError);
        Logger.recordOutput("Shooter/FlywheelAtSpeed", isFlywheelAtSpeed());
    }

    // ── Flywheel ──────────────────────────────────────────────────────────────

    /**
     * Closed-loop: commands the flywheel to a specific speed in rad/s.
     * The TalonFX will maintain this speed regardless of battery voltage.
     * This is what you should use for all shooting during matches.
     */
    public void setFlywheelVelocity(double velocityRadPerSec) {
        io.setFlywheelVelocity(velocityRadPerSec);
    }

    /**
     * Looks up the correct flywheel speed for a given distance from the shot map
     * and commands closed-loop velocity control.
     * NOTE: kShooterFlywheelMap values must be in RPS (rotations per second),
     * NOT 0-1 duty cycle. Update ShooterConstants accordingly.
     * Example: 60.0 RPS ≈ 3600 RPM (reasonable mid-range shooting speed)
     */
    public void setFlywheelVelocityForDistance(double distanceMeters) {
        double velocityRPS = ShooterConstants.kShooterFlywheelMap.get(distanceMeters);
        double velocityRadPerSec = Units.rotationsToRadians(velocityRPS);
        Logger.recordOutput("Shooter/FlywheelTargetRPS", velocityRPS);
        setFlywheelVelocity(velocityRadPerSec);
    }

    /**
     * Open-loop: sets raw duty cycle. Use ONLY for manual override or testing.
     * Battery-dependent — do not use during matches.
     */
    public void setFlywheelOpenLoop(double speed) {
        io.setFlywheelOpenLoop(speed);
    }

    /** @deprecated Use setFlywheelVelocityForDistance() instead for battery-independent shooting. */
    @Deprecated
    public void setFlywheelSpeed(double speed) {
        io.setFlywheelOpenLoop(speed);
    }

    public void stopFlywheel() {
        io.setFlywheelOpenLoop(0.0);
    }

    /**
     * Returns true when the flywheel is within tolerance of its target speed.
     * Use this to gate indexer/feeding — don't feed until this is true!
     * Tolerance defined in ShooterConstants.flywheelToleranceRadPerSec.
     */
    public boolean isFlywheelAtSpeed() {
        double error = Math.abs(
            inputs.flywheelTargetVelocityRadPerSec - inputs.flywheelVelocityRadPerSec
        );
        return inputs.flywheelTargetVelocityRadPerSec > 0
            && error < ShooterConstants.flywheelToleranceRadPerSec;
    }

    // ── Hood ─────────────────────────────────────────────────────────────────

    public void setHoodOpenLoop(double speed) {
        io.setHoodOpenLoop(speed);
    }

    public void setHoodPosition(Rotation2d rotation) {
        io.setHoodPosition(rotation);
    }

    public void setHoodForDistance(double distanceMeters) {
        double hoodOffsetDeg = ShooterConstants.kShooterHoodMap.get(distanceMeters);
        setHoodPosition(Rotation2d.fromDegrees(hoodOffsetDeg));
    }

    public void setHoodAtInitialPosition() {
        io.setHoodPosition(new Rotation2d());
    }
}
