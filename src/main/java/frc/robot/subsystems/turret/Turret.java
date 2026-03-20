package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputsAutoLogged;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputsAutoLogged;

public class Turret extends Shooter {
    private final TurretIO io;
    private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();
    private final TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();

    private final TurretVisualizer visualizer = new TurretVisualizer();

    public Turret(TurretIO turretIO, ShooterIO shooterIO) {
        super(shooterIO);
        this.io = turretIO;
    }

    @Override
    public void periodic() {
        super.periodic();
        io.updateInputs(shooterInputs, turretInputs);
        Logger.processInputs("TurretShooter", shooterInputs);
        Logger.processInputs("Turret", turretInputs);

        visualizer.updateVisualization(new Rotation3d(turretInputs.rotationMotorPosition));
    }

    /** Returns turret position as Rotation2d (wraps at ±180° — display only). */
    public Rotation2d getRotation() {
        return turretInputs.rotationMotorPosition;
    }

    /** Returns turret position in raw radians (NOT wrapped — use for control logic). */
    public double getRotationRad() {
        return turretInputs.rotationMotorPositionRad;
    }

    /** Open-loop rotation at the given duty cycle [-1, 1]. */
    public void rotate(double speed) {
        io.setRotationMotorOpenLoop(speed);
    }

    /**
     * Command turret to an absolute position in radians.
     * Uses raw double to avoid Rotation2d's internal ±180° wrapping.
     * The caller is responsible for ensuring the value is within
     * [TurretConstants.minRotationRad, TurretConstants.maxRotationRad].
     */
    public void rotateToAngle(double positionRad) {
        io.setRotationMotorPosition(positionRad);
    }
}
