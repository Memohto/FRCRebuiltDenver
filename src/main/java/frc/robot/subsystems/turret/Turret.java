package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.constants.TurretConstants;
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

    /**
     * Command turret to an absolute position in radians.
     * Uses raw double to avoid Rotation2d's internal ±180° wrapping.
     * The caller is responsible for ensuring the value is within
     * [TurretConstants.minRotationRad, TurretConstants.maxRotationRad].
     */
    public void rotateToAngle(double positionRad) {
        io.setRotationMotorPosition(positionRad);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    //  DISTANCE-BASED AIMING — auto-adjusts hood and flywheel for target range
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Set turret hood angle based on distance to target.
     * Uses TurretConstants.kTurretHoodMap for interpolation.
     * The map returns degrees of offset from starting position;
     * the value is negated because CW = hood UP = negative mechanism position.
     */
    public void setHoodForDistance(double distanceMeters) {
        double hoodOffsetDeg = TurretConstants.kTurretHoodMap.get(distanceMeters);
        setHoodRotation(Rotation2d.fromDegrees(-hoodOffsetDeg));
        Logger.recordOutput("Turret/Hood/DistanceM", distanceMeters);
        Logger.recordOutput("Turret/Hood/OffsetDeg", hoodOffsetDeg);
    }

    /**
     * Set turret flywheel speed based on distance to target.
     * Uses TurretConstants.kTurretFlywheelMap for interpolation.
     * The map returns duty cycle (0.0–1.0).
     */
    public void startFlywheelForDistance(double distanceMeters) {
        double dutyCycle = TurretConstants.kTurretFlywheelMap.get(distanceMeters);
        setFlywheelSpeed(dutyCycle);
        Logger.recordOutput("Turret/Flywheel/DistanceM", distanceMeters);
        Logger.recordOutput("Turret/Flywheel/DutyCycle", dutyCycle);
    }
}