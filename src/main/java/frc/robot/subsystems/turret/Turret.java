package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.constants.RobotConstants.TurretMode;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputsAutoLogged;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputsAutoLogged;

public class Turret extends Shooter {
    public static TurretMode mode = TurretMode.NORMAL;

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

    public void rotateToAngle(double positionRad) {
        io.setRotationMotorPosition(positionRad);
    }
}