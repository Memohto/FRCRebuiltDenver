package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.RobotConstants.TurretMode;
import frc.robot.constants.TurretConstants;
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

  public static double computeTurretAngleRad(Pose2d robotPose, Translation2d fieldTarget) {
    double dx = fieldTarget.getX() - robotPose.getX();
    double dy = fieldTarget.getY() - robotPose.getY();
    double worldAngle = Math.atan2(dy, dx);
    double robotHeading = robotPose.getRotation().getRadians();

    double rawAngle = worldAngle - robotHeading - TurretConstants.turretZeroOffsetRad;

    double shifted = rawAngle - TurretConstants.minRotationRad;
    shifted = shifted - Math.floor(shifted / (2.0 * Math.PI)) * (2.0 * Math.PI);
    double targetAngle = shifted + TurretConstants.minRotationRad;

    targetAngle = MathUtil.clamp(targetAngle,
        TurretConstants.minRotationRad, TurretConstants.maxRotationRad);

    return targetAngle;
  }
}