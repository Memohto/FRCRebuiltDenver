package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.DemoConstants;
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
    // Sin esto, la torreta escribiría al log bajo la clave "Shooter" y pisaría
    // los datos del shooter fijo en cada ciclo.
    this.logKey = "TurretShooter";
  }

  @Override
  public void periodic() {
    super.periodic();
    io.updateInputs(shooterInputs, turretInputs);
    // NOTA: shooterInputs aquí es una lectura duplicada de los MISMOS TalonFX
    // (IDs 28 y 27) que super.periodic() ya leyó y logueó bajo "TurretShooter".
    // Es consecuencia de que RobotContainer le pase dos IOs al mismo hardware.
    // No lo volvemos a loguear para no pisar la tabla. La solución de fondo es
    // que Turret deje de heredar de Shooter y componga un solo TurretIO.
    Logger.processInputs("Turret", turretInputs);

    // Se usa el ángulo en radianes crudos y no el Rotation2d, porque ese
    // envuelve a ±180° y hacía que el modelo 3D pegara un salto al pasar de
    // 180° a 181° aunque la torreta girara continuo.
    visualizer.updateVisualization(new Rotation3d(0.0, 0.0, turretInputs.rotationMotorPositionRad));
  }

  public void rotateToAngle(double positionRad) {
    io.setRotationMotorPosition(positionRad);
  }

  // ══════════════════════════════════════════════════════════════════════════
  // Helpers para el Demo Mode
  // ══════════════════════════════════════════════════════════════════════════

  /** Ángulo actual del mecanismo en radianes, SIN envolver a ±180°. */
  public double getAngleRad() {
    return turretInputs.rotationMotorPositionRad;
  }

  public double getVelocityRadPerSec() {
    return turretInputs.rotationMotorVelocityRadPerSec;
  }

  public boolean isAtAngle(double targetRad, double toleranceRad) {
    return Math.abs(getAngleRad() - targetRad) < toleranceRad;
  }

  /** Mantiene la torreta en cero con Motion Magic. Usado en modo BOMBER. */
  public void holdZero() {
    io.setRotationMotorPosition(0.0);
  }

  /** Recorta un ángulo a los soft limits del mecanismo. */
  public static double clampToLimits(double angleRad) {
    return MathUtil.clamp(angleRad, TurretConstants.minRotationRad, TurretConstants.maxRotationRad);
  }

  /**
   * Transformada robot → cámara para la Limelight montada en la torreta.
   *
   * <p>
   * La cámara gira con el mecanismo, así que esta transformada cambia cada
   * ciclo. Se compone así:
   *
   * <pre>
   *   robot → pivote de torreta  (traslación fija)
   *         → rotación actual    (ángulo del mecanismo + offset de cero)
   *         → lente             (transformada fija en el marco de la torreta)
   * </pre>
   *
   * <p>
   * El {@code turretZeroOffsetRad} (π) se suma aquí porque el cero mecánico de
   * la torreta apunta hacia ATRÁS del robot, no hacia adelante.
   */
  public Transform3d getRobotToCamera() {
    Transform3d pivot = new Transform3d(
        DemoConstants.robotToTurretPivot,
        new Rotation3d(0.0, 0.0,
            getAngleRad() * DemoConstants.turretCameraYawSign
                + TurretConstants.turretZeroOffsetRad));
    return pivot.plus(DemoConstants.turretPivotToCamera);
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