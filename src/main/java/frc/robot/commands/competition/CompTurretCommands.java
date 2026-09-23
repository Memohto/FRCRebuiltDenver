package frc.robot.commands.competition;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.CompetitionConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.CompetitionState;
import frc.robot.util.FieldTracking;
import frc.robot.util.MatchDashboard;
import frc.robot.util.ShotSolution;
import frc.robot.util.TurretAimTracker;

/**
 * Torreta en COMPETENCIA v2.
 *
 * <h2>Apuntado por odometría</h2>
 *
 * La torreta sabe dónde está el objetivo (constante del campo) y dónde está el
 * robot (pose estimator: encoders + Pigeon + MegaTag2 de la Limelight fija).
 * Con eso calcula su ángulo cada 20 ms con
 * {@link Turret#computeTurretAngleRad}, el método de Denver. Lo que cambia
 * respecto a Denver es <b>cómo llega ese ángulo al motor</b>: pasa por
 * {@link TurretAimTracker} (filtro, slew y deadband de comando) en vez de
 * comandarse crudo a 50 Hz, que es lo que hacía temblar el mecanismo.
 *
 * <h2>Flujo</h2>
 *
 * <pre>
 *   sin botón    → torreta a cero, flywheel apagado, hood en reposo
 *   APUNTAR (X)  → STRIKER: rastrea el objetivo   |  BOMBER: se queda en cero
 *   CARGAR  (RT) → lo mismo + hood y flywheel a la solución de tiro
 * </pre>
 *
 * <p>
 * La compensación de disparo en movimiento ({@link ShotSolution}) entra como
 * argumento de {@code computeTurretAngleRad}, ANTES del envolvimiento y del
 * clamp, para que un objetivo cerca de la costura de los soft limits dé la
 * vuelta en vez de saturar.
 */
public final class CompTurretCommands {

    private CompTurretCommands() {
    }

    /**
     * Default command de la torreta.
     *
     * @param aimSupplier    APUNTAR: rastrea, sin flywheel.
     * @param chargeSupplier CARGAR: rastrea y acelera a la solución de tiro.
     */
    public static Command turretCmd(
            Turret turret,
            Drive drive,
            BooleanSupplier aimSupplier,
            BooleanSupplier chargeSupplier) {

        TurretAimTracker tracker = new TurretAimTracker();

        return Commands.run(
                () -> {
                    boolean charging = chargeSupplier.getAsBoolean();
                    boolean aiming = aimSupplier.getAsBoolean() || charging;

                    // ═══════════ INACTIVA ═══════════
                    if (!aiming) {
                        turret.holdZero();
                        turret.stopFlywheel();
                        turret.setHoodAtInitialPosition();
                        tracker.reset(turret.getAngleRad());

                        MatchDashboard.turretState = "INACTIVA";
                        MatchDashboard.aiming = false;
                        MatchDashboard.charging = false;
                        MatchDashboard.turretOnTarget = false;
                        MatchDashboard.distanceMeters = 0.0;
                        MatchDashboard.turretErrorDeg = 0.0;
                        MatchDashboard.shotAimOffsetDeg = 0.0;
                        MatchDashboard.shotCompensationMeters = 0.0;
                        MatchDashboard.fieldSpeedMetersPerSec = 0.0;
                        Logger.recordOutput("Match/Turret/State", "IDLE");
                        return;
                    }

                    MatchDashboard.aiming = true;
                    MatchDashboard.charging = charging;

                    // Solución de tiro compartida con el cañón fijo y el chasis.
                    Pose2d pose = drive.getPose();
                    Translation2d target = CompetitionState.getActiveTarget(pose);
                    ShotSolution shot = ShotSolution.compute(
                            pose, drive.getFieldRelativeVelocity(), target);
                    shot.log();

                    double distance = shot.distanceMeters;
                    MatchDashboard.distanceMeters = distance;
                    MatchDashboard.fieldSpeedMetersPerSec = drive.getFieldRelativeVelocity().getNorm();
                    MatchDashboard.shotCompensationMeters = shot.compensationMeters;

                    // ═══════════ APUNTADO ═══════════
                    if (CompetitionState.isBomber()) {
                        // En BOMBER la torreta se congela en cero y el chasis
                        // apunta (CompDriveCommands). Los dos cañones salen
                        // hacia atrás.
                        turret.holdZero();
                        tracker.reset(turret.getAngleRad());
                        MatchDashboard.turretState = "CERO (chasis apunta)";
                        MatchDashboard.turretOnTarget = turret.isAtAngle(
                                0.0, CompetitionConstants.turretOnTargetToleranceRad);
                        MatchDashboard.turretErrorDeg = Math.toDegrees(-turret.getAngleRad());
                        MatchDashboard.shotAimOffsetDeg = 0.0;
                        Logger.recordOutput("Match/Turret/State", "HOLD_ZERO");

                    } else {
                        // STRIKER: apuntado continuo por odometría.
                        boolean tooClose = pose.getTranslation().getDistance(target)
                                < CompetitionConstants.turretAimMinDistanceMeters;

                        if (tooClose) {
                            // Pegados al objetivo unos centímetros de pose son
                            // decenas de grados: se congela el último ángulo.
                            tracker.hold();
                            Logger.recordOutput("Match/Turret/State", "TOO_CLOSE");
                        } else {
                            double angle = Turret.computeTurretAngleRad(pose, target, shot.aimOffsetRad);
                            tracker.track(angle);
                            Logger.recordOutput("Match/Turret/State",
                                    FieldTracking.isOdometryValid() ? "ODOMETRY" : "ODOMETRY_STALE");
                            Logger.recordOutput("Match/Turret/TargetAngleRad", angle);
                        }
                        tracker.apply(turret);

                        String targetName = CompetitionState.isHubTarget() ? "HUB" : "FEEDER";
                        MatchDashboard.turretState = tooClose
                                ? targetName + " · muy cerca"
                                : FieldTracking.isOdometryValid()
                                        ? targetName + " por odometría"
                                        : String.format("%s por odometría (%.0fs sin tag)",
                                                targetName, FieldTracking.secondsSinceUpdate());
                        MatchDashboard.turretOnTarget = turret.isAtAngle(
                                tracker.setpoint(), CompetitionConstants.turretOnTargetToleranceRad);
                        MatchDashboard.turretErrorDeg = Math.toDegrees(
                                tracker.setpoint() - turret.getAngleRad());
                        MatchDashboard.shotAimOffsetDeg = Math.toDegrees(shot.aimOffsetRad);
                    }

                    Logger.recordOutput("Match/Turret/SetpointRad", tracker.setpoint());
                    Logger.recordOutput("Match/Turret/AimOffsetDeg", Math.toDegrees(shot.aimOffsetRad));
                    Logger.recordOutput("Match/ShotDistanceMeters", distance);

                    // ═══════════ CARGA ═══════════
                    if (!charging) {
                        turret.stopFlywheel();
                        turret.setHoodAtInitialPosition();
                        return;
                    }

                    // Mapas de competencia, con la distancia al objetivo VIRTUAL
                    // (ya compensada por movimiento). Con la compensación
                    // apagada es la distancia real, como en Denver.
                    turret.setHoodForDistance(distance);
                    turret.setFlywheelVelocityForDistance(distance);
                },
                turret)
                .beforeStarting(() -> tracker.reset(turret.getAngleRad()));
    }
}
