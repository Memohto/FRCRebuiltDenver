package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.CompetitionConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.CompetitionTarget;
import frc.robot.constants.RobotConstants.RobotMode;

/**
 * Estado global del robot de COMPETENCIA (v2).
 *
 * <p>
 * Reemplaza a los tres enums estáticos de Denver ({@code Robot.mode},
 * {@code Drive.mode}, {@code Turret.mode}), que se pisaban entre sí desde
 * cuatro botones distintos y era imposible saber en qué combinación estaba el
 * robot sin leer el log. Aquí hay exactamente dos preguntas:
 *
 * <pre>
 *   ¿QUIÉN apunta?    STRIKER = la torreta   |  BOMBER = el chasis (torreta en cero)
 *   ¿A QUÉ le apunta? HUB                    |  FEEDER (pase hacia la propia alianza)
 * </pre>
 *
 * <p>
 * Todo cambio se loguea, así que en AdvantageScope se ve exactamente en qué
 * modo estaba el robot en cada frame.
 *
 * <h2>Estado al dar enable</h2>
 *
 * STRIKER apuntando al HUB. Es la configuración de siempre: torreta rastreando
 * el HUB por odometría y el piloto con el chasis 100% manual.
 */
public final class CompetitionState {

    private static RobotMode mode = RobotMode.STRIKER;
    private static CompetitionTarget target = CompetitionTarget.HUB;

    private CompetitionState() {
    }

    // ── Getters ─────────────────────────────────────────────────────────────

    public static RobotMode getMode() {
        return mode;
    }

    public static CompetitionTarget getTarget() {
        return target;
    }

    public static boolean isStriker() {
        return mode == RobotMode.STRIKER;
    }

    public static boolean isBomber() {
        return mode == RobotMode.BOMBER;
    }

    public static boolean isHubTarget() {
        return target == CompetitionTarget.HUB;
    }

    // ── Transiciones ────────────────────────────────────────────────────────

    /** Alterna STRIKER ↔ BOMBER. Botón Y del operador. */
    public static void toggleMode() {
        mode = isStriker() ? RobotMode.BOMBER : RobotMode.STRIKER;
        log();
    }

    /** Alterna HUB ↔ FEEDER. Botón Back del operador. */
    public static void toggleTarget() {
        target = isHubTarget() ? CompetitionTarget.FEEDER : CompetitionTarget.HUB;
        log();
    }

    public static void setMode(RobotMode newMode) {
        mode = newMode;
        log();
    }

    public static void setTarget(CompetitionTarget newTarget) {
        target = newTarget;
        log();
    }

    /** Vuelve al estado por defecto. Se llama en teleopInit. */
    public static void reset() {
        mode = RobotMode.STRIKER;
        target = CompetitionTarget.HUB;
        log();
    }

    // ── Objetivos ───────────────────────────────────────────────────────────

    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
    }

    /** HUB de la alianza que reporte la Driver Station. */
    public static Translation2d getHubPosition() {
        return isRedAlliance() ? RobotConstants.redHub : RobotConstants.blueHub;
    }

    /**
     * Punto de pase para FEEDER: sobre la línea de la alianza, a la misma Y del
     * robot. El robot "tira hacia su pared" y la pelota cae en la zona propia.
     */
    public static Translation2d getFeederPosition(Pose2d robotPose) {
        double x = isRedAlliance()
                ? CompetitionConstants.feederTargetXRed
                : CompetitionConstants.feederTargetXBlue;
        return new Translation2d(x, robotPose.getY());
    }

    /**
     * Objetivo activo del apuntado según {@link #getTarget()}.
     *
     * <p>
     * Torreta, cañón fijo y rumbo del chasis llaman aquí con la misma pose, así
     * que apuntan siempre al mismo punto.
     */
    public static Translation2d getActiveTarget(Pose2d robotPose) {
        return isHubTarget() ? getHubPosition() : getFeederPosition(robotPose);
    }

    public static void log() {
        Logger.recordOutput("Match/Mode", mode);
        Logger.recordOutput("Match/Target", target);
    }
}
