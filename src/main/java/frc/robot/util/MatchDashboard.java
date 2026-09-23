package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;

/**
 * Publicación de estado a Elastic durante un partido.
 *
 * <p>
 * Todo sale bajo {@code /SmartDashboard/Match/...}. La pestaña "Competencia"
 * del layout {@code src/main/deploy/elastic-layout.json} la consume.
 *
 * <p>
 * Diseño: en un partido el operador tiene medio segundo para mirar la
 * pantalla. Necesita saber <b>en qué modo estoy</b>, <b>a qué le apunto</b> y
 * <b>puedo tirar ya</b>. Todo lo demás es para el pit y va abajo.
 */
public final class MatchDashboard {

    private static final int CAM = 0;

    // ── Escrito por los comandos cada ciclo ─────────────────────────────────

    /** Estado legible de la torreta: INACTIVA / HUB por odometría / CERO ... */
    public static String turretState = "INACTIVA";

    public static boolean aiming = false;
    public static boolean charging = false;
    public static boolean turretOnTarget = false;
    public static boolean headingAssisted = false;
    public static boolean precisionActive = false;

    public static double distanceMeters = 0.0;
    public static double turretErrorDeg = 0.0;
    public static double shotAimOffsetDeg = 0.0;
    public static double shotCompensationMeters = 0.0;
    public static double fieldSpeedMetersPerSec = 0.0;
    public static double headingErrorDeg = 0.0;

    /** Fuente del rumbo asistido en BOMBER: ODOMETRIA / MANUAL / — */
    public static String alignSource = "—";

    // ── Referencias a subsistemas ───────────────────────────────────────────

    private static Vision vision = null;
    private static Shooter shooter = null;
    private static Turret turret = null;

    private MatchDashboard() {
    }

    public static void configure(Vision visionRef, Shooter shooterRef, Turret turretRef) {
        vision = visionRef;
        shooter = shooterRef;
        turret = turretRef;
    }

    /** Devuelve todo al estado seguro. Se llama al deshabilitar. */
    public static void reset() {
        turretState = "INACTIVA";
        aiming = false;
        charging = false;
        turretOnTarget = false;
        headingAssisted = false;
        precisionActive = false;
        distanceMeters = 0.0;
        turretErrorDeg = 0.0;
        shotAimOffsetDeg = 0.0;
        shotCompensationMeters = 0.0;
        fieldSpeedMetersPerSec = 0.0;
        headingErrorDeg = 0.0;
        alignSource = "—";
    }

    /** ¿Están a velocidad los flywheels que van a disparar en este modo? */
    public static boolean flywheelsReady() {
        if (turret == null || shooter == null) {
            return false;
        }
        // En BOMBER disparan los dos cañones; en STRIKER sólo la torreta.
        return CompetitionState.isBomber()
                ? turret.isFlywheelAtSpeed() && shooter.isFlywheelAtSpeed()
                : turret.isFlywheelAtSpeed();
    }

    /** Tolerancia de rumbo del chasis para declarar "alineado" en BOMBER. */
    private static final double HEADING_READY_DEG = 3.0;

    /**
     * Apuntada + acelerada + cargando. Verde = alimentar mete el tiro.
     *
     * <p>
     * En BOMBER quien apunta es el chasis, así que además de la torreta en cero
     * se exige que la asistencia esté activa y el error de rumbo sea chico.
     */
    public static boolean readyToShoot() {
        boolean aimed = CompetitionState.isBomber()
                ? turretOnTarget && headingAssisted && Math.abs(headingErrorDeg) < HEADING_READY_DEG
                : turretOnTarget;
        return charging && flywheelsReady() && aimed;
    }

    /** Se llama desde {@code Robot.robotPeriodic()}. */
    public static void publish() {
        // ── Estado principal ────────────────────────────────────────────────
        SmartDashboard.putString("Match/Modo", CompetitionState.getMode().toString());
        SmartDashboard.putString("Match/Objetivo", CompetitionState.getTarget().toString());
        SmartDashboard.putString("Match/Torreta", turretState);

        // ── Los indicadores grandes ─────────────────────────────────────────
        SmartDashboard.putBoolean("Match/LISTO PARA TIRAR", readyToShoot());
        SmartDashboard.putBoolean("Match/Torreta apuntada", turretOnTarget);
        SmartDashboard.putBoolean("Match/Flywheels listos", flywheelsReady());
        SmartDashboard.putBoolean("Match/Odometria fresca", FieldTracking.isOdometryValid());

        // ── Detalle ─────────────────────────────────────────────────────────
        SmartDashboard.putBoolean("Match/Apuntando", aiming);
        SmartDashboard.putBoolean("Match/Cargando", charging);
        SmartDashboard.putBoolean("Match/Chasis asistido", headingAssisted);
        SmartDashboard.putBoolean("Match/Modo precision", precisionActive);
        SmartDashboard.putString("Match/Fuente rumbo", alignSource);

        SmartDashboard.putNumber("Match/Distancia m", round(distanceMeters, 2));
        SmartDashboard.putNumber("Match/Error torreta deg", round(turretErrorDeg, 1));
        SmartDashboard.putNumber("Match/Error rumbo deg", round(headingErrorDeg, 1));
        SmartDashboard.putNumber("Match/Compensacion mov deg", round(shotAimOffsetDeg, 1));
        SmartDashboard.putNumber("Match/Compensacion mov m", round(shotCompensationMeters, 2));
        SmartDashboard.putNumber("Match/Velocidad campo mps", round(fieldSpeedMetersPerSec, 2));
        SmartDashboard.putNumber(
                "Match/Sin ver tag s", round(Math.min(FieldTracking.secondsSinceUpdate(), 99.0), 1));

        // ── Salud de la Limelight ───────────────────────────────────────────
        if (vision != null) {
            SmartDashboard.putBoolean("Match/Limelight OK", vision.isConnected(CAM));
            SmartDashboard.putBoolean("Match/Tag a la vista", vision.hasTarget(CAM));
            SmartDashboard.putNumber("Match/Tag ID", vision.getPrimaryTagId(CAM));
            SmartDashboard.putNumber(
                    "Match/Latencia ms", round(vision.getLatencySeconds(CAM) * 1000.0, 1));
            SmartDashboard.putNumber("Match/Pipeline", vision.getPipelineIndex(CAM));

            // La cadena completa: ¿ve el tag? → ¿manda poses? → ¿se aceptan?
            // Si "Tag a la vista" es verde pero "Poses por ciclo" es 0, la
            // Limelight no está publicando botpose (mapa de campo / pipeline).
            // Si manda poses y "Pose aceptada" está en rojo, mira el motivo.
            SmartDashboard.putNumber("Match/Vision/Poses por ciclo", vision.getObservationsLastCycle());
            SmartDashboard.putBoolean("Match/Vision/Pose aceptada", vision.secondsSinceAcceptedPose() < 0.5);
            SmartDashboard.putString("Match/Vision/Ultimo rechazo", vision.getLastRejectReason());
            SmartDashboard.putBoolean("Match/Vision/Sembrando rumbo", vision.seededHeadingLastCycle());
        }

        // Si la alianza está mal en la DS, el robot apunta al HUB del otro
        // lado del campo y desde afuera parece que "no apunta".
        SmartDashboard.putString("Match/Alianza", CompetitionState.isRedAlliance() ? "ROJA" : "AZUL");

        SmartDashboard.putString("Match/Estado", statusLine());
    }

    /** Una línea que resume todo, para el widget de texto grande. */
    private static String statusLine() {
        StringBuilder sb = new StringBuilder();
        sb.append(CompetitionState.getMode())
                .append(" → ")
                .append(CompetitionState.getTarget())
                .append(" · ")
                .append(turretState);
        if (charging) {
            sb.append(readyToShoot() ? " · LISTO" : " · cargando...");
        } else if (aiming) {
            sb.append(" · apuntando");
        }
        if (!FieldTracking.isOdometryValid()) {
            sb.append(" · ODOMETRIA VIEJA");
        }
        return sb.toString();
    }

    private static double round(double value, int decimals) {
        double scale = Math.pow(10, decimals);
        return Math.round(value * scale) / scale;
    }
}
