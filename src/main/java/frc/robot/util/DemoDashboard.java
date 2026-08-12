package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.DemoConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;

/**
 * Publicación de estado a Elastic.
 *
 * <p>
 * Todo sale bajo {@code /SmartDashboard/Demo/...}, que es donde Elastic lo
 * encuentra sin configuración extra. El layout listo para importar está en
 * {@code src/main/deploy/elastic-layout.json}.
 *
 * <p>
 * La idea de diseño: durante una demo el operador no puede estar interpretando
 * gráficas. Necesita tres cosas de un vistazo — <b>en qué modo estoy</b>, <b>ve
 * la cámara mi objetivo</b>, y <b>puedo tirar ya</b>. Todo lo demás es
 * secundario y va abajo en el layout.
 */
public class DemoDashboard {

    private static final int CAM = 0;

    // ── Escrito por los comandos cada ciclo ─────────────────────────────────

    /** Estado legible de la torreta: INACTIVA / BUSCANDO / ENGANCHADA / APUNTADA. */
    public static String turretState = "INACTIVA";

    public static boolean aiming = false;
    public static boolean charging = false;
    public static boolean targetVisible = false;
    public static boolean turretOnTarget = false;

    public static int tagId = -1;
    public static double distanceMeters = 0.0;
    /**
     * Cuánto le FALTA girar a la torreta, en grados, positivo en sentido CCW.
     *
     * <p>
     * Es la misma convención en las dos ramas de apuntado —visión y odometría—,
     * así que el número no cambia de signo solo cuando el tag aparece o se
     * pierde. Ojo si vienen de la versión anterior: la rama de visión reportaba
     * {@code tx} crudo, que tiene el signo contrario.
     */
    public static double turretErrorDeg = 0.0;

    /**
     * Sesgo aprendido entre el apuntado por visión y el por odometría, en grados.
     *
     * <p>
     * Es cuánto se equivoca la pose del robot vista desde la torreta. Unos pocos
     * grados es normal. Si crece mucho o no para de moverse, la pose o la
     * calibración de la cámara están mal y vale la pena revisarlas antes de
     * confiar en el apuntado sin ver el tag.
     */
    public static double turretBiasDeg = 0.0;

    /** "SUAVE" / "COMPETENCIA" / "VOLCADO x%" / "—". */
    public static String shotPower = "—";

    // Manejo
    public static double speedFraction = DemoConstants.maxSpeedFraction;
    public static boolean creepActive = false;
    public static boolean headingAssisted = false;

    // Follow-me
    public static boolean followLocked = false;
    public static double followDistanceMeters = 0.0;

    /** Fuente del apuntado del chasis en BOMBER. */
    public static String alignSource = "—";

    /**
     * Cuánto se corrió el punto de mira por disparo en movimiento, en metros.
     *
     * <p>
     * Es el indicador para tunear el shoot-while-move: parado debe ser 0, y al
     * moverte lateralmente debe crecer proporcional a tu velocidad.
     */
    public static double shotCompensationMeters = 0.0;

    /**
     * Corrección angular del shoot-while-move, en grados.
     *
     * <p>
     * Es la que de verdad mueve la torreta, ya gateada: si el apuntado no la
     * está aplicando, aquí sale 0.
     *
     * <p>
     * Parado debe ser 0. Dándole la cara al HUB y manejando <b>a la derecha</b>
     * debe crecer <b>positivo</b> — positivo es CCW, o sea que el apuntado se
     * corre a la izquierda, contra tu movimiento, que es lo que dice la física.
     * Si crece negativo, el signo está invertido: ver
     * {@code shootWhileMovingAimSign}.
     */
    public static double shotAimOffsetDeg = 0.0;

    /**
     * Rapidez del robot en marco de campo, m/s.
     *
     * <p>
     * Es el contexto sin el cual los otros dos números no se pueden leer: la
     * compensación tiene que ser proporcional a esto. Parado, cero; y si a 1 m/s
     * la compensación no llega ni a un grado, el problema es la ganancia, no el
     * signo.
     */
    public static double fieldSpeedMetersPerSec = 0.0;

    /**
     * Sesgo aprendido entre el rumbo de visión y el de odometría, en grados.
     *
     * <p>
     * Es lo que hace que el traspaso visión → odometría sea continuo. Si crece
     * mucho (más de ~20°), la pose del robot está bastante equivocada y vale la
     * pena revisar la calibración de la cámara.
     */
    public static double alignBiasDeg = 0.0;

    // ── Referencias a subsistemas ───────────────────────────────────────────

    private static Vision vision = null;
    private static Shooter shooter = null;
    private static Turret turret = null;

    private DemoDashboard() {
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
        targetVisible = false;
        turretOnTarget = false;
        tagId = -1;
        distanceMeters = 0.0;
        turretErrorDeg = 0.0;
        turretBiasDeg = 0.0;
        shotPower = "—";
        creepActive = false;
        headingAssisted = false;
        followLocked = false;
        followDistanceMeters = 0.0;
        alignSource = "—";
        shotCompensationMeters = 0.0;
        shotAimOffsetDeg = 0.0;
        fieldSpeedMetersPerSec = 0.0;
        alignBiasDeg = 0.0;
    }

    /** Se llama desde {@code Robot.robotPeriodic()}. */
    public static void publish() {
        // ── Estado principal ────────────────────────────────────────────────
        SmartDashboard.putString("Demo/Modo", DemoState.getMode().toString());
        SmartDashboard.putString("Demo/Feature", featureText());
        SmartDashboard.putString("Demo/Torreta", turretState);
        SmartDashboard.putString("Demo/Potencia de tiro", shotPower);

        // ── Los tres indicadores grandes ────────────────────────────────────
        boolean flywheelsReady = false;
        if (turret != null && shooter != null) {
            // En BOMBER disparan los dos cañones, así que los dos tienen que
            // estar a velocidad. En STRIKER sólo importa la torreta.
            flywheelsReady = DemoState.isBomber() && !DemoState.isFollowing()
                    ? turret.isFlywheelAtSpeed() && shooter.isFlywheelAtSpeed()
                    : turret.isFlywheelAtSpeed();
        }

        SmartDashboard.putBoolean("Demo/Objetivo a la vista", targetVisible);
        SmartDashboard.putBoolean("Demo/Torreta apuntada", turretOnTarget);
        SmartDashboard.putBoolean("Demo/Flywheels listos", flywheelsReady);

        // El indicador que de verdad importa: apuntada + acelerada + cargando.
        // Si esto está en verde, alimentar con la Y-valve mete el tiro.
        // En volcado suave no hay objetivo al cual apuntar: basta con que los
        // flywheels estén a velocidad.
        boolean readyToShoot = charging && flywheelsReady
                && (DemoState.isSmoothDump() || turretOnTarget);
        SmartDashboard.putBoolean("Demo/LISTO PARA TIRAR", readyToShoot);

        // ── Detalle ─────────────────────────────────────────────────────────
        SmartDashboard.putBoolean("Demo/Apuntando", aiming);
        SmartDashboard.putBoolean("Demo/Cargando", charging);
        SmartDashboard.putBoolean("Demo/Siguiendo target", DemoState.isFollowing());
        SmartDashboard.putBoolean("Demo/Chasis asistido", headingAssisted);
        SmartDashboard.putBoolean("Demo/Modo precision", creepActive);

        SmartDashboard.putNumber("Demo/Tag ID", tagId);
        SmartDashboard.putNumber("Demo/Distancia m", round(distanceMeters, 2));
        SmartDashboard.putNumber("Demo/Error torreta deg", round(turretErrorDeg, 2));
        SmartDashboard.putNumber("Demo/Velocidad pct", Math.round(speedFraction * 100.0));

        // ── Salud de la Limelight ───────────────────────────────────────────
        if (vision != null) {
            SmartDashboard.putBoolean("Demo/Limelight OK", vision.isConnected(CAM));
            SmartDashboard.putNumber("Demo/Pipeline", vision.getPipelineIndex(CAM));
            SmartDashboard.putNumber(
                    "Demo/Latencia ms", round(vision.getLatencySeconds(CAM) * 1000.0, 1));
            SmartDashboard.putNumber("Demo/Tag lockeado", DemoState.getLockedTagId());
        }

        if (DemoState.isFollowing()) {
            SmartDashboard.putBoolean("Demo/Follow enganchado", followLocked);
            SmartDashboard.putNumber("Demo/Follow distancia m", round(followDistanceMeters, 2));
        }
        // Memoria de campo: de dónde sale el apuntado y hace cuánto que la
        // visión no confirma dónde estamos.
        SmartDashboard.putString("Demo/Fuente apuntado", alignSource);
        SmartDashboard.putBoolean("Demo/Volcado libre", DemoState.isSmoothDump());
        SmartDashboard.putNumber(
                "Demo/Compensacion mov m", round(shotCompensationMeters, 2));
        SmartDashboard.putNumber(
                "Demo/Compensacion mov deg", round(shotAimOffsetDeg, 1));
        SmartDashboard.putNumber("Demo/Sesgo torreta deg", round(turretBiasDeg, 1));
        SmartDashboard.putNumber("Demo/Sesgo rumbo deg", round(alignBiasDeg, 1));
        SmartDashboard.putNumber(
                "Demo/Velocidad campo mps", round(fieldSpeedMetersPerSec, 2));
        SmartDashboard.putBoolean("Demo/Odometria fresca", FieldTracking.isOdometryValid());
        SmartDashboard.putNumber(
                "Demo/Sin ver tag s", round(Math.min(FieldTracking.secondsSinceUpdate(), 99.0), 1));

        // ── Línea de estado para el widget de texto grande ──────────────────
        SmartDashboard.putString("Demo/Estado", statusLine());
    }

    private static String featureText() {
        if (DemoState.isStriker()) {
            return switch (DemoState.getStrikerTargeting()) {
                case HUB -> "HUB (competencia)";
                case GLOBAL_HUNT -> DemoState.getLockedTagId() >= 0
                        ? "Caza libre · tag " + DemoState.getLockedTagId()
                        : "Caza libre";
            };
        }
        return switch (DemoState.getBomberTask()) {
            case HUB_ORBIT -> "Orbitar HUB";
            case FOLLOW_TARGET -> "Follow-me";
            case SMOOTH_DUMP -> "Volcado suave";
        };
    }

    /**
     * Una sola línea que resume todo, para el widget de texto grande.
     *
     * <p>
     * Es lo que el mentor lee de reojo desde tres metros mientras un alumno
     * maneja.
     */
    private static String statusLine() {
        StringBuilder sb = new StringBuilder();
        sb.append(DemoState.getMode()).append(" · ").append(turretState);
        if (tagId >= 0) {
            sb.append(" · tag ").append(tagId);
        }
        if (distanceMeters > 0.05) {
            sb.append(" · ").append(String.format("%.1f m", distanceMeters));
        }
        if (charging) {
            sb.append(" · CARGANDO ").append(shotPower);
        }
        return sb.toString();
    }

    private static double round(double value, int decimals) {
        double factor = Math.pow(10.0, decimals);
        return Math.round(value * factor) / factor;
    }
}
