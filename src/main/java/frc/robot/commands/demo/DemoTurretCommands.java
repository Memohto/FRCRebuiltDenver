package frc.robot.commands.demo;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.DemoConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.DemoDashboard;
import frc.robot.util.DemoState;
import frc.robot.util.FieldTracking;
import frc.robot.util.ShotSolution;

/**
 * Comportamiento de la torreta en Demo Mode.
 *
 * <h2>Una sola máquina de estados para STRIKER</h2>
 *
 * <pre>
 *   1. VE EL TAG        → cierra el lazo sobre tx. Es lo más preciso y no
 *                         depende de la pose del robot para nada.
 *   2. LO PERDIÓ HACE POCO
 *                       → mantiene el último apuntado (gracia). Alguien se
 *                         cruzó enfrente, no hay que reaccionar.
 *   3. MODO HUB Y POSE FRESCA
 *                       → apunta por odometría con
 *                         {@code Turret.computeTurretAngleRad}, el método del
 *                         código de competencia. Es lo que le permite mantener
 *                         el HUB cuando ya salió del rango de la cámara.
 *   4. NADA DE LO ANTERIOR
 *                       → barre buscando, rotando pipelines.
 * </pre>
 *
 * <p>
 * El orden importa y el nivel 4 es el que hace que el sistema arranque solo: al
 * habilitar, la pose todavía no está confirmada por visión, así que barre; en
 * cuanto encuentra el tag se engancha por visión y de paso la visión corrige la
 * pose; a partir de ahí, perder el tag ya no importa porque el nivel 3 toma el
 * relevo.
 *
 * <h2>Nota de mantenimiento</h2>
 *
 * <b>Este archivo no debe tocarse para arreglar BOMBER.</b> En BOMBER la torreta
 * sólo se congela en cero; todo el apuntado de ese modo vive en
 * {@code DemoDriveCommands}, que es quien mueve el chasis. Mezclar las dos cosas
 * fue lo que rompió el rastreo de STRIKER.
 */
public class DemoTurretCommands {

    private static final int CAM = DemoDriveCommands.TURRET_CAMERA;

    private DemoTurretCommands() {
    }

    // ════════════════════════════════════════════════════════════════════════
    // Rastreador con histéresis de enganche
    // ════════════════════════════════════════════════════════════════════════

    /**
     * Genera setpoints suaves para la torreta.
     *
     * <h2>Histéresis: por qué no tiembla</h2>
     *
     * Por buena que sea la ganancia, {@code tx} nunca es exactamente cero:
     * oscila un par de décimas de grado por ruido de píxel. Con un lazo continuo
     * esas décimas se vuelven comandos, cada comando reinicia el perfil de
     * Motion Magic, y el mecanismo tiembla persiguiendo ruido.
     *
     * <p>
     * Cuando el error cae bajo {@code turretLockThresholdRad} la torreta se
     * declara enganchada y <b>congela el setpoint</b>. No vuelve a moverse hasta
     * que el error supere {@code turretUnlockThresholdRad}, que es notablemente
     * mayor, así que el ruido nunca alcanza a desengancharla.
     *
     * <h2>Compensación de latencia</h2>
     *
     * El {@code tx} que leemos describe dónde estaba el target hace 40 ms.
     * Aplicarlo sobre el ángulo actual corrige de más y produce oscilación, así
     * que primero se calcula dónde estaba la torreta en el instante de la
     * captura.
     */
    private static class TurretTracker {
        private double setpointRad = 0.0;
        private double lastCommandedRad = 0.0;
        private boolean initialized = false;
        private boolean locked = false;
        private double lastErrorRad = 0.0;
        private double lastTimestamp = -1.0;
        private double lastSampleTimestamp = -1.0;

        void reset(double currentAngleRad) {
            setpointRad = currentAngleRad;
            lastCommandedRad = currentAngleRad;
            initialized = false;
            locked = false;
            lastErrorRad = 0.0;
            lastTimestamp = -1.0;
            lastSampleTimestamp = -1.0;
        }

        private double stepSeconds() {
            double now = Timer.getFPGATimestamp();
            double dt = (lastTimestamp < 0.0) ? 0.02 : now - lastTimestamp;
            lastTimestamp = now;
            return MathUtil.clamp(dt, 0.001, 0.1);
        }

        /** Seguimiento por visión: {@code tx} es el error de apuntado directo. */
        void updateFromVision(
                Turret turret, double txRad, double latencySeconds, double sampleTimestamp) {
            // Con la cámara a ~22 FPS y el código a 50 Hz, dos de cada tres
            // lecturas son el mismo frame repetido. Reprocesarlo aplicaría la
            // compensación de latencia otra vez sobre un ángulo que sí cambió.
            if (sampleTimestamp == lastSampleTimestamp) {
                return;
            }
            lastSampleTimestamp = sampleTimestamp;
            lastErrorRad = txRad;

            // ── Histéresis ────────────────────────────────────────────────
            if (locked) {
                if (Math.abs(txRad) < DemoConstants.turretUnlockThresholdRad) {
                    return; // Enganchada y quieta. No se comanda nada.
                }
                locked = false;
            } else if (Math.abs(txRad) < DemoConstants.turretLockThresholdRad) {
                locked = true;
                return;
            }

            double dt = stepSeconds();

            double angleAtCapture = turret.getAngleRad();
            if (DemoConstants.turretLatencyCompensation) {
                angleAtCapture -= turret.getVelocityRadPerSec()
                        * (latencySeconds + DemoConstants.turretExtraLatencySeconds);
            }

            // Estimación ABSOLUTA del ángulo del target — no se acumula.
            applyFiltered(angleAtCapture - txRad * DemoConstants.turretTxSign, dt);
        }

        /** Seguimiento por odometría: el ángulo ya viene limpio, sólo se suaviza. */
        void updateFromOdometry(double targetAngleRad) {
            double dt = stepSeconds();
            lastErrorRad = targetAngleRad - setpointRad;
            locked = false;
            applyFiltered(targetAngleRad, dt);
        }

        private void applyFiltered(double measured, double dt) {
            double previous = setpointRad;
            double filtered = initialized
                    ? previous + DemoConstants.turretTrackFilterAlpha * (measured - previous)
                    : measured;
            initialized = true;

            double maxStep = DemoConstants.turretMaxSetpointRateRadPerSec * dt;
            setpointRad = Turret.clampToLimits(
                    MathUtil.clamp(filtered, previous - maxStep, previous + maxStep));
        }

        /** Mueve el setpoint hacia un ángulo arbitrario respetando el slew. */
        void slewToward(double targetRad) {
            double dt = stepSeconds();
            double maxStep = DemoConstants.turretMaxSetpointRateRadPerSec * dt;
            setpointRad = Turret.clampToLimits(
                    MathUtil.clamp(targetRad, setpointRad - maxStep, setpointRad + maxStep));
            initialized = true;
            locked = false;
        }

        /**
         * Comanda la torreta sólo si el setpoint cambió lo suficiente.
         *
         * <p>
         * Cada setpoint nuevo reinicia el perfil de Motion Magic. Re-comandar a
         * 50 Hz con cambios de milésimas de grado hace que el perfil nunca se
         * complete y el mecanismo tiemble en sitio.
         */
        void apply(Turret turret) {
            if (Math.abs(setpointRad - lastCommandedRad) > DemoConstants.turretSetpointDeadbandRad) {
                turret.rotateToAngle(setpointRad);
                lastCommandedRad = setpointRad;
            }
        }

        boolean isLocked() {
            return locked;
        }

        double errorRad() {
            return lastErrorRad;
        }

        double setpoint() {
            return setpointRad;
        }
    }

    // ════════════════════════════════════════════════════════════════════════
    // Comando principal
    // ════════════════════════════════════════════════════════════════════════

    /**
     * Default command de la torreta.
     *
     * @param aimSupplier    APUNTAR: la torreta rastrea, los flywheels NO giran.
     * @param chargeSupplier CARGAR (el gatillo): rastrea y acelera a la solución
     *                       de tiro. Mientras se mantenga, el cañón queda listo.
     */
    public static Command demoTurretCmd(
            Turret turret,
            Vision vision,
            Drive drive,
            BooleanSupplier aimSupplier,
            BooleanSupplier chargeSupplier,
            Supplier<Pose2d> poseSupplier) {

        TurretTracker tracker = new TurretTracker();
        Timer graceTimer = new Timer();
        Timer sweepTimer = new Timer();

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
                        graceTimer.reset();
                        sweepTimer.stop();
                        sweepTimer.reset();
                        vision.setPriorityTagId(CAM, -1);

                        DemoDashboard.turretState = "INACTIVA";
                        DemoDashboard.aiming = false;
                        DemoDashboard.charging = false;
                        DemoDashboard.turretOnTarget = false;
                        DemoDashboard.targetVisible = false;
                        DemoDashboard.tagId = -1;
                        DemoDashboard.shotPower = "—";
                        DemoDashboard.distanceMeters = 0.0;
                        Logger.recordOutput("Demo/Turret/State", "IDLE");
                        return;
                    }

                    DemoDashboard.aiming = true;
                    DemoDashboard.charging = charging;
                    DemoDashboard.targetVisible = vision.hasTarget(CAM);
                    DemoDashboard.tagId = vision.getPrimaryTagId(CAM);

                    // Solución de tiro compartida con el cañón fijo, para que
                    // los dos hoods reciban el mismo número.
                    ShotSolution shot = ShotSolution.compute(
                            poseSupplier.get(),
                            drive.getFieldRelativeVelocity(),
                            FieldTracking.getActiveTarget());
                    shot.log();
                    double distance = shot.distanceMeters;

                    // ═══════════ APUNTADO ═══════════
                    if (DemoState.isBomber()) {
                        // En BOMBER la torreta NO apunta: se congela en cero y el
                        // chasis hace todo el trabajo. Todo lo de ese modo vive
                        // en DemoDriveCommands.
                        turret.holdZero();
                        tracker.reset(turret.getAngleRad());
                        vision.setPriorityTagId(CAM, -1);
                        DemoDashboard.turretState = DemoState.isSmoothDump()
                                ? "CERO (volcado libre)"
                                : "CERO (chasis apunta)";
                        DemoDashboard.turretOnTarget = turret.isAtAngle(
                                0.0, DemoConstants.turretOnTargetToleranceRad);
                        Logger.recordOutput("Demo/Turret/State", "HOLD_ZERO");

                    } else {
                        // STRIKER — una sola máquina de estados, para HUB y para
                        // caza libre. La diferencia entre ambos la hace
                        // DemoState.acceptsTag(): en HUB sólo acepta tags del
                        // HUB, en caza libre acepta cualquiera.
                        trackTargets(turret, vision, tracker, graceTimer, sweepTimer, poseSupplier);
                        if (!DemoState.isHubTargeting()) {
                            distance = visionDistance(vision);
                        }
                    }

                    DemoDashboard.distanceMeters = distance;
                    Logger.recordOutput("Demo/ShotDistanceMeters", distance);
                    Logger.recordOutput("Demo/Turret/SetpointRad", tracker.setpoint());
                    Logger.recordOutput("Demo/Turret/Locked", tracker.isLocked());

                    // ═══════════ CARGA ═══════════
                    if (!charging) {
                        turret.stopFlywheel();
                        turret.setHoodAtInitialPosition();
                        DemoDashboard.shotPower = "—";
                        return;
                    }

                    if (DemoState.isSmoothDump()) {
                        // Volcado suave: hood plano y potencia fija, sin mapas.
                        turret.setHoodPosition(
                                Rotation2d.fromDegrees(DemoConstants.dumpHoodDegrees));
                        turret.setFlywheelVelocity(
                                Units.rotationsToRadians(DemoConstants.dumpFlywheelRPS()));
                        DemoDashboard.shotPower = String.format(
                                "VOLCADO %.0f%%", DemoConstants.dumpPowerFraction * 100.0);
                        Logger.recordOutput("Demo/ShotPower", "DUMP");

                    } else if (DemoState.allowsCompetitionPower()) {
                        // Los mapas de competencia, tal cual.
                        turret.setHoodForDistance(distance);
                        turret.setFlywheelVelocityForDistance(distance);
                        DemoDashboard.shotPower = "COMPETENCIA";
                        Logger.recordOutput("Demo/ShotPower", "COMPETITION");

                    } else {
                        // Mapa de tiro suave, tuneable por puntos.
                        turret.setHoodFromMap(DemoConstants.kGentleHoodMap, distance);
                        turret.setFlywheelVelocityFromMap(
                                DemoConstants.kGentleFlywheelMap,
                                distance,
                                DemoConstants.gentleMaxFlywheelRPS);
                        DemoDashboard.shotPower = "SUAVE";
                        Logger.recordOutput("Demo/ShotPower", "GENTLE");
                    }
                },
                turret)
                .beforeStarting(() -> {
                    tracker.reset(turret.getAngleRad());
                    graceTimer.restart();
                    sweepTimer.reset();
                });
    }

    /**
     * Máquina de estados del rastreo de STRIKER.
     *
     * <p>
     * Cuatro niveles, en orden: visión → gracia → odometría → barrido. Ver el
     * javadoc de la clase.
     */
    private static void trackTargets(
            Turret turret,
            Vision vision,
            TurretTracker tracker,
            Timer graceTimer,
            Timer sweepTimer,
            Supplier<Pose2d> poseSupplier) {

        int locked = DemoState.getLockedTagId();
        if (DemoConstants.useLimelightPriorityId) {
            vision.setPriorityTagId(CAM, locked);
        }

        boolean sees = vision.hasTarget(CAM)
                && DemoState.acceptsTag(vision.getPrimaryTagId(CAM));

        // ── 1. VISIÓN ──────────────────────────────────────────────────────
        if (sees) {
            graceTimer.restart();
            sweepTimer.stop();
            sweepTimer.reset();

            double txRad = DemoConstants.useTxNoCrosshair
                    ? vision.getTargetXNoCrosshairRad(CAM)
                    : vision.getTargetXRad(CAM);

            tracker.updateFromVision(
                    turret, txRad, vision.getLatencySeconds(CAM), vision.getSampleTimestamp(CAM));
            tracker.apply(turret);

            DemoDashboard.turretState = tracker.isLocked() ? "ENGANCHADA" : "SIGUIENDO";
            DemoDashboard.turretOnTarget = tracker.isLocked();
            DemoDashboard.turretErrorDeg = Math.toDegrees(tracker.errorRad());
            Logger.recordOutput("Demo/Turret/State", tracker.isLocked() ? "LOCKED" : "TRACKING");
            return;
        }

        // ── 2. GRACIA ──────────────────────────────────────────────────────
        // Alguien se cruzó enfrente o taparon el tag un instante. Sin esto, la
        // torreta se iría a barrer cada vez que pasa alguien.
        if (!graceTimer.hasElapsed(DemoConstants.turretTargetGraceSeconds)) {
            tracker.apply(turret);
            DemoDashboard.turretState = "PERDIDA (esperando)";
            DemoDashboard.turretOnTarget = false;
            Logger.recordOutput("Demo/Turret/State", "GRACE");
            return;
        }

        DemoDashboard.turretOnTarget = false;

        // ── 3. ODOMETRÍA ───────────────────────────────────────────────────
        // Sólo en modo HUB, porque un tag suelto no está en el layout del campo
        // y la odometría no tiene forma de saber dónde está.
        //
        // La compuerta de pose fresca SÍ va aquí (a diferencia del chasis en
        // BOMBER): la torreta puede arrancar sola barriendo, así que exigir
        // confirmación no la bloquea. Al habilitar barre, encuentra el tag, la
        // visión corrige la pose, y a partir de ahí este nivel toma el relevo
        // cuando el tag sale de rango.
        if (DemoState.isHubTargeting() && FieldTracking.isOdometryValid()) {
            double angle = Turret.computeTurretAngleRad(
                    poseSupplier.get(), FieldTracking.getHubPosition());
            tracker.updateFromOdometry(angle);
            tracker.apply(turret);

            DemoDashboard.turretState = String.format(
                    "HUB por odometría (%.0fs)", FieldTracking.secondsSinceUpdate());
            DemoDashboard.turretOnTarget = turret.isAtAngle(
                    angle, DemoConstants.turretOnTargetToleranceRad);
            DemoDashboard.turretErrorDeg = Math.toDegrees(turret.getAngleRad() - angle);
            Logger.recordOutput("Demo/Turret/State", "ODOMETRY");
            Logger.recordOutput("Demo/Turret/TargetAngleRad", angle);
            return;
        }

        // ── 4. BARRIDO ─────────────────────────────────────────────────────
        if (!DemoConstants.searchSweepEnabled) {
            DemoDashboard.turretState = "SIN OBJETIVO";
            Logger.recordOutput("Demo/Turret/State", "NO_TARGET");
            return;
        }

        if (!sweepTimer.isRunning()) {
            sweepTimer.restart();
        }
        double t = sweepTimer.get();
        double sweepTarget = DemoConstants.turretSweepAmplitudeRad
                * Math.sin(2.0 * Math.PI * DemoConstants.turretSweepFrequencyHz * t);
        tracker.slewToward(sweepTarget);
        tracker.apply(turret);

        // Rotación de pipelines para adaptarse a la iluminación del lugar.
        int slot = (int) (t / DemoConstants.pipelineCycleSeconds);
        vision.setPipeline(
                CAM, DemoConstants.searchPipelines[slot % DemoConstants.searchPipelines.length]);

        DemoDashboard.turretState = "BUSCANDO";
        Logger.recordOutput("Demo/Turret/State", "SWEEPING");
    }

    /** Distancia al tag suelto. En caza libre es lo único que hay. */
    private static double visionDistance(Vision vision) {
        if (vision.hasTarget(CAM)) {
            double d = vision.getTargetDistanceMeters(CAM);
            if (d > 0.3 && d < DemoConstants.maxTrustedVisionDistanceMeters) {
                return d;
            }
        }
        return DemoConstants.fallbackDistanceMeters;
    }
}
