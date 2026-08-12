package frc.robot.commands.demo;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
 *   0. HUB CON POSE FRESCA
 *                       → apuntado CONTINUO: el setpoint sale de la odometría
 *                         cada 20 ms y la visión sólo corrige un sesgo lento.
 *                         Reemplaza a los niveles 1-3 mientras aplica, y es lo
 *                         que hace que la torreta siga el HUB sin rezagarse
 *                         mientras el robot maneja.
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
 * <p>
 * <b>Disparo en movimiento.</b> La corrección de {@code ShotSolution} entra como
 * un delta angular que se le suma a los niveles 1 y 3 — a los dos, con el mismo
 * número. Si viviera sólo en el nivel 3, la torreta pegaría un brinco cada vez
 * que el tag aparece o se pierde, porque una rama compensaría y la otra no.
 *
 * <p>
 * El orden importa y el nivel 4 es el que hace que el sistema arranque solo: al
 * habilitar, la pose todavía no está confirmada por visión, así que barre; en
 * cuanto encuentra el tag se engancha por visión y de paso la visión corrige la
 * pose; a partir de ahí toma el relevo el nivel 0, y perder el tag ya no
 * importa.
 *
 * <p>
 * Los niveles 1, 2 y 4 son el camino de la caza libre, donde un tag suelto no
 * está en el layout del campo y la odometría no puede ayudar. El nivel 3 sólo
 * corre con {@code turretContinuousOdometryAim = false}: con el nivel 0
 * encendido su compuerta es la misma y nunca se alcanza. Se conserva porque ese
 * flag es la salida de emergencia al comportamiento anterior.
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

        /** Sesgo aprendido visión − odometría. Ver {@link #updateBias}. */
        private double biasRad = 0.0;
        private boolean biasValid = false;

        /** Última estimación de visión, de qué tag, y de cuándo. Ver {@link #seedBias}. */
        private double lastVisionAngleRad = 0.0;
        private double lastVisionCaptureTimestamp = -1.0;
        private int lastVisionTagId = -1;

        void reset(double currentAngleRad) {
            setpointRad = currentAngleRad;
            lastCommandedRad = currentAngleRad;
            initialized = false;
            locked = false;
            lastErrorRad = 0.0;
            lastTimestamp = -1.0;
            lastSampleTimestamp = -1.0;
            biasRad = 0.0;
            biasValid = false;
            lastVisionAngleRad = 0.0;
            lastVisionCaptureTimestamp = -1.0;
            lastVisionTagId = -1;
        }

        /**
         * ¿Este frame de cámara es nuevo?
         *
         * <p>
         * Con la cámara a ~22 FPS y el código a 50 Hz, dos de cada tres lecturas
         * son el mismo frame repetido. Procesarlo otra vez aplicaría la
         * compensación de latencia sobre un ángulo de torreta que sí cambió.
         */
        boolean isNewSample(double sampleTimestamp) {
            if (sampleTimestamp == lastSampleTimestamp) {
                return false;
            }
            lastSampleTimestamp = sampleTimestamp;
            return true;
        }

        /**
         * Ángulo ABSOLUTO del target según la cámara, en el marco del mecanismo.
         *
         * <p>
         * Es dónde tendría que estar la torreta para tener el target centrado.
         * No se acumula: cada frame produce una estimación completa.
         */
        double visionAngleRad(
                Turret turret,
                double txRad,
                double latencySeconds,
                double captureTimestamp,
                int tagId) {
            double angleAtCapture = turret.getAngleRad();
            if (DemoConstants.turretLatencyCompensation) {
                angleAtCapture -= turret.getVelocityRadPerSec()
                        * (latencySeconds + DemoConstants.turretExtraLatencySeconds);
            }
            double angle = angleAtCapture - txRad * DemoConstants.turretTxSign;

            // Se guarda —con su tag y su instante de captura— para poder sembrar
            // el sesgo al entrar al apuntado continuo. Ver seedBias().
            lastVisionAngleRad = angle;
            lastVisionCaptureTimestamp = captureTimestamp;
            lastVisionTagId = tagId;
            return angle;
        }

        /** ¿Hay una medición de visión lo bastante reciente para sembrar? */
        boolean canSeedBias() {
            return !biasValid
                    && lastVisionCaptureTimestamp >= 0.0
                    && tagUsableForBias(lastVisionTagId)
                    && Timer.getFPGATimestamp() - lastVisionCaptureTimestamp
                            <= DemoConstants.turretBiasSeedMaxAgeSeconds;
        }

        double lastVisionCaptureTimestamp() {
            return lastVisionCaptureTimestamp;
        }

        /**
         * Siembra el sesgo con la última medición de visión, para que el traspaso
         * al apuntado continuo no salte.
         *
         * <p>
         * Sin esto, el sesgo empieza en cero y el setpoint se va al ángulo de
         * odometría <i>cruda</i> — un brinco del tamaño del error de pose, justo
         * cuando la torreta acababa de enganchar. Y esperar al primer frame no
         * alcanza: se entra al continuo en el ciclo en que la visión corrige la
         * pose, y ese frame ya lo consumió el nivel de visión, así que a 22 FPS
         * contra 50 Hz lo más probable es que el primer ciclo del continuo vea un
         * frame repetido.
         *
         * <p>
         * <b>Se siembra con la medición de visión, no con el setpoint.</b> El
         * setpoint parece más cómodo y es una trampa: puede venir del barrido, de
         * {@code holdZero}, o de un tag suelto de la caza libre, y en esos casos
         * {@code setpoint − odometría} no es error de pose sino basura — que se
         * quedaría congelada hasta 15° si el tag no vuelve a aparecer. El
         * setpoint además ya trae la compensación de movimiento adentro, así que
         * sembrar con él la contaría dos veces.
         *
         * <p>
         * Por la misma razón {@link #canSeedBias()} exige que la medición sea de
         * un tag <b>del HUB</b> y reciente: veniendo de la caza libre, la última
         * medición apunta a un tag que alguien traía en la mano, y el sesgo
         * contra el HUB sería basura pura.
         *
         * @param odometryAngleRad Ángulo por odometría <b>en el instante de
         *                         captura</b> de esa medición, no el de ahora.
         */
        void seedBias(double odometryAngleRad) {
            if (!canSeedBias()) {
                return;
            }
            biasRad = MathUtil.clamp(
                    MathUtil.angleModulus(lastVisionAngleRad - odometryAngleRad),
                    -DemoConstants.turretMaxBiasRad,
                    DemoConstants.turretMaxBiasRad);
            biasValid = true;
        }

        /**
         * Aprende cuánto se equivoca la odometría, y devuelve el sesgo a aplicar.
         *
         * <p>
         * <b>Los dos ángulos tienen que ser al objetivo REAL</b>, sin la
         * compensación de disparo en movimiento. Si el de odometría la trajera y
         * el de visión no, el sesgo se la comería como si fuera error de pose y
         * la cancelaría — la compensación existiría en el log y no en el
         * mecanismo.
         *
         * @param visionAngleRad   Estimación de la cámara, o {@code null} si no
         *                         hay frame nuevo utilizable.
         * @param odometryAngleRad Ángulo por odometría al objetivo real, en el
         *                         mismo instante que la medición de visión.
         */
        double updateBias(Double visionAngleRad, double odometryAngleRad) {
            if (visionAngleRad != null) {
                double instant = MathUtil.angleModulus(visionAngleRad - odometryAngleRad);
                biasRad = biasValid
                        ? biasRad + DemoConstants.turretBiasFilterAlpha * (instant - biasRad)
                        : instant;
                biasRad = MathUtil.clamp(
                        biasRad, -DemoConstants.turretMaxBiasRad, DemoConstants.turretMaxBiasRad);
                biasValid = true;
            }
            return biasValid ? biasRad : 0.0;
        }

        boolean isBiasValid() {
            return biasValid;
        }

        double bias() {
            return biasValid ? biasRad : 0.0;
        }

        /**
         * Apuntado continuo: el setpoint sigue al ángulo dado, cada ciclo.
         *
         * <p>
         * Sin histéresis y con filtro suave, porque aquí no hay ruido que
         * filtrar: el ángulo viene de la pose, que es una función suave del
         * tiempo. La histéresis existía para no perseguir el ruido de píxel de
         * {@code tx}, y es lo que hacía que la torreta se moviera a brincos y se
         * quedara rezagada al manejar.
         */
        void trackContinuous(double targetAngleRad) {
            double dt = stepSeconds();
            lastErrorRad = targetAngleRad - setpointRad;
            locked = false;
            applyFiltered(targetAngleRad, dt, DemoConstants.turretContinuousFilterAlpha);
        }

        private double stepSeconds() {
            double now = Timer.getFPGATimestamp();
            double dt = (lastTimestamp < 0.0) ? 0.02 : now - lastTimestamp;
            lastTimestamp = now;
            return MathUtil.clamp(dt, 0.001, 0.1);
        }

        /**
         * Seguimiento por visión: {@code tx} es el error de apuntado directo.
         *
         * @param aimOffsetRad Corrección de disparo en movimiento, en radianes
         *                     CCW. Se le suma al ángulo absoluto estimado del
         *                     target, no a {@code tx}, porque {@code tx} es una
         *                     medición y esto es una decisión — mezclarlos haría
         *                     que la histéresis midiera el error equivocado.
         */
        void updateFromVision(
                Turret turret,
                double txRad,
                double latencySeconds,
                double sampleTimestamp,
                int tagId,
                double aimOffsetRad) {
            if (!isNewSample(sampleTimestamp)) {
                return;
            }

            // Se calcula ANTES de la histéresis, aunque enganchada no se use:
            // así queda registrada la última medición de visión aunque la
            // torreta esté congelada, que es lo que le permite al apuntado
            // continuo sembrar su sesgo al tomar el relevo.
            double measuredAngleRad = visionAngleRad(
                    turret, txRad, latencySeconds, sampleTimestamp - latencySeconds, tagId);

            // Error de apuntado REAL, en el marco del mecanismo: cuánto le falta
            // girar a la torreta para quedar donde queremos que quede, positivo
            // en sentido CCW. Con la compensación apagada es el tx de siempre
            // con el signo volteado, porque tx vive en el marco de la cámara y
            // esto vive en el del mecanismo. Su magnitud —lo único que mira la
            // histéresis— es idéntica.
            //
            // La histéresis tiene que evaluar esto y no |tx|. Si evaluara |tx|,
            // apuntar al centro del tag contaría como "enganchada" y congelaría
            // el setpoint justo cuando la compensación pide estar corrido unos
            // grados — la torreta se quedaría sin compensar precisamente al
            // disparar.
            double effectiveErrorRad = aimOffsetRad - txRad * DemoConstants.turretTxSign;
            lastErrorRad = effectiveErrorRad;

            // ── Histéresis ────────────────────────────────────────────────
            if (locked) {
                if (Math.abs(effectiveErrorRad) < DemoConstants.turretUnlockThresholdRad) {
                    return; // Enganchada y quieta. No se comanda nada.
                }
                locked = false;
            } else if (Math.abs(effectiveErrorRad) < DemoConstants.turretLockThresholdRad) {
                locked = true;
                return;
            }

            // Estimación ABSOLUTA del ángulo del target — no se acumula.
            applyFiltered(
                    measuredAngleRad + aimOffsetRad,
                    stepSeconds(),
                    DemoConstants.turretTrackFilterAlpha);
        }

        /** Seguimiento por odometría: el ángulo ya viene limpio, sólo se suaviza. */
        void updateFromOdometry(double targetAngleRad) {
            double dt = stepSeconds();
            lastErrorRad = targetAngleRad - setpointRad;
            locked = false;
            applyFiltered(targetAngleRad, dt, DemoConstants.turretTrackFilterAlpha);
        }

        private void applyFiltered(double measured, double dt, double alpha) {
            double previous = setpointRad;
            double filtered = initialized ? previous + alpha * (measured - previous) : measured;
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
            apply(turret, DemoConstants.turretSetpointDeadbandRad);
        }

        void apply(Turret turret, double deadbandRad) {
            if (Math.abs(setpointRad - lastCommandedRad) > deadbandRad) {
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
                        DemoDashboard.shotAimOffsetDeg = 0.0;
                        DemoDashboard.shotCompensationMeters = 0.0;
                        DemoDashboard.turretBiasDeg = 0.0;
                        DemoDashboard.fieldSpeedMetersPerSec = 0.0;
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
                    DemoDashboard.fieldSpeedMetersPerSec =
                            drive.getFieldRelativeVelocity().getNorm();
                    double distance = shot.distanceMeters;

                    // La compensación sólo tiene sentido apuntándole al HUB: es
                    // el único objetivo cuya posición de campo conocemos. En
                    // caza libre el "objetivo" de la solución es el HUB, que no
                    // es a donde está mirando la torreta, así que su corrección
                    // no aplica.
                    boolean compensates = DemoState.isHubTargeting();
                    double aimOffsetRad = compensates ? shot.aimOffsetRad : 0.0;

                    // El dashboard muestra lo que se APLICA, no lo que se
                    // calculó. En caza libre son distintos a propósito, y ver el
                    // número calculado ahí sería mentirle al piloto. Quién lo
                    // aplica cambia según el modo: en STRIKER la torreta, en
                    // BOMBER el chasis.
                    DemoDashboard.shotAimOffsetDeg = Math.toDegrees(aimOffsetRad);
                    DemoDashboard.shotCompensationMeters =
                            compensates ? shot.compensationMeters : 0.0;

                    // La torreta sólo compensa en STRIKER. En BOMBER se queda
                    // congelada en cero y quien apunta es el chasis.
                    double turretAimOffsetRad = DemoState.isStriker() ? aimOffsetRad : 0.0;

                    // Se limpia cada ciclo y sólo el apuntado continuo lo llena,
                    // para que el widget no se quede con un valor rancio de hace
                    // rato cuando el apuntado sale de otro lado.
                    DemoDashboard.turretBiasDeg = 0.0;

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
                        trackTargets(
                                turret, vision, drive, tracker, graceTimer, sweepTimer,
                                poseSupplier, turretAimOffsetRad);
                        if (!DemoState.isHubTargeting()) {
                            distance = visionDistance(vision);
                        }
                    }

                    DemoDashboard.distanceMeters = distance;
                    Logger.recordOutput("Demo/ShotDistanceMeters", distance);
                    Logger.recordOutput("Demo/Turret/SetpointRad", tracker.setpoint());
                    Logger.recordOutput("Demo/Turret/Locked", tracker.isLocked());
                    Logger.recordOutput(
                            "Demo/Turret/AimOffsetDeg", Math.toDegrees(turretAimOffsetRad));

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
            Drive drive,
            TurretTracker tracker,
            Timer graceTimer,
            Timer sweepTimer,
            Supplier<Pose2d> poseSupplier,
            double aimOffsetRad) {

        int locked = DemoState.getLockedTagId();
        if (DemoConstants.useLimelightPriorityId) {
            vision.setPriorityTagId(CAM, locked);
        }

        boolean sees = vision.hasTarget(CAM)
                && DemoState.acceptsTag(vision.getPrimaryTagId(CAM));

        // ── 0. APUNTADO CONTINUO POR ODOMETRÍA ─────────────────────────────
        // Cuando le apuntamos al HUB y la pose es confiable, éste reemplaza a
        // los niveles 1 a 3: el setpoint sale de la odometría cada 20 ms y la
        // visión sólo corrige el sesgo. No hay traspaso que hacer al perder el
        // tag porque nunca dejamos de apuntar por odometría.
        if (DemoConstants.turretContinuousOdometryAim
                && DemoState.isHubTargeting()
                && FieldTracking.isOdometryValid()) {
            trackContinuous(turret, vision, drive, tracker, graceTimer, sweepTimer,
                    poseSupplier, aimOffsetRad, sees);
            return;
        }

        // ── 1. VISIÓN ──────────────────────────────────────────────────────
        if (sees) {
            graceTimer.restart();
            sweepTimer.stop();
            sweepTimer.reset();

            double txRad = DemoConstants.useTxNoCrosshair
                    ? vision.getTargetXNoCrosshairRad(CAM)
                    : vision.getTargetXRad(CAM);

            tracker.updateFromVision(
                    turret,
                    txRad,
                    vision.getLatencySeconds(CAM),
                    vision.getSampleTimestamp(CAM),
                    vision.getPrimaryTagId(CAM),
                    aimOffsetRad);
            tracker.apply(turret);

            DemoDashboard.turretState = tracker.isLocked() ? "ENGANCHADA" : "SIGUIENDO";

            // El enganche no alcanza como criterio de "apuntada" cuando hay
            // compensación de movimiento: la torreta se queda a propósito unos
            // grados fuera del centro del tag, así que la histéresis nunca
            // engancha y el indicador de LISTO PARA TIRAR se apagaría justo al
            // disparar en movimiento. Lo que importa es si el mecanismo llegó a
            // donde se le pidió.
            DemoDashboard.turretOnTarget = tracker.isLocked()
                    || turret.isAtAngle(tracker.setpoint(), DemoConstants.turretOnTargetToleranceRad);
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
            // MISMA corrección que se le suma a tx en la rama de visión. Que las
            // dos ramas apliquen el mismo número es lo que hace que el traspaso
            // no se note cuando el tag aparece o se pierde.
            //
            // Va como argumento y no sumada al resultado porque el método
            // envuelve y recorta el ángulo: sumarla después la perdería contra
            // el soft limit cuando el objetivo cae cerca de la costura.
            double angle = Turret.computeTurretAngleRad(
                    poseSupplier.get(), FieldTracking.getHubPosition(), aimOffsetRad);
            tracker.updateFromOdometry(angle);
            tracker.apply(turret);

            DemoDashboard.turretState = String.format(
                    "HUB por odometría (%.0fs)", FieldTracking.secondsSinceUpdate());
            DemoDashboard.turretOnTarget = turret.isAtAngle(
                    angle, DemoConstants.turretOnTargetToleranceRad);
            // Misma convención que la rama de visión: cuánto le FALTA girar.
            // Antes esta rama reportaba el error al revés que la otra y el
            // número cambiaba de signo solo al perder el tag.
            DemoDashboard.turretErrorDeg = Math.toDegrees(angle - turret.getAngleRad());
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

    /**
     * Apuntado continuo: odometría de base, visión como corrección de sesgo.
     *
     * <h2>Por qué existe</h2>
     *
     * Cerrar el lazo sobre {@code tx} hace que el apuntado herede los defectos
     * de la cámara: llega a ~22 FPS y describe dónde estaba el target hace 45
     * ms. Con la histéresis encima, la torreta se quedaba <b>5-6° rezagada</b>
     * mientras el robot maneja, moviéndose a brincos del tamaño del deadband.
     * Rezagada hacia el lado del que vienes, que desde afuera se ve igualito a
     * una compensación con el signo al revés.
     *
     * <p>
     * Aquí el setpoint sale de la pose, que se actualiza cada 20 ms y no tiene
     * latencia de cámara. La visión aporta un <b>sesgo</b> filtrado lento que
     * absorbe el error de la pose y el de la calibración de la cámara:
     *
     * <pre>
     *   sesgo    = filtro(ánguloVisión − ánguloOdometría)
     *   setpoint = ánguloOdometría(objetivo, sesgo + compensación)
     * </pre>
     *
     * <p>
     * El seguimiento rápido lo hace la odometría, que no tiene ruido; el sesgo
     * sólo corrige un error sistemático, que cambia despacio y por eso se puede
     * filtrar fuerte. Es exactamente lo que hace {@code HubAlignment} para el
     * chasis, y lo que el javadoc de {@code FieldTracking} describe como el
     * diseño correcto.
     *
     * <p>
     * Los dos ángulos se comparan <b>sin</b> la compensación de movimiento y
     * ésta se suma al final, junto con el sesgo, antes del envolvimiento. Si el
     * sesgo se calculara contra el ángulo ya compensado, se comería la
     * compensación creyendo que es error de pose.
     */
    private static void trackContinuous(
            Turret turret,
            Vision vision,
            Drive drive,
            TurretTracker tracker,
            Timer graceTimer,
            Timer sweepTimer,
            Supplier<Pose2d> poseSupplier,
            double aimOffsetRad,
            boolean sees) {

        sweepTimer.stop();
        sweepTimer.reset();
        if (sees) {
            graceTimer.restart();
        }

        Pose2d pose = poseSupplier.get();
        Translation2d hub = FieldTracking.getHubPosition();
        double odometryAngleRad = Turret.computeTurretAngleRad(pose, hub);

        // Pegados al HUB, el rumbo deja de significar algo: unos centímetros de
        // error de pose son decenas de grados de apuntado, y sin la histéresis
        // que antes lo amortiguaba la torreta perseguiría ese ruido a toda
        // velocidad. Mejor congelar el último ángulo bueno.
        if (pose.getTranslation().getDistance(hub) < DemoConstants.turretAimMinDistanceMeters) {
            tracker.apply(turret, DemoConstants.turretContinuousDeadbandRad);
            DemoDashboard.turretState = "HUB continuo · muy cerca";
            DemoDashboard.turretOnTarget = turret.isAtAngle(
                    tracker.setpoint(), DemoConstants.turretOnTargetToleranceRad);
            DemoDashboard.turretErrorDeg = Math.toDegrees(
                    tracker.setpoint() - turret.getAngleRad());
            DemoDashboard.turretBiasDeg = Math.toDegrees(tracker.bias());
            Logger.recordOutput("Demo/Turret/State", "CONTINUOUS_TOO_CLOSE");
            Logger.recordOutput("Demo/Turret/BiasDeg", Math.toDegrees(tracker.bias()));
            Logger.recordOutput("Demo/Turret/BiasValid", tracker.isBiasValid());
            return;
        }

        // Sólo se le da de comer al sesgo con frames nuevos. Repetir el mismo
        // frame lo sesgaría hacia esa lectura al triple de velocidad.
        Double visionAngleRad = null;
        double biasReferenceRad = odometryAngleRad;
        double sampleTimestamp = vision.getSampleTimestamp(CAM);
        int tagId = vision.getPrimaryTagId(CAM);
        if (sees && tagUsableForBias(tagId) && tracker.isNewSample(sampleTimestamp)) {
            double txRad = DemoConstants.useTxNoCrosshair
                    ? vision.getTargetXNoCrosshairRad(CAM)
                    : vision.getTargetXRad(CAM);
            double latencySeconds = vision.getLatencySeconds(CAM);
            double captureTimestamp = sampleTimestamp - latencySeconds;
            visionAngleRad = tracker.visionAngleRad(
                    turret, txRad, latencySeconds, captureTimestamp, tagId);
            biasReferenceRad = odometryAngleAt(drive, captureTimestamp, hub, odometryAngleRad);

        } else if (tracker.canSeedBias()) {
            // Sin frame nuevo y sin sesgo todavía: se siembra con la última
            // medición buena para que el traspaso no salte. Va DESPUÉS del
            // bloque de arriba a propósito — si hubiera frame nuevo, sembrar
            // primero haría que esa medición entrara filtrada al 5% en vez de
            // fijar el sesgo de golpe.
            tracker.seedBias(odometryAngleAt(
                    drive, tracker.lastVisionCaptureTimestamp(), hub, odometryAngleRad));
        }

        double biasRad = tracker.updateBias(visionAngleRad, biasReferenceRad);

        // La compensación y el sesgo entran ANTES del envolvimiento, por eso van
        // como argumento y no sumados al resultado.
        double targetRad = Turret.computeTurretAngleRad(pose, hub, aimOffsetRad + biasRad);

        tracker.trackContinuous(targetRad);
        tracker.apply(turret, DemoConstants.turretContinuousDeadbandRad);

        DemoDashboard.turretState = sees
                ? (tracker.isBiasValid() ? "HUB continuo · visión" : "HUB continuo · calibrando")
                : String.format("HUB continuo · odometría (%.0fs)",
                        FieldTracking.secondsSinceUpdate());
        DemoDashboard.turretOnTarget = turret.isAtAngle(
                targetRad, DemoConstants.turretOnTargetToleranceRad);
        DemoDashboard.turretErrorDeg = Math.toDegrees(targetRad - turret.getAngleRad());
        DemoDashboard.turretBiasDeg = Math.toDegrees(biasRad);

        Logger.recordOutput("Demo/Turret/State", sees ? "CONTINUOUS_VISION" : "CONTINUOUS_ODOMETRY");
        Logger.recordOutput("Demo/Turret/TargetAngleRad", targetRad);
        Logger.recordOutput("Demo/Turret/OdometryAngleRad", odometryAngleRad);
        Logger.recordOutput("Demo/Turret/BiasDeg", Math.toDegrees(biasRad));
        Logger.recordOutput("Demo/Turret/BiasValid", tracker.isBiasValid());
    }

    /**
     * ¿Esta tag sirve para aprender el sesgo?
     *
     * <p>
     * Sirve la del centro de la cara, porque apuntarle a ella es apuntarle al
     * eje del HUB. La de la izquierda no: está corrida ~35 cm, y esa diferencia
     * es geometría, no error de pose. Ver
     * {@code turretBiasRequiresCenterTag}.
     */
    private static boolean tagUsableForBias(int tagId) {
        return DemoConstants.turretBiasRequiresCenterTag
                ? DemoState.isHubCenterTag(tagId)
                : DemoState.isHubTag(tagId);
    }

    /**
     * Ángulo de torreta al objetivo según la pose de un instante PASADO.
     *
     * <p>
     * El sesgo compara una medición de cámara contra la odometría, y las dos
     * tienen que describir el mismo instante. El frame habla del mundo de hace
     * ~45 ms; restarle la pose de ahora metería ese retraso adentro del sesgo, y
     * manejando rápido eso vuelve a ser rezago — el mismo problema que este
     * nivel arregla, en chiquito.
     *
     * @param fallbackRad Qué usar si el instante ya salió del buffer del pose
     *                    estimator.
     */
    private static double odometryAngleAt(
            Drive drive, double timestampSeconds, Translation2d target, double fallbackRad) {
        var sample = drive.samplePoseAt(timestampSeconds);

        // Si el buffer no tenía ese instante, el sesgo de este ciclo vuelve a
        // mezclar tiempos. Se registra porque si pasa seguido, el sesgo está
        // aprendiendo con basura y no hay otra forma de darse cuenta.
        Logger.recordOutput("Demo/Turret/PoseSampleMissed", sample.isEmpty());
        return sample.map(past -> Turret.computeTurretAngleRad(past, target))
                .orElse(fallbackRad);
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
