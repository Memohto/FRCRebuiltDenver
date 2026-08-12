package frc.robot.commands.demo;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.util.SmoothDriveFilter;
import frc.robot.util.SmoothRateLimiter;

/**
 * Comandos de manejo del Demo Mode.
 *
 * <h2>Dos orientaciones de manejo (toggle con X en el piloto)</h2>
 *
 * <ul>
 * <li><b>FIELD</b> — igual que competencia. Field-relative usando la rotación de
 * la odometría, con flip por alianza. El "adelante" del stick es el adelante del
 * campo, y la visión mantiene esa referencia correcta.</li>
 *
 * <li><b>DRIVER</b> — el piloto fija su propio frente con B, y ese frente es
 * <b>inmune</b> a las correcciones de visión.</li>
 * </ul>
 *
 * <p>
 * El modo DRIVER existe por un bug concreto: antes B hacía
 * {@code drive.setPose()}, que reescribe la rotación de la odometría. Un segundo
 * después llegaba una corrección de visión, devolvía la pose a la verdad del
 * campo, y se llevaba el frente que el piloto acababa de fijar. Ahora B guarda un
 * <b>offset</b> en vez de reescribir la pose: la visión sigue corrigiendo la
 * odometría (que es lo que el apuntado necesita) y el frente del piloto no se
 * mueve.
 */
public class DemoDriveCommands {

    /** Índice de la Limelight de la torreta en el arreglo de Vision. */
    public static final int TURRET_CAMERA = 0;

    private DemoDriveCommands() {
    }

    // ════════════════════════════════════════════════════════════════════════
    // Utilidades
    // ════════════════════════════════════════════════════════════════════════

    /**
     * Convierte los dos ejes del stick en un vector normalizado.
     *
     * <p>
     * El deadband se aplica sobre la MAGNITUD, no sobre cada eje. Aplicarlo por
     * eje deja una zona muerta en forma de cruz y el robot no se mueve en
     * diagonal hasta que ambos ejes superan el umbral.
     */
    private static Translation2d joystickToVector(double x, double y) {
        double magnitude = MathUtil.applyDeadband(Math.hypot(x, y), DemoConstants.joystickDeadband);
        if (magnitude < 1.0e-6) {
            return Translation2d.kZero;
        }
        Rotation2d direction = new Rotation2d(Math.atan2(y, x));
        return new Translation2d(Math.pow(magnitude, DemoConstants.joystickExponent), direction);
    }

    private static double joystickToScalar(double value) {
        double out = MathUtil.applyDeadband(value, DemoConstants.joystickDeadband);
        return Math.copySign(Math.pow(Math.abs(out), DemoConstants.joystickExponent), out);
    }

    private static boolean isRedAlliance() {
        return DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
    }

    /**
     * Rotación de referencia para convertir el comando a robot-relativo.
     *
     * <p>
     * <b>FIELD</b> — lo mismo que hace {@code DriveCommands.joystickDrive} en
     * competencia: la rotación de la odometría, más 180° en alianza roja.
     *
     * <p>
     * <b>DRIVER</b> — la rotación de la odometría menos el offset que el piloto
     * fijó con B. Como los dos términos vienen del mismo pose estimator, una
     * corrección de visión los mueve a los dos por igual y la diferencia no
     * cambia: el frente del piloto sobrevive.
     */
    private static Rotation2d fieldReference(Drive drive) {
        Rotation2d current = drive.getRotation();
        if (DemoState.isDriverOriented()) {
            return current.minus(DemoState.getDriverFrontOffset());
        }
        return isRedAlliance() ? current.plus(Rotation2d.k180deg) : current;
    }

    // ════════════════════════════════════════════════════════════════════════
    // SMOOTH DRIVE — default command
    // ════════════════════════════════════════════════════════════════════════

    /**
     * Manejo suavizado con asistencia de rumbo en BOMBER.
     *
     * <p>
     * La asistencia usa <b>odometría</b>, igual que el modo ORBIT de
     * competencia: se calcula el rumbo del robot al HUB y se le suman 180°
     * porque los cañones apuntan hacia atrás. Lo único que cambia respecto a
     * competencia es el PID, que aquí es mucho más suave y no re-escala su
     * propia salida.
     *
     * @param assistSupplier Sólo asiste mientras el operador apunta o carga.
     */
    public static Command smoothDrive(
            Drive drive,
            Vision vision,
            Turret turret,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier,
            BooleanSupplier creepSupplier,
            BooleanSupplier assistSupplier) {

        SmoothDriveFilter translationFilter = new SmoothDriveFilter(
                DemoConstants.translationMaxAccel,
                DemoConstants.translationMaxDecel,
                DemoConstants.translationSmoothingTau);

        SmoothRateLimiter rotationFilter = new SmoothRateLimiter(
                DemoConstants.rotationMaxAccel,
                DemoConstants.rotationMaxDecel,
                DemoConstants.rotationSmoothingTau);

        HeadingController heading = new HeadingController();
        HubAlignment alignment = new HubAlignment();

        return Commands.run(
                () -> {
                    // ── Traslación ─────────────────────────────────────────
                    Translation2d desired = joystickToVector(
                            xSupplier.getAsDouble(), ySupplier.getAsDouble());
                    Translation2d filtered = translationFilter.calculate(desired);

                    double speedFraction = creepSupplier.getAsBoolean()
                            ? DemoConstants.creepSpeedFraction
                            : DemoConstants.maxSpeedFraction;
                    double maxMetersPerSec =
                            DemoConstants.measuredFreeSpeedMetersPerSec * speedFraction;

                    double vx = filtered.getX() * maxMetersPerSec;
                    double vy = filtered.getY() * maxMetersPerSec;

                    // ── Rotación ───────────────────────────────────────────
                    // El volcado suave NO corrige dirección por diseño: el
                    // piloto apunta con el robot y el código no calcula nada.
                    //
                    // OJO: aquí NO se exige odometría confirmada. Hacerlo creaba
                    // un bloqueo circular — ver requireFreshOdometryForAssist.
                    boolean headingAssisted = DemoState.isBomber()
                            && !DemoState.isFollowing()
                            && !DemoState.isSmoothDump()
                            && assistSupplier.getAsBoolean()
                            && (!DemoConstants.requireFreshOdometryForAssist
                                    || FieldTracking.isOdometryValid());

                    double omegaRadPerSec;
                    if (headingAssisted) {
                        Double error = alignment.computeErrorRad(drive, vision, turret);

                        if (error == null) {
                            // Ninguna fuente utilizable. Mejor devolverle el giro
                            // al piloto que girar hacia un rumbo inventado.
                            headingAssisted = false;
                            omegaRadPerSec = rotationFilter.calculate(
                                    joystickToScalar(omegaSupplier.getAsDouble()))
                                    * DemoConstants.maxAngularSpeedRadPerSec * speedFraction;
                            DemoDashboard.alignSource = "SIN FUENTE";
                        } else {
                            omegaRadPerSec = rotationFilter.calculate(
                                    heading.calculate(error) / DemoConstants.maxAngularSpeedRadPerSec)
                                    * DemoConstants.maxAngularSpeedRadPerSec;
                            Logger.recordOutput("Demo/Bomber/HeadingErrorDeg", Math.toDegrees(error));
                        }
                    } else {
                        double desiredOmega = joystickToScalar(omegaSupplier.getAsDouble());
                        omegaRadPerSec = rotationFilter.calculate(desiredOmega)
                                * DemoConstants.maxAngularSpeedRadPerSec * speedFraction;
                        heading.reset();
                        DemoDashboard.alignSource = DemoState.isSmoothDump()
                                ? "LIBRE (volcado)"
                                : DemoState.isBomber() ? "MANUAL" : "—";
                    }

                    Logger.recordOutput("Demo/Drive/RawMagnitude", desired.getNorm());
                    Logger.recordOutput("Demo/Drive/CommandMagnitude", translationFilter.getMagnitude());
                    Logger.recordOutput("Demo/Drive/CommandAccel", translationFilter.getAcceleration());
                    Logger.recordOutput("Demo/Drive/HeadingAssisted", headingAssisted);

                    DemoDashboard.speedFraction = speedFraction;
                    DemoDashboard.creepActive = creepSupplier.getAsBoolean();
                    DemoDashboard.headingAssisted = headingAssisted;

                    drive.runVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    new ChassisSpeeds(vx, vy, omegaRadPerSec),
                                    fieldReference(drive)));
                },
                drive)
                .beforeStarting(() -> {
                    translationFilter.reset();
                    rotationFilter.reset(0.0);
                    heading.reset();
                    alignment.reset();
                });
    }

    /**
     * Fusiona visión y odometría para alinear el chasis al HUB.
     *
     * <h2>El problema que resuelve</h2>
     *
     * Cerca del HUB la visión alinea perfecto. Al perder el tag, el error se
     * recalculaba desde la <b>posición estimada</b> del robot — y esa posición
     * nunca es exacta, sobre todo con una cámara en torreta cuya transformada
     * todavía no está medida. El salto entre las dos respuestas hacía que el
     * chasis pegara un tirón y se quedara persiguiendo un rumbo equivocado.
     *
     * <h2>Aprender el sesgo</h2>
     *
     * Mientras la visión funciona se mide cuánto se equivoca la odometría:
     *
     * <pre>
     *   sesgo = errorVisión − errorOdometría
     * </pre>
     *
     * y al perder el tag se usa {@code errorOdometría + sesgo}. El traspaso
     * queda continuo <b>por construcción</b>: en el instante exacto del cambio
     * las dos respuestas son idénticas, así que el chasis ni se entera. Y de
     * paso se corrige el error sistemático de la pose, que es lo que hacía que
     * el rumbo por odometría estuviera mal de entrada.
     *
     * <h2>Dos redes de seguridad</h2>
     *
     * <ul>
     * <li><b>Slew del rumbo objetivo.</b> Se limita qué tan rápido puede moverse
     * el rumbo al que el chasis apunta. Una pose basura de un solo ciclo ya no
     * puede mandar al robot a dar la vuelta.</li>
     *
     * <li><b>Distancia mínima.</b> Muy cerca del objetivo, centímetros de error
     * de posición se vuelven decenas de grados de error de rumbo. Por debajo del
     * umbral se congela el último rumbo bueno en vez de perseguir una dirección
     * que brinca.</li>
     * </ul>
     */
    private static class HubAlignment {
        private double biasRad = 0.0;
        private boolean biasValid = false;
        private double targetHeadingRad = Double.NaN;
        private double lastTimestamp = -1.0;

        void reset() {
            biasRad = 0.0;
            biasValid = false;
            targetHeadingRad = Double.NaN;
            lastTimestamp = -1.0;
        }

        /**
         * @return Error de rumbo a comandar, o {@code null} si no hay ninguna
         *         fuente utilizable.
         */
        Double computeErrorRad(Drive drive, Vision vision, Turret turret) {
            double now = Timer.getFPGATimestamp();
            double dt = (lastTimestamp < 0.0) ? 0.02 : now - lastTimestamp;
            lastTimestamp = now;
            dt = MathUtil.clamp(dt, 0.001, 0.1);

            double currentHeading = drive.getRotation().getRadians();

            // ── Fuente A: odometría ───────────────────────────────────────
            Double odometryError = null;
            Translation2d toTarget = FieldTracking.getActiveTarget()
                    .minus(drive.getPose().getTranslation());
            if (toTarget.getNorm() > DemoConstants.hubAlignMinPoseDistanceMeters) {
                double desired = toTarget.getAngle().plus(Rotation2d.k180deg).getRadians();
                odometryError = MathUtil.angleModulus(desired - currentHeading);
            }

            // ── Fuente B: visión ──────────────────────────────────────────
            Double visionError = null;
            int tag = vision.getPrimaryTagId(TURRET_CAMERA);
            if (DemoConstants.bomberUsesVisionWhenTagVisible
                    && vision.hasTarget(TURRET_CAMERA)
                    && DemoState.isHubTag(tag)) {
                // Con la torreta congelada en cero la cámara mira por donde
                // salen los cañones, así que tx ES el error de rumbo. Se incluye
                // el ángulo de torreta por si quedó unos grados corrida.
                visionError = turret.getAngleRad() - vision.getTargetXRad(TURRET_CAMERA);
            }

            // ── Disparo en movimiento ─────────────────────────────────────
            // El MISMO delta que usa la torreta. Se aplica después de la fusión,
            // no antes, para que el sesgo siga midiendo lo que debe medir: la
            // diferencia entre visión y odometría apuntando al objetivo REAL. Si
            // se lo sumáramos antes, la compensación entraría en las dos fuentes
            // y se cancelaría en la resta, pero ensuciaría el filtro del sesgo
            // cada vez que el robot acelera.
            // Gateado igual que en la torreta: la compensación se calcula contra
            // el HUB, así que sólo vale cuando al HUB le estamos apuntando.
            double aimOffsetRad = DemoState.isHubTargeting()
                    ? ShotSolution.compute(
                            drive.getPose(),
                            drive.getFieldRelativeVelocity(),
                            FieldTracking.getActiveTarget()).aimOffsetRad
                    : 0.0;
            Logger.recordOutput("Demo/Bomber/AimOffsetDeg", Math.toDegrees(aimOffsetRad));

            // ── Fusión ────────────────────────────────────────────────────
            double rawError;
            if (visionError != null) {
                if (odometryError != null) {
                    double instantBias = MathUtil.angleModulus(visionError - odometryError);
                    biasRad = biasValid
                            ? biasRad + DemoConstants.hubAlignBiasAlpha
                                    * MathUtil.angleModulus(instantBias - biasRad)
                            : instantBias;
                    biasValid = true;
                }
                rawError = visionError;
                DemoDashboard.alignSource = "VISION (tag " + tag + ")";
                Logger.recordOutput("Demo/Bomber/AlignSource", "VISION");

            } else if (odometryError != null) {
                rawError = odometryError + (biasValid ? biasRad : 0.0);
                DemoDashboard.alignSource = biasValid
                        ? "ODOMETRIA + sesgo"
                        : "ODOMETRIA (sin calibrar)";
                Logger.recordOutput("Demo/Bomber/AlignSource", "ODOMETRY");

            } else if (!Double.isNaN(targetHeadingRad)) {
                // Demasiado cerca del objetivo para confiar en el rumbo: se
                // mantiene el último bueno.
                DemoDashboard.alignSource = "RUMBO CONGELADO";
                Logger.recordOutput("Demo/Bomber/AlignSource", "HELD");
                return MathUtil.angleModulus(targetHeadingRad - currentHeading);

            } else {
                return null;
            }

            // ── Slew del rumbo objetivo ───────────────────────────────────
            double desiredTarget = currentHeading + rawError + aimOffsetRad;
            if (Double.isNaN(targetHeadingRad)) {
                targetHeadingRad = desiredTarget;
            } else {
                double step = MathUtil.angleModulus(desiredTarget - targetHeadingRad);
                double maxStep = DemoConstants.hubAlignMaxTargetRateRadPerSec * dt;
                targetHeadingRad += MathUtil.clamp(step, -maxStep, maxStep);
            }

            Logger.recordOutput("Demo/Bomber/BiasDeg", Math.toDegrees(biasRad));
            Logger.recordOutput("Demo/Bomber/BiasValid", biasValid);
            DemoDashboard.alignBiasDeg = Math.toDegrees(biasRad);

            return MathUtil.angleModulus(targetHeadingRad - currentHeading);
        }
    }

    /**
     * Controlador de rumbo suave, compartido por el orbit y el follow-me.
     *
     * <p>
     * Tres cosas lo hacen suave, y las tres faltaban en la versión de
     * competencia:
     *
     * <ol>
     * <li><b>La salida ya está en rad/s y no se re-escala.</b> Competencia
     * multiplica la salida del PID por {@code getMaxAngularSpeedRadPerSec()}
     * (~20 rad/s), lo que satura al instante y convierte el lazo en bang-bang.
     * Ése es el origen del "muy wobbly y errático".</li>
     *
     * <li><b>Compuerta suave en vez de corte.</b> Usar {@code atSetpoint()} para
     * apagar la salida de golpe es una discontinuidad: corrige, se pasa, vuelve
     * a corregir. Aquí la salida se escala de 0 a 1 a lo largo de una rampa.</li>
     *
     * <li><b>Derivativo sobre el error filtrado.</b> Un término D crudo amplifica
     * el ruido de la medición; filtrarlo primero es lo que permite subirlo lo
     * suficiente para amortiguar de verdad.</li>
     * </ol>
     */
    private static class HeadingController {
        private double lastError = 0.0;
        private double filteredDerivative = 0.0;
        private double lastTimestamp = -1.0;
        private boolean initialized = false;

        double calculate(double errorRad) {
            double now = Timer.getFPGATimestamp();
            double dt = (lastTimestamp < 0.0) ? 0.02 : now - lastTimestamp;
            lastTimestamp = now;
            dt = MathUtil.clamp(dt, 0.001, 0.1);

            double derivative = initialized ? (errorRad - lastError) / dt : 0.0;
            initialized = true;
            lastError = errorRad;

            // Filtro sobre el derivativo: sin esto, kD amplifica el ruido y hay
            // que dejarlo tan bajo que no amortigua nada.
            filteredDerivative += DemoConstants.headingDerivativeAlpha
                    * (derivative - filteredDerivative);

            double output = DemoConstants.headingKp * errorRad
                    + DemoConstants.headingKd * filteredDerivative;

            // Compuerta suave alrededor del setpoint.
            double gate = MathUtil.clamp(
                    (Math.abs(errorRad) - DemoConstants.headingToleranceRad)
                            / DemoConstants.headingSoftZoneRad,
                    0.0, 1.0);
            output *= gate;

            return MathUtil.clamp(
                    output,
                    -DemoConstants.headingMaxOmegaRadPerSec,
                    DemoConstants.headingMaxOmegaRadPerSec);
        }

        void reset() {
            lastError = 0.0;
            filteredDerivative = 0.0;
            lastTimestamp = -1.0;
            initialized = false;
        }
    }

    // ════════════════════════════════════════════════════════════════════════
    // FOLLOW-ME — ahora sólo el chasis
    // ════════════════════════════════════════════════════════════════════════

    /**
     * El robot sigue un AprilTag manteniendo distancia. <b>La torreta no se
     * mueve.</b>
     *
     * <h2>Qué cambió y por qué</h2>
     *
     * La versión anterior hacía que la torreta rastreara y el chasis persiguiera
     * el ángulo de la torreta ("la cabeza va primero"). Se veía bien en el papel
     * y en la práctica arrastraba el sentido de giro del mecanismo, que no se
     * puede deducir del código, y terminaba girando al revés hasta topar la
     * torreta.
     *
     * <p>
     * Ahora la torreta se queda congelada en cero y <b>todo el rastreo lo hace
     * el chasis</b>, exactamente como el orbit al HUB. Con la torreta en cero, la
     * cámara mira por donde salen los cañones, así que:
     *
     * <pre>
     *   error de rumbo  = −tx        (llevar tx a cero apunta la trasera al tag)
     *   error de dist.  = distancia medida − distancia objetivo
     * </pre>
     *
     * Sólo depende de {@code tx} y de la distancia. Ni pose 3D, ni convención de
     * yaw del tag, ni signo del mecanismo que adivinar. Y el rumbo usa el mismo
     * controlador suave que el orbit.
     *
     * <h2>Seguridad</h2>
     * <ul>
     * <li>Velocidad tope muy por debajo de la de manejo.</li>
     * <li>Piso duro de distancia: por debajo, sólo puede alejarse.</li>
     * <li>Sin ver el target, frena.</li>
     * <li>Cualquier movimiento del stick del piloto lo cancela al instante.</li>
     * </ul>
     */
    public static Command followTarget(
            Drive drive,
            Vision vision,
            Turret turret,
            DoubleSupplier driverX,
            DoubleSupplier driverY,
            DoubleSupplier driverOmega) {

        SmoothDriveFilter translationFilter = new SmoothDriveFilter(
                DemoConstants.translationMaxAccel * 0.5,
                DemoConstants.translationMaxDecel * 0.7,
                DemoConstants.translationSmoothingTau * 1.5);

        SmoothRateLimiter bodyFilter = new SmoothRateLimiter(
                DemoConstants.followBodyMaxAccel,
                DemoConstants.followBodyMaxDecel,
                DemoConstants.followBodySmoothingTau);

        HeadingController heading = new HeadingController();
        FollowState state = new FollowState();
        Timer lostTimer = new Timer();

        return Commands.run(
                () -> {
                    // La torreta se queda en cero — de eso se encarga el comando
                    // de torreta, que en BOMBER siempre la congela. Aquí la
                    // cabeza NO se mueve: todo el rastreo es del chasis.
                    boolean sees = vision.hasTarget(TURRET_CAMERA)
                            && DemoState.acceptsTag(vision.getPrimaryTagId(TURRET_CAMERA));

                    if (sees) {
                        state.update(
                                vision.getTargetXRad(TURRET_CAMERA),
                                vision.getTargetDistanceMeters(TURRET_CAMERA));
                        lostTimer.restart();
                    }

                    boolean lost = !state.valid
                            || lostTimer.hasElapsed(DemoConstants.followLostTargetTimeoutSeconds);

                    if (lost) {
                        Translation2d stopping = translationFilter.calculate(Translation2d.kZero);
                        double omega = bodyFilter.calculate(0.0);
                        drive.runVelocity(new ChassisSpeeds(
                                stopping.getX() * DemoConstants.followMaxSpeedMetersPerSec,
                                stopping.getY() * DemoConstants.followMaxSpeedMetersPerSec,
                                omega * DemoConstants.headingMaxOmegaRadPerSec));
                        DemoDashboard.followLocked = false;
                        Logger.recordOutput("Demo/Follow/Locked", false);
                        return;
                    }

                    // ── Rumbo: llevar tx a cero apunta la trasera al tag ───
                    double headingError = -state.tx;
                    double omegaCommand = heading.calculate(headingError);
                    double omega = bodyFilter.calculate(
                            omegaCommand / DemoConstants.headingMaxOmegaRadPerSec)
                            * DemoConstants.headingMaxOmegaRadPerSec;

                    // ── Distancia: acercarse o alejarse a lo largo de la línea
                    //    de vista, que en marco de robot está en π − tx ───────
                    double bearing = Math.PI - state.tx;
                    double distanceError = state.distance - DemoConstants.followDistanceMeters;

                    // Piso duro: por debajo de la distancia mínima sólo puede
                    // alejarse, nunca acercarse.
                    if (state.distance < DemoConstants.followMinSafeDistanceMeters) {
                        distanceError = Math.min(distanceError, 0.0);
                    }
                    if (Math.abs(distanceError) < DemoConstants.followPositionToleranceMeters) {
                        distanceError = 0.0;
                    }

                    double speed = MathUtil.clamp(
                            distanceError * DemoConstants.followTranslationKp,
                            -DemoConstants.followMaxSpeedMetersPerSec,
                            DemoConstants.followMaxSpeedMetersPerSec);

                    Translation2d velocity = new Translation2d(
                            speed * Math.cos(bearing), speed * Math.sin(bearing));
                    Translation2d filtered = translationFilter.calculate(
                            velocity.div(DemoConstants.followMaxSpeedMetersPerSec));

                    DemoDashboard.followLocked = true;
                    DemoDashboard.followDistanceMeters = state.distance;
                    Logger.recordOutput("Demo/Follow/Locked", true);
                    Logger.recordOutput("Demo/Follow/DistanceMeters", state.distance);
                    Logger.recordOutput("Demo/Follow/HeadingErrorDeg", Math.toDegrees(headingError));

                    // Robot-relativo a propósito: no se usa la odometría.
                    drive.runVelocity(new ChassisSpeeds(
                            filtered.getX() * DemoConstants.followMaxSpeedMetersPerSec,
                            filtered.getY() * DemoConstants.followMaxSpeedMetersPerSec,
                            omega));
                },
                drive)
                .beforeStarting(() -> {
                    translationFilter.reset();
                    bodyFilter.reset(0.0);
                    heading.reset();
                    state.reset();
                    lostTimer.restart();
                })
                .finallyDo(interrupted -> drive.stop())
                // El piloto siempre gana. Tocar el stick devuelve el control.
                .until(() -> {
                    double t = DemoConstants.followDriverOverrideThreshold;
                    return Math.hypot(driverX.getAsDouble(), driverY.getAsDouble()) > t
                            || Math.abs(driverOmega.getAsDouble()) > t;
                });
    }

    /** Estado filtrado del target. Sólo tx y distancia — nada más hace falta. */
    private static class FollowState {
        double tx = 0.0;
        double distance = 0.0;
        boolean valid = false;

        void reset() {
            tx = 0.0;
            distance = 0.0;
            valid = false;
        }

        void update(double newTx, double newDistance) {
            if (newDistance < 0.2) {
                return; // lectura inservible
            }
            double alpha = DemoConstants.followTargetFilterAlpha;
            if (!valid) {
                tx = newTx;
                distance = newDistance;
                valid = true;
                return;
            }
            tx += alpha * (newTx - tx);
            distance += alpha * (newDistance - distance);
        }
    }
}
