package frc.robot.commands.competition;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.CompetitionConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.CompetitionState;
import frc.robot.util.HeadingController;
import frc.robot.util.MatchDashboard;
import frc.robot.util.ShotSolution;
import frc.robot.util.SmoothDriveFilter;
import frc.robot.util.SmoothRateLimiter;

/**
 * Manejo de COMPETENCIA v2.
 *
 * <p>
 * Es el Smooth Drive del demo a velocidad de partido, con la asistencia de
 * rumbo de BOMBER hecha por <b>odometría</b>: el rumbo objetivo es el rumbo
 * del robot al objetivo activo más 180°, porque los dos cañones disparan
 * hacia atrás. Es exactamente lo que hacía el modo ORBIT de Denver, con dos
 * diferencias:
 *
 * <ul>
 * <li>El PID de rumbo no re-escala su salida (ver {@link HeadingController}).
 * Ése era el bug del "orbit muy violento".</li>
 * <li>La asistencia sólo actúa <b>mientras el operador apunta o carga</b>. Antes
 * cambiar a BOMBER le quitaba el giro al piloto de golpe sin que nadie lo
 * hubiera pedido.</li>
 * </ul>
 *
 * <p>
 * Siempre field-relative con flip por alianza y reset de giro con B, igual que
 * Denver. No hay orientación DRIVER: en cancha la referencia es el campo.
 */
public final class CompDriveCommands {

    private CompDriveCommands() {
    }

    // ── Utilidades ──────────────────────────────────────────────────────────

    /** Stick → vector normalizado. Deadband sobre la MAGNITUD, no por eje. */
    private static Translation2d joystickToVector(double x, double y) {
        double magnitude = MathUtil.applyDeadband(
                Math.hypot(x, y), CompetitionConstants.joystickDeadband);
        if (magnitude < 1.0e-6) {
            return Translation2d.kZero;
        }
        Rotation2d direction = new Rotation2d(Math.atan2(y, x));
        return new Translation2d(
                Math.pow(Math.min(magnitude, 1.0), CompetitionConstants.joystickExponent), direction);
    }

    private static double joystickToScalar(double value) {
        double out = MathUtil.applyDeadband(value, CompetitionConstants.joystickDeadband);
        return Math.copySign(Math.pow(Math.abs(out), CompetitionConstants.joystickExponent), out);
    }

    /** Rotación de referencia: odometría, más 180° en alianza roja. */
    private static Rotation2d fieldReference(Drive drive) {
        Rotation2d current = drive.getRotation();
        return CompetitionState.isRedAlliance() ? current.plus(Rotation2d.k180deg) : current;
    }

    // ── Default command ─────────────────────────────────────────────────────

    /**
     * Manejo field-relative suavizado con asistencia de rumbo en BOMBER.
     *
     * @param precisionSupplier LT del piloto: baja la velocidad.
     * @param assistSupplier    "El chasis debe apuntar" = el operador está
     *                          apuntando o cargando.
     */
    public static Command joystickDrive(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier,
            BooleanSupplier precisionSupplier,
            BooleanSupplier assistSupplier) {

        SmoothDriveFilter translationFilter = new SmoothDriveFilter(
                CompetitionConstants.translationMaxAccel,
                CompetitionConstants.translationMaxDecel,
                CompetitionConstants.translationSmoothingTau);

        SmoothRateLimiter rotationFilter = new SmoothRateLimiter(
                CompetitionConstants.rotationMaxAccel,
                CompetitionConstants.rotationMaxDecel,
                CompetitionConstants.rotationSmoothingTau);

        HeadingController heading = new HeadingController(
                CompetitionConstants.headingKp,
                CompetitionConstants.headingKd,
                CompetitionConstants.headingDerivativeAlpha,
                CompetitionConstants.headingToleranceRad,
                CompetitionConstants.headingSoftZoneRad,
                CompetitionConstants.headingMaxOmegaRadPerSec);

        TargetHeading targetHeading = new TargetHeading();

        return Commands.run(
                () -> {
                    // ── Traslación ─────────────────────────────────────────
                    Translation2d desired = joystickToVector(
                            xSupplier.getAsDouble(), ySupplier.getAsDouble());
                    Translation2d filtered = translationFilter.calculate(desired);

                    boolean precision = precisionSupplier.getAsBoolean();
                    double speedFraction = precision
                            ? CompetitionConstants.precisionSpeedFraction
                            : CompetitionConstants.maxSpeedFraction;
                    double maxMetersPerSec = drive.getMaxLinearSpeedMetersPerSec() * speedFraction;

                    double vx = filtered.getX() * maxMetersPerSec;
                    double vy = filtered.getY() * maxMetersPerSec;

                    // ── Rotación ───────────────────────────────────────────
                    double stickOmega = joystickToScalar(omegaSupplier.getAsDouble());

                    boolean driverOverride = CompetitionConstants.headingDriverOverride
                            && Math.abs(omegaSupplier.getAsDouble())
                                    > CompetitionConstants.headingOverrideThreshold;

                    boolean headingAssisted = CompetitionState.isBomber()
                            && assistSupplier.getAsBoolean()
                            && !driverOverride;

                    double omegaRadPerSec;
                    if (headingAssisted) {
                        Double error = targetHeading.computeErrorRad(drive);
                        if (error == null) {
                            // Demasiado cerca del objetivo y sin rumbo previo:
                            // mejor devolverle el giro al piloto.
                            headingAssisted = false;
                            omegaRadPerSec = manualOmega(rotationFilter, stickOmega, speedFraction);
                            MatchDashboard.alignSource = "SIN FUENTE";
                        } else {
                            // La salida del controlador ya está en rad/s. El
                            // filtro trabaja normalizado, así que se divide y
                            // se vuelve a multiplicar por el MISMO número: no
                            // hay re-escalado.
                            double max = CompetitionConstants.headingMaxOmegaRadPerSec;
                            omegaRadPerSec = rotationFilter.calculate(heading.calculate(error) / max) * max;
                            MatchDashboard.alignSource = "ODOMETRIA";
                            MatchDashboard.headingErrorDeg = Math.toDegrees(error);
                            Logger.recordOutput("Match/Drive/HeadingErrorDeg", Math.toDegrees(error));
                        }
                    } else {
                        omegaRadPerSec = manualOmega(rotationFilter, stickOmega, speedFraction);
                        heading.reset();
                        targetHeading.reset();
                        MatchDashboard.alignSource = CompetitionState.isBomber()
                                ? (driverOverride ? "MANUAL (piloto)" : "MANUAL")
                                : "—";
                        MatchDashboard.headingErrorDeg = 0.0;
                    }

                    MatchDashboard.headingAssisted = headingAssisted;
                    MatchDashboard.precisionActive = precision;

                    Logger.recordOutput("Match/Drive/RawMagnitude", desired.getNorm());
                    Logger.recordOutput("Match/Drive/CommandMagnitude", translationFilter.getMagnitude());
                    Logger.recordOutput("Match/Drive/HeadingAssisted", headingAssisted);
                    Logger.recordOutput("Match/Drive/SpeedFraction", speedFraction);

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
                    targetHeading.reset();
                });
    }

    private static double manualOmega(
            SmoothRateLimiter rotationFilter, double stickOmega, double speedFraction) {
        return rotationFilter.calculate(stickOmega)
                * CompetitionConstants.maxAngularSpeedRadPerSec * speedFraction;
    }

    /**
     * Rumbo objetivo por odometría, con slew.
     *
     * <p>
     * El rumbo deseado es el rumbo al objetivo activo <b>más 180°</b> (los
     * cañones miran hacia atrás) <b>más la compensación de disparo en
     * movimiento</b>, que es el mismo delta que aplica la torreta. Se limita la
     * velocidad a la que ese rumbo puede cambiar para que una corrección de pose
     * de un ciclo no mande al robot a dar la vuelta.
     */
    private static final class TargetHeading {
        private double targetHeadingRad = Double.NaN;
        private double lastTimestamp = -1.0;

        void reset() {
            targetHeadingRad = Double.NaN;
            lastTimestamp = -1.0;
        }

        /** @return Error de rumbo a comandar, o {@code null} si no hay fuente. */
        Double computeErrorRad(Drive drive) {
            double now = Timer.getFPGATimestamp();
            double dt = (lastTimestamp < 0.0) ? 0.02 : now - lastTimestamp;
            lastTimestamp = now;
            dt = MathUtil.clamp(dt, 0.001, 0.1);

            Pose2d pose = drive.getPose();
            double currentHeading = pose.getRotation().getRadians();
            Translation2d target = CompetitionState.getActiveTarget(pose);
            Translation2d toTarget = target.minus(pose.getTranslation());

            if (toTarget.getNorm() < CompetitionConstants.headingMinTargetDistanceMeters) {
                // Pegados al objetivo: mantener el último rumbo bueno.
                if (Double.isNaN(targetHeadingRad)) {
                    return null;
                }
                return MathUtil.angleModulus(targetHeadingRad - currentHeading);
            }

            // Misma solución de tiro que la torreta y el cañón fijo.
            ShotSolution shot = ShotSolution.compute(
                    pose, drive.getFieldRelativeVelocity(), target);
            Logger.recordOutput("Match/Drive/AimOffsetDeg", Math.toDegrees(shot.aimOffsetRad));

            double desired = toTarget.getAngle().plus(Rotation2d.k180deg).getRadians()
                    + shot.aimOffsetRad;

            if (Double.isNaN(targetHeadingRad)) {
                targetHeadingRad = desired;
            } else {
                double step = MathUtil.angleModulus(desired - targetHeadingRad);
                double maxStep = CompetitionConstants.headingTargetSlewRadPerSec * dt;
                targetHeadingRad += MathUtil.clamp(step, -maxStep, maxStep);
            }

            return MathUtil.angleModulus(targetHeadingRad - currentHeading);
        }
    }
}
