package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DemoConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputsAutoLogged;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    public final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    /**
     * Clave bajo la que este shooter escribe al log.
     *
     * <p>
     * Existe porque {@link frc.robot.subsystems.turret.Turret} hereda de esta
     * clase: sin esto, la torreta y el shooter fijo escribían los dos a la tabla
     * "Shooter" y se pisaban en cada ciclo, haciendo imposible saber en el log
     * cuál de los dos cañones hizo qué.
     */
    protected String logKey = "Shooter";

    /**
     * Últimos setpoints comandados, para no re-comandar cambios insignificantes.
     *
     * <p>
     * <b>Cada setpoint nuevo reinicia el perfil de Motion Magic del hood.</b>
     * Re-comandar a 50 Hz con cambios de milésimas de grado hace que el perfil
     * nunca se complete y el mecanismo tiemble en su lugar — es el mismo
     * problema que la torreta ya resolvía con {@code turretSetpointDeadbandRad}
     * y que aquí faltaba.
     *
     * <p>
     * Se vuelve importante con el disparo en movimiento: sin compensación la
     * distancia cambia despacio, con compensación se mueve con la velocidad del
     * robot y el hood bailaría.
     *
     * <p>
     * Arrancan en NaN para que el primer comando de cada uno siempre pase.
     */
    private double lastHoodDeg = Double.NaN;
    private double lastFlywheelRadPerSec = Double.NaN;

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);

        // Deshabilitado se olvida lo último comandado. Si el motor se reinició
        // por un brownout, su setpoint interno ya no es el que creemos, y sin
        // esto el deadband se lo tragaría por pedir "el mismo" número.
        if (DriverStation.isDisabled()) {
            lastHoodDeg = Double.NaN;
            lastFlywheelRadPerSec = Double.NaN;
        }

        // Log error between target and actual for tuning kV/kP
        double velocityError = inputs.flywheelTargetVelocityRadPerSec - inputs.flywheelVelocityRadPerSec;
        Logger.recordOutput(logKey + "/FlywheelVelocityErrorRadPerSec", velocityError);
        Logger.recordOutput(logKey + "/FlywheelAtSpeed", isFlywheelAtSpeed());
    }

    // ── Flywheel ──────────────────────────────────────────────────────────────

    /**
     * Closed-loop: commands the flywheel to a specific speed in rad/s.
     * The TalonFX will maintain this speed regardless of battery voltage.
     * This is what you should use for all shooting during matches.
     */
    public void setFlywheelVelocity(double velocityRadPerSec) {
        if (!Double.isFinite(velocityRadPerSec)) {
            return; // Un NaN aquí es un comando sin sentido para el motor.
        }
        if (velocityRadPerSec == 0.0) {
            // Apagar nunca se filtra. Hoy todos los paros pasan por
            // stopFlywheel(), pero un deadband que se pueda tragar un cero es
            // una trampa esperando a que alguien llame a este método directo.
            lastFlywheelRadPerSec = 0.0;
            io.setFlywheelVelocity(0.0);
            return;
        }
        double deadbandRadPerSec =
                Units.rotationsToRadians(DemoConstants.flywheelSetpointDeadbandRPS);
        if (Math.abs(velocityRadPerSec - lastFlywheelRadPerSec) < deadbandRadPerSec) {
            return;
        }
        lastFlywheelRadPerSec = velocityRadPerSec;
        io.setFlywheelVelocity(velocityRadPerSec);
    }

    /**
     * Looks up the correct flywheel speed for a given distance from the shot map
     * and commands closed-loop velocity control.
     * NOTE: kShooterFlywheelMap values must be in RPS (rotations per second),
     * NOT 0-1 duty cycle. Update ShooterConstants accordingly.
     * Example: 60.0 RPS ≈ 3600 RPM (reasonable mid-range shooting speed)
     */
    public void setFlywheelVelocityForDistance(double distanceMeters) {
        double velocityRPS = ShooterConstants.kShooterFlywheelMap.get(distanceMeters);
        double velocityRadPerSec = Units.rotationsToRadians(velocityRPS);
        // logKey y no "Shooter" a secas: si no, la torreta pisa la clave del
        // cañón fijo y en el log no se sabe cuál de los dos pidió qué.
        Logger.recordOutput(logKey + "/FlywheelTargetRPS", velocityRPS);
        setFlywheelVelocity(velocityRadPerSec);
    }

    /**
     * Open-loop: sets raw duty cycle. Use ONLY for manual override or testing.
     * Battery-dependent — do not use during matches.
     */
    public void setFlywheelOpenLoop(double speed) {
        // Salir de lazo cerrado invalida el setpoint recordado: el siguiente
        // comando de velocidad tiene que llegar al motor aunque pida el mismo
        // número que la última vez.
        lastFlywheelRadPerSec = Double.NaN;
        io.setFlywheelOpenLoop(speed);
    }

    /** @deprecated Use setFlywheelVelocityForDistance() instead for battery-independent shooting. */
    @Deprecated
    public void setFlywheelSpeed(double speed) {
        setFlywheelOpenLoop(speed);
    }

    public void stopFlywheel() {
        setFlywheelOpenLoop(0.0);
    }

    /**
     * Returns true when the flywheel is within tolerance of its target speed.
     * Use this to gate indexer/feeding — don't feed until this is true!
     * Tolerance defined in ShooterConstants.flywheelToleranceRadPerSec.
     */
    public boolean isFlywheelAtSpeed() {
        double error = Math.abs(
            inputs.flywheelTargetVelocityRadPerSec - inputs.flywheelVelocityRadPerSec
        );
        return inputs.flywheelTargetVelocityRadPerSec > 0
            && error < ShooterConstants.flywheelToleranceRadPerSec;
    }

    // ── Hood ─────────────────────────────────────────────────────────────────

    public void setHoodOpenLoop(double speed) {
        lastHoodDeg = Double.NaN;
        io.setHoodOpenLoop(speed);
    }

    /**
     * Comanda el hood, saltándose los cambios insignificantes.
     *
     * <p>
     * Es el único embudo hacia {@code io.setHoodPosition}, así que el deadband
     * protege a todos los que comanden el hood, vengan del mapa que vengan.
     */
    public void setHoodPosition(Rotation2d rotation) {
        double degrees = rotation.getDegrees();
        if (!Double.isFinite(degrees)) {
            return;
        }
        if (Math.abs(degrees - lastHoodDeg) < DemoConstants.hoodSetpointDeadbandDeg) {
            return;
        }
        lastHoodDeg = degrees;
        Logger.recordOutput(logKey + "/HoodTargetDeg", degrees);
        io.setHoodPosition(rotation);
    }

    public void setHoodForDistance(double distanceMeters) {
        double hoodOffsetDeg = ShooterConstants.kShooterHoodMap.get(distanceMeters);
        setHoodPosition(Rotation2d.fromDegrees(hoodOffsetDeg));
    }

    public void setHoodAtInitialPosition() {
        setHoodPosition(Rotation2d.kZero);
    }

    // ══════════════════════════════════════════════════════════════════════════
    // Overloads genéricos — permiten usar mapas de tiro distintos (competencia
    // vs. tiro suave de demo) sin duplicar la lógica de control.
    // ══════════════════════════════════════════════════════════════════════════

    /**
     * Velocidad de flywheel desde un mapa arbitrario, con tope duro opcional.
     *
     * @param map            Mapa distancia (m) → RPS.
     * @param distanceMeters Distancia al objetivo.
     * @param maxRPS         Tope duro. Ningún valor del mapa puede excederlo.
     *                       Es la red de seguridad del modo demo: aunque alguien
     *                       edite el mapa mal, el robot no puede disparar un
     *                       torpedo a un alumno.
     */
    public void setFlywheelVelocityFromMap(
            InterpolatingDoubleTreeMap map, double distanceMeters, double maxRPS) {
        double velocityRPS = Math.min(map.get(distanceMeters), maxRPS);
        Logger.recordOutput(logKey + "/FlywheelTargetRPS", velocityRPS);
        setFlywheelVelocity(Units.rotationsToRadians(velocityRPS));
    }

    /** Ángulo de hood desde un mapa arbitrario. */
    public void setHoodFromMap(InterpolatingDoubleTreeMap map, double distanceMeters) {
        setHoodPosition(Rotation2d.fromDegrees(map.get(distanceMeters)));
    }
}
