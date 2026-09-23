package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.CompetitionConstants;
import frc.robot.subsystems.turret.Turret;

/**
 * Generador de setpoints suaves para la torreta en competencia.
 *
 * <p>
 * Es la parte del rastreador del demo que sigue aplicando cuando la cámara ya
 * no está en la torreta: el ángulo viene de la <b>odometría</b> (una función
 * suave del tiempo) y lo único que hace falta es que no se convierta en
 * latigazos ni en temblor:
 *
 * <ul>
 * <li><b>Filtro</b> — lima el escalón de una corrección de visión.</li>
 * <li><b>Slew</b> — el setpoint no puede moverse más rápido que
 * {@code turretMaxSetpointRateRadPerSec}. Una pose basura de un ciclo no manda
 * la torreta a dar la vuelta.</li>
 * <li><b>Deadband de comando</b> — el motor sólo se re-comanda si el setpoint
 * cambió más de {@code turretSetpointDeadbandRad}. Cada comando nuevo reinicia
 * el perfil de Motion Magic; re-comandar milésimas de grado a 50 Hz es
 * exactamente lo que hacía temblar la torreta en Denver.</li>
 * <li><b>Clamp</b> — siempre dentro de los soft limits.</li>
 * </ul>
 *
 * <p>
 * No hay histéresis de enganche: existía para no perseguir el ruido de píxel de
 * {@code tx}, y aquí no hay {@code tx}.
 */
public class TurretAimTracker {

    private double setpointRad = 0.0;
    private double lastCommandedRad = Double.NaN;
    private boolean initialized = false;
    private double lastTimestamp = -1.0;

    /** Arranca desde el ángulo actual para que el primer comando no sea un salto. */
    public void reset(double currentAngleRad) {
        setpointRad = Turret.clampToLimits(currentAngleRad);
        lastCommandedRad = Double.NaN;
        initialized = false;
        lastTimestamp = -1.0;
    }

    private double stepSeconds() {
        double now = Timer.getFPGATimestamp();
        double dt = (lastTimestamp < 0.0) ? 0.02 : now - lastTimestamp;
        lastTimestamp = now;
        return MathUtil.clamp(dt, 0.001, 0.1);
    }

    /** Mueve el setpoint hacia el ángulo objetivo con filtro y slew. */
    public void track(double targetAngleRad) {
        if (!Double.isFinite(targetAngleRad)) {
            return;
        }
        double dt = stepSeconds();
        double previous = setpointRad;
        double filtered = initialized
                ? previous + CompetitionConstants.turretSetpointFilterAlpha * (targetAngleRad - previous)
                : targetAngleRad;
        initialized = true;

        double maxStep = CompetitionConstants.turretMaxSetpointRateRadPerSec * dt;
        setpointRad = Turret.clampToLimits(
                MathUtil.clamp(filtered, previous - maxStep, previous + maxStep));
    }

    /** Mantiene el setpoint donde está (p. ej. muy cerca del objetivo). */
    public void hold() {
        stepSeconds();
    }

    /** Comanda la torreta sólo si el setpoint cambió lo suficiente. */
    public void apply(Turret turret) {
        if (Double.isNaN(lastCommandedRad)
                || Math.abs(setpointRad - lastCommandedRad) > CompetitionConstants.turretSetpointDeadbandRad) {
            turret.rotateToAngle(setpointRad);
            lastCommandedRad = setpointRad;
        }
    }

    public double setpoint() {
        return setpointRad;
    }
}
