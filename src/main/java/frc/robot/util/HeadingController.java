package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

/**
 * Controlador de rumbo suave para el chasis (PD con compuerta suave).
 *
 * <p>
 * Es el controlador que se probó en el demo, extraído a una clase reutilizable
 * con sus ganancias por constructor. Tres cosas lo hacen suave, y las tres
 * faltaban en el {@code DriveCommands.joystickDrive} de Denver:
 *
 * <ol>
 * <li><b>La salida ya está en rad/s y no se re-escala.</b> Denver multiplicaba
 * la salida del {@code ProfiledPIDController} por
 * {@code getMaxAngularSpeedRadPerSec()} (~20 rad/s), lo que saturaba al
 * instante y convertía el lazo en bang-bang. Ése es el origen del "orbit muy
 * violento".</li>
 *
 * <li><b>Compuerta suave en vez de corte.</b> Apagar la salida de golpe con
 * {@code atSetpoint()} es una discontinuidad: corrige, se pasa, vuelve a
 * corregir. Aquí la salida se escala de 0 a 1 a lo largo de una rampa.</li>
 *
 * <li><b>Derivativo sobre el error filtrado.</b> Un término D crudo amplifica
 * el ruido de la medición; filtrarlo primero es lo que permite subirlo lo
 * suficiente para amortiguar de verdad.</li>
 * </ol>
 */
public class HeadingController {

    private final double kp;
    private final double kd;
    private final double derivativeAlpha;
    private final double toleranceRad;
    private final double softZoneRad;
    private final double maxOmegaRadPerSec;

    private double lastError = 0.0;
    private double filteredDerivative = 0.0;
    private double lastTimestamp = -1.0;
    private boolean initialized = false;

    /**
     * @param kp                Ganancia proporcional, rad/s por radián.
     * @param kd                Ganancia derivativa sobre el error filtrado.
     * @param derivativeAlpha   Filtro del derivativo (0-1, menor = más filtro).
     * @param toleranceRad      Debajo de esto no se comanda nada.
     * @param softZoneRad       Ancho de la rampa suave arriba de la tolerancia.
     * @param maxOmegaRadPerSec Tope de la salida.
     */
    public HeadingController(
            double kp,
            double kd,
            double derivativeAlpha,
            double toleranceRad,
            double softZoneRad,
            double maxOmegaRadPerSec) {
        this.kp = kp;
        this.kd = kd;
        this.derivativeAlpha = derivativeAlpha;
        this.toleranceRad = toleranceRad;
        this.softZoneRad = softZoneRad;
        this.maxOmegaRadPerSec = maxOmegaRadPerSec;
    }

    /**
     * @param errorRad Error de rumbo (objetivo − actual), en radianes CCW.
     * @return Velocidad angular a comandar, en rad/s.
     */
    public double calculate(double errorRad) {
        double now = Timer.getFPGATimestamp();
        double dt = (lastTimestamp < 0.0) ? 0.02 : now - lastTimestamp;
        lastTimestamp = now;
        dt = MathUtil.clamp(dt, 0.001, 0.1);

        // El error viene envuelto a ±π; envolver también la diferencia evita un
        // pico de ~2π/dt en el derivativo al cruzar la costura.
        double derivative = initialized ? MathUtil.angleModulus(errorRad - lastError) / dt : 0.0;
        initialized = true;
        lastError = errorRad;

        filteredDerivative += derivativeAlpha * (derivative - filteredDerivative);

        double output = kp * errorRad + kd * filteredDerivative;

        // Compuerta suave alrededor del setpoint.
        double gate = MathUtil.clamp(
                (Math.abs(errorRad) - toleranceRad) / Math.max(softZoneRad, 1.0e-6),
                0.0, 1.0);
        output *= gate;

        return MathUtil.clamp(output, -maxOmegaRadPerSec, maxOmegaRadPerSec);
    }

    public void reset() {
        lastError = 0.0;
        filteredDerivative = 0.0;
        lastTimestamp = -1.0;
        initialized = false;
    }
}
