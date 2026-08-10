package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

/**
 * Limitador escalar de tasa con suavizado de jerk.
 *
 * <h2>Qué estaba mal en la versión anterior (y por qué el robot se sentía
 * poseído)</h2>
 *
 * La primera versión usaba un perfil de tiempo mínimo: calculaba la distancia de
 * frenado y comandaba <b>aceleración máxima</b> siempre que hubiera cualquier
 * error, hasta que tocaba empezar a frenar. Eso es un controlador bang-bang.
 *
 * <p>
 * Las consecuencias en la mano del driver eran exactamente las que reportaron:
 *
 * <ul>
 * <li><b>"Movemos tantito el joystick y acelera brutalmente."</b> Un stick al 5%
 * generaba un error de 0.05, y el controlador respondía con la misma aceleración
 * máxima que habría usado para ir al 100%. No existía el concepto de "movimiento
 * pequeño".</li>
 *
 * <li><b>"Se avanza cañón" / "tiene lag".</b> La tasa interna tenía inercia
 * propia (limitada por jerk), así que cuando el driver centraba el stick, el
 * comando seguía creciendo unos ciclos más antes de poder revertirse.</li>
 *
 * <li><b>"Parece que se maneja solo."</b> Porque literalmente lo hacía: era un
 * servo persiguiendo un setpoint con su propia dinámica, no un filtro sobre la
 * intención del driver.</li>
 * </ul>
 *
 * <h2>Cómo funciona esta versión</h2>
 *
 * Es un <b>limitador</b>, no un controlador. La diferencia es de fondo: la
 * salida sigue a la entrada tan de cerca como se pueda, y sólo se recorta lo que
 * exceda los límites físicos.
 *
 * <pre>
 *   1. tasaPedida = (objetivo − valor) / dt        ← lo que el driver quiere YA
 *   2. tasaPedida = recortar(tasaPedida, accel/decel)
 *   3. tasa = pasoBajo(tasa → tasaPedida, tau)     ← redondea las esquinas
 *   4. paso = tasa · dt, nunca mayor que el error  ← jamás rebasa
 *   5. valor += paso
 * </pre>
 *
 * <p>
 * La propiedad clave es el paso 1 combinado con el 4: <b>si el driver pide un
 * cambio pequeño, el cambio pequeño pasa completo.</b> Los límites sólo entran a
 * trabajar cuando el driver pide algo que el hardware no puede dar. Eso es lo
 * que hace que se sienta en control.
 *
 * <p>
 * Y el paso 4 garantiza que soltar el stick detiene el robot: nunca se rebasa el
 * objetivo, así que no hay coleo ni "drift".
 */
public class SmoothRateLimiter {
    private final double maxAccel;
    private final double maxDecel;
    private final double smoothingTau;

    private double value = 0.0;
    private double rate = 0.0;
    private double lastTimestamp = -1.0;

    /**
     * @param maxAccel     Máxima tasa de crecimiento en magnitud (unidades / s).
     * @param maxDecel     Máxima tasa de decrecimiento en magnitud (unidades / s).
     * @param smoothingTau Constante de tiempo del suavizado de la tasa, en
     *                     segundos. Es lo que redondea las esquinas del perfil.
     *                     0 = sin suavizado (trapecio puro). 0.08-0.12 se siente
     *                     suave sin lag perceptible. Arriba de 0.25 el driver
     *                     empieza a sentir retardo.
     */
    public SmoothRateLimiter(double maxAccel, double maxDecel, double smoothingTau) {
        this.maxAccel = Math.abs(maxAccel);
        this.maxDecel = Math.abs(maxDecel);
        this.smoothingTau = Math.max(0.0, smoothingTau);
    }

    public double calculate(double target) {
        double now = Timer.getFPGATimestamp();
        double dt = (lastTimestamp < 0.0) ? 0.02 : now - lastTimestamp;
        lastTimestamp = now;
        dt = MathUtil.clamp(dt, 0.001, 0.1);

        double error = target - value;
        if (Math.abs(error) < 1.0e-6) {
            rate = 0.0;
            value = target;
            return value;
        }

        // Acelerar = alejarse de cero. Frenar = acercarse. Son dos problemas
        // mecánicos distintos (corriente de batería vs. back-drive de la
        // transmisión) y merecen dos límites distintos.
        boolean accelerating = Math.abs(target) >= Math.abs(value);
        double limit = accelerating ? maxAccel : maxDecel;

        // Lo que el driver pide en este ciclo, recortado a lo posible.
        double requestedRate = MathUtil.clamp(error / dt, -limit, limit);

        // Suavizado de primer orden. NO es un controlador: sólo evita que la
        // aceleración salte de golpe, que es lo que produce el tirón mecánico.
        double alpha = (smoothingTau <= 0.0) ? 1.0 : dt / (smoothingTau + dt);
        rate += alpha * (requestedRate - rate);

        // Nunca rebasar el objetivo. Sin esta línea el filtro coleaba.
        double step = rate * dt;
        if (Math.abs(step) > Math.abs(error)) {
            step = error;
            rate = step / dt;
        }

        value += step;
        return value;
    }

    /** Fuerza el estado interno. Llámalo en {@code teleopInit}. */
    public void reset(double newValue) {
        value = newValue;
        rate = 0.0;
        lastTimestamp = -1.0;
    }

    public double getValue() {
        return value;
    }

    /** Tasa de cambio actual — para graficar al tunear. */
    public double getRate() {
        return rate;
    }
}
