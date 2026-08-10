package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * Smooth Drive System — limitador vectorial del comando de traslación.
 *
 * <h2>Qué se quitó respecto a la primera versión</h2>
 *
 * La primera versión tenía tres mecanismos que sonaban bien en teoría y en la
 * mano del driver se sentían horrible. Los tres se quitaron:
 *
 * <ol>
 * <li><b>El perfil de tiempo mínimo (bang-bang).</b> Cualquier movimiento del
 * stick, por chico que fuera, disparaba aceleración máxima. Ver
 * {@link SmoothRateLimiter} para el detalle.</li>
 *
 * <li><b>El frenado ante inversión.</b> Si el driver cambiaba de dirección más
 * de 100°, el filtro decidía frenar hasta cero <i>conservando el rumbo viejo</i>
 * antes de hacerle caso. Desde el asiento del driver eso es el robot ignorándote
 * y siguiéndose de largo. Ahora una inversión simplemente se maneja sola: el
 * vector de error apunta hacia atrás y el comando pasa por cero de forma
 * natural.</li>
 *
 * <li><b>El límite de aceleración lateral (arc limiting).</b> Impedía que el
 * vector de velocidad girara rápido a alta velocidad. Físicamente correcto,
 * pero es justo lo que hacía sentir que el robot "buscaba una posición" en vez
 * de obedecer. Y era redundante: las llantas ya imponen ese límite solas. Si
 * derrapa, se baja la aceleración, no se le agrega una regla más.</li>
 * </ol>
 *
 * <h2>Qué hace ahora</h2>
 *
 * Un limitador de tasa isotrópico sobre el vector completo:
 *
 * <pre>
 *   error     = deseado − actual                (vector)
 *   tasaPedida = recortarNorma(error / dt, límite)
 *   tasa      = pasoBajo(tasa → tasaPedida, tau)
 *   paso      = tasa · dt, nunca mayor que el error
 *   actual   += paso
 * </pre>
 *
 * <p>
 * Isotrópico significa que se recorta la <b>norma</b> del vector, no cada eje
 * por separado. Recortar por eje haría que un movimiento diagonal acelerara √2
 * veces más rápido que uno recto, y el robot se sentiría distinto según hacia
 * dónde apuntes el stick.
 *
 * <p>
 * Lo importante: si el driver pide un cambio que cabe dentro del límite, ese
 * cambio pasa <b>completo y sin retardo</b>. El filtro sólo interviene cuando se
 * le pide al hardware algo que no puede dar.
 */
public class SmoothDriveFilter {
    private final double maxAccel;
    private final double maxDecel;
    private final double smoothingTau;

    private Translation2d value = Translation2d.kZero;
    private Translation2d rate = Translation2d.kZero;
    private double lastTimestamp = -1.0;

    /**
     * @param maxAccel     Aceleración máxima del comando, en unidades
     *                     normalizadas/s. 2.5 ≈ de parado a full en 0.40 s.
     * @param maxDecel     Desaceleración máxima, en unidades normalizadas/s.
     * @param smoothingTau Constante de tiempo del suavizado de la aceleración,
     *                     en segundos. Es la perilla de "qué tan redondeadas
     *                     salen las esquinas".
     */
    public SmoothDriveFilter(double maxAccel, double maxDecel, double smoothingTau) {
        this.maxAccel = Math.abs(maxAccel);
        this.maxDecel = Math.abs(maxDecel);
        this.smoothingTau = Math.max(0.0, smoothingTau);
    }

    /**
     * Filtra un comando de traslación normalizado (magnitud 0..1).
     *
     * @param desired Comando crudo del joystick, ya con deadband y curva de
     *                respuesta aplicadas.
     * @return Comando suavizado, misma escala.
     */
    public Translation2d calculate(Translation2d desired) {
        double now = Timer.getFPGATimestamp();
        double dt = (lastTimestamp < 0.0) ? 0.02 : now - lastTimestamp;
        lastTimestamp = now;
        dt = MathUtil.clamp(dt, 0.001, 0.1);

        Translation2d error = desired.minus(value);
        double errorNorm = error.getNorm();

        if (errorNorm < 1.0e-6) {
            value = desired;
            rate = Translation2d.kZero;
            return value;
        }

        // ¿Estamos pidiendo ir más rápido o más lento? Acelerar consume
        // corriente de la batería; frenar back-drivea la transmisión. Dos
        // problemas distintos, dos límites distintos.
        boolean accelerating = desired.getNorm() >= value.getNorm();
        double limit = accelerating ? maxAccel : maxDecel;

        // Lo que el driver pide en este ciclo, recortado en NORMA (isotrópico).
        Translation2d requestedRate = error.div(dt);
        double requestedNorm = requestedRate.getNorm();
        if (requestedNorm > limit) {
            requestedRate = requestedRate.times(limit / requestedNorm);
        }

        // Suavizado de primer orden sobre la aceleración.
        double alpha = (smoothingTau <= 0.0) ? 1.0 : dt / (smoothingTau + dt);
        rate = rate.plus(requestedRate.minus(rate).times(alpha));

        // Nunca rebasar el objetivo: si el paso es más largo que el error,
        // aterrizamos exacto. Esto es lo que garantiza que soltar el stick
        // detenga el robot en vez de dejarlo coleando.
        Translation2d step = rate.times(dt);
        if (step.getNorm() > errorNorm) {
            step = error;
            rate = step.div(dt);
        }

        value = value.plus(step);
        return value;
    }

    /** Reinicia el filtro al reposo. Llámalo en {@code teleopInit}. */
    public void reset() {
        value = Translation2d.kZero;
        rate = Translation2d.kZero;
        lastTimestamp = -1.0;
    }

    /** Magnitud filtrada actual — para logging/tuning. */
    public double getMagnitude() {
        return value.getNorm();
    }

    /** Magnitud de la aceleración instantánea — para logging/tuning. */
    public double getAcceleration() {
        return rate.getNorm();
    }

    /** Dirección filtrada actual — para logging/tuning. */
    public Rotation2d getDirection() {
        return value.getNorm() > 1.0e-6 ? value.getAngle() : Rotation2d.kZero;
    }
}
