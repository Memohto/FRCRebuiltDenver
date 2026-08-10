package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.DemoConstants;

/**
 * Solución de tiro: a dónde apuntar y a qué distancia.
 *
 * <h2>Por qué existe esta clase</h2>
 *
 * <b>Es la corrección al bug de BOMBER.</b> Antes, la torreta y el cañón fijo
 * calculaban su distancia por separado y con criterios distintos: la torreta
 * usaba odometría pura, y el cañón fijo usaba visión-si-hay-else-odometría. Con
 * un tag a la vista cada uno recibía una distancia diferente, así que ajustaban
 * hoods distintos; al perder el tag volvían a coincidir. De ahí el "se queda
 * trabado el shooter fijo, sólo se ajusta el hood de la torreta, y a veces pasa
 * al revés".
 *
 * <p>
 * Ahora los tres consumidores —torreta, cañón fijo y rumbo del chasis— llaman a
 * este mismo método con los mismos argumentos. Reciben el mismo número por
 * construcción, no por coincidencia.
 *
 * <h2>Disparo en movimiento</h2>
 *
 * La pelota sale del robot cargando <b>la velocidad del robot</b>. Si el robot
 * se mueve a 2 m/s hacia la izquierda y el vuelo dura 0.6 s, la pelota se
 * desvía 1.2 m a la izquierda del punto al que apuntaste.
 *
 * <p>
 * La solución estándar —la que usan los equipos que disparan en movimiento— es
 * el <b>objetivo virtual</b>: en vez de compensar el ángulo con una fórmula
 * aparte, se mueve el punto al que apuntas en dirección contraria a tu
 * velocidad, y se apunta ahí normalmente:
 *
 * <pre>
 *   objetivoVirtual = objetivoReal − velocidadDelRobot × tiempoDeVuelo
 * </pre>
 *
 * <p>
 * La elegancia está en que <b>resuelve las dos cosas a la vez</b>: el ángulo Y
 * la distancia. Si te mueves acercándote, el objetivo virtual queda más lejos y
 * el mapa de tiro pide más potencia solo. No hay que compensar nada por
 * separado.
 *
 * <p>
 * Se itera un par de veces porque mover el objetivo cambia la distancia, y la
 * distancia cambia el tiempo de vuelo. Dos iteraciones convergen de sobra a las
 * distancias de FRC.
 */
public class ShotSolution {

    /** Punto al que hay que apuntar, ya compensado por movimiento. */
    public final Translation2d aimPoint;

    /** Distancia al punto de mira. Es la que va a los mapas de tiro. */
    public final double distanceMeters;

    /** Tiempo de vuelo estimado. Sólo informativo/telemetría. */
    public final double timeOfFlightSeconds;

    /** Cuánto se corrió el punto de mira respecto al objetivo real. */
    public final double compensationMeters;

    private ShotSolution(
            Translation2d aimPoint,
            double distanceMeters,
            double timeOfFlightSeconds,
            double compensationMeters) {
        this.aimPoint = aimPoint;
        this.distanceMeters = distanceMeters;
        this.timeOfFlightSeconds = timeOfFlightSeconds;
        this.compensationMeters = compensationMeters;
    }

    /**
     * Calcula la solución de tiro.
     *
     * <p>
     * <b>Todos los consumidores deben llamar a este método con los mismos
     * argumentos.</b> Es lo que garantiza que la torreta, el cañón fijo y el
     * chasis apunten al mismo lugar.
     *
     * @param robotPose     Pose actual del robot.
     * @param fieldVelocity Velocidad del robot en marco de campo, m/s.
     * @param target        Objetivo real (HUB de alianza, o el virtual del modo
     *                      de cero odometría).
     */
    public static ShotSolution compute(
            Pose2d robotPose, Translation2d fieldVelocity, Translation2d target) {

        Translation2d robot = robotPose.getTranslation();
        double staticDistance = robot.getDistance(target);

        // Sin compensación: apuntar directo al objetivo.
        if (!DemoConstants.shootWhileMovingEnabled
                || fieldVelocity.getNorm() < DemoConstants.shootWhileMovingMinSpeed) {
            double tof = DemoConstants.kTimeOfFlightMap.get(staticDistance);
            return new ShotSolution(target, clampDistance(staticDistance), tof, 0.0);
        }

        // Objetivo virtual, con un par de iteraciones para converger.
        Translation2d aim = target;
        double tof = DemoConstants.kTimeOfFlightMap.get(staticDistance);
        for (int i = 0; i < DemoConstants.shootWhileMovingIterations; i++) {
            tof = DemoConstants.kTimeOfFlightMap.get(robot.getDistance(aim));
            Translation2d offset = fieldVelocity.times(-tof * DemoConstants.shootWhileMovingGain);

            // Tope de seguridad: si la velocidad estimada viene basura, la
            // corrección se dispara y la torreta se va a un punto absurdo.
            if (offset.getNorm() > DemoConstants.shootWhileMovingMaxCompensationMeters) {
                offset = offset.times(
                        DemoConstants.shootWhileMovingMaxCompensationMeters / offset.getNorm());
            }
            aim = target.plus(offset);
        }

        double compensation = aim.getDistance(target);
        return new ShotSolution(
                aim, clampDistance(robot.getDistance(aim)), tof, compensation);
    }

    /** Recorta la distancia a un rango físicamente sensato. */
    private static double clampDistance(double distance) {
        if (distance < 0.5 || distance > 12.0) {
            return DemoConstants.fallbackDistanceMeters;
        }
        return distance;
    }

    /** Publica la solución al log. Se llama una sola vez por ciclo. */
    public void log() {
        Logger.recordOutput("Demo/Shot/AimPoint", aimPoint);
        Logger.recordOutput("Demo/Shot/DistanceMeters", distanceMeters);
        Logger.recordOutput("Demo/Shot/TimeOfFlightSec", timeOfFlightSeconds);
        Logger.recordOutput("Demo/Shot/CompensationMeters", compensationMeters);
        DemoDashboard.shotCompensationMeters = compensationMeters;
    }
}
