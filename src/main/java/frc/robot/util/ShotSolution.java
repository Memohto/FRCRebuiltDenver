package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
 * la distancia. Si te mueves <i>acercándote</i>, el objetivo virtual queda entre
 * tú y el HUB —o sea, más cerca— y el mapa de tiro pide menos potencia solo,
 * porque la pelota ya lleva tu velocidad hacia adelante. Alejándote pasa lo
 * contrario. No hay que compensar nada por separado.
 *
 * <p>
 * Se itera un par de veces porque mover el objetivo cambia la distancia, y la
 * distancia cambia el tiempo de vuelo. Dos iteraciones convergen de sobra a las
 * distancias de FRC.
 *
 * <h2>Cómo se consume: el ángulo va como DELTA, no como setpoint</h2>
 *
 * La compensación se publica en dos formas y eso es deliberado:
 *
 * <ul>
 * <li>{@link #distanceMeters} — para los mapas de hood y flywheel.</li>
 * <li>{@link #aimOffsetRad} — <b>cuánto hay que girar</b> respecto a apuntar al
 * objetivo real, no a dónde apuntar.</li>
 * </ul>
 *
 * <p>
 * La razón es el traspaso visión ↔ odometría. La torreta apunta con {@code tx}
 * cuando ve el tag y con la pose cuando no; si la compensación viviera sólo en
 * la rama de odometría, la torreta pegaría un brinco cada vez que aparece o se
 * pierde el tag. Como delta se le suma a <b>las dos</b> ramas y el traspaso
 * sigue siendo continuo por construcción — el mismo principio del sesgo que
 * aprende {@code HubAlignment}.
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

    /**
     * Corrección angular, en radianes, respecto a apuntar al objetivo real.
     *
     * <p>
     * Positivo = girar en sentido antihorario (CCW), la misma convención que
     * usan {@code Rotation2d} y {@code Turret.computeTurretAngleRad}. Se le
     * <b>suma</b> al ángulo que ya calcula cada consumidor; nunca lo reemplaza.
     * Es 0 exacto cuando no hay compensación.
     */
    public final double aimOffsetRad;

    private ShotSolution(
            Translation2d aimPoint,
            double distanceMeters,
            double timeOfFlightSeconds,
            double compensationMeters,
            double aimOffsetRad) {
        this.aimPoint = aimPoint;
        this.distanceMeters = distanceMeters;
        this.timeOfFlightSeconds = timeOfFlightSeconds;
        this.compensationMeters = compensationMeters;
        this.aimOffsetRad = aimOffsetRad;
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
        double staticTof = DemoConstants.kTimeOfFlightMap.get(clampDistance(staticDistance));

        // Sin compensación: apuntar directo al objetivo.
        //
        // Los tres primeros casos son la lógica de siempre; los dos últimos son
        // redes de seguridad. isFinite() atrapa un NaN que se haya colado desde
        // la pose o desde los módulos, y la distancia mínima evita calcular un
        // rumbo encima del objetivo, donde unos centímetros son decenas de
        // grados.
        if (!DemoConstants.shootWhileMovingEnabled
                || fieldVelocity == null
                || !Double.isFinite(fieldVelocity.getNorm())
                || fieldVelocity.getNorm() < DemoConstants.shootWhileMovingMinSpeed
                || !Double.isFinite(staticDistance)
                || staticDistance < DemoConstants.minShotDistanceMeters) {
            return new ShotSolution(
                    target, clampDistance(staticDistance), staticTof, 0.0, 0.0);
        }

        // Objetivo virtual, con un par de iteraciones para converger.
        Translation2d aim = target;
        double tof = staticTof;
        for (int i = 0; i < DemoConstants.shootWhileMovingIterations; i++) {
            tof = DemoConstants.kTimeOfFlightMap.get(clampDistance(robot.getDistance(aim)));
            Translation2d offset = fieldVelocity.times(-tof * DemoConstants.shootWhileMovingGain);

            // Tope de seguridad: si la velocidad estimada viene basura, la
            // corrección se dispara y la torreta se va a un punto absurdo.
            if (offset.getNorm() > DemoConstants.shootWhileMovingMaxCompensationMeters) {
                offset = offset.times(
                        DemoConstants.shootWhileMovingMaxCompensationMeters / offset.getNorm());
            }
            aim = target.plus(offset);
        }

        // El tiempo de vuelo que se reporta tiene que ser el del punto de mira
        // que devolvemos, no el de la iteración anterior.
        tof = DemoConstants.kTimeOfFlightMap.get(clampDistance(robot.getDistance(aim)));

        // ── Corrección angular ──────────────────────────────────────────────
        // Es la diferencia de rumbos vista DESDE EL ROBOT, no la orientación
        // del desplazamiento del punto de mira: correr el objetivo un metro a
        // 3 m de distancia no es lo mismo que a 8 m.
        Translation2d toTarget = target.minus(robot);
        Translation2d toAim = aim.minus(robot);

        // El objetivo virtual puede caer encima del robot aunque el real esté
        // lejos: la corrección llega a 2.5 m. Sin esta guarda, atan2(0,0) da un
        // rumbo inventado que cambia de signo con el ruido.
        if (toAim.getNorm() < DemoConstants.minShotDistanceMeters) {
            return new ShotSolution(
                    target, clampDistance(staticDistance), staticTof, 0.0, 0.0);
        }

        double bearingToTarget = Math.atan2(toTarget.getY(), toTarget.getX());
        double rawOffsetRad = MathUtil.angleModulus(
                Math.atan2(toAim.getY(), toAim.getX()) - bearingToTarget);

        // El signo se aplica aquí y el tope angular después. El tope en metros no
        // alcanza como red de seguridad: 2.5 m son 15° a 9 m pero más de 60° a
        // 2 m.
        double offsetRad = MathUtil.clamp(
                rawOffsetRad * DemoConstants.shootWhileMovingAimSign,
                -DemoConstants.shootWhileMovingMaxAimOffsetRad,
                DemoConstants.shootWhileMovingMaxAimOffsetRad);

        if (!Double.isFinite(offsetRad)) {
            offsetRad = 0.0;
        }

        // El punto de mira se reconstruye desde el ángulo FINAL, ya con signo y
        // tope aplicados. Si no, el ángulo y la distancia describirían tiros
        // distintos: la torreta apuntando a un lado y el hood calculando para un
        // punto al que ya no le estamos apuntando. La distancia no cambia —
        // espejear o recortar el ángulo gira el punto de mira alrededor del
        // robot, no lo acerca ni lo aleja.
        if (offsetRad != rawOffsetRad) {
            aim = robot.plus(new Translation2d(
                    toAim.getNorm(), new Rotation2d(bearingToTarget + offsetRad)));
        }

        double compensation = aim.getDistance(target);
        return new ShotSolution(
                aim, clampDistance(robot.getDistance(aim)), tof, compensation, offsetRad);
    }

    /**
     * Recorta la distancia a un rango físicamente sensato.
     *
     * <p>
     * <b>Recorta, no teletransporta.</b> La versión anterior devolvía
     * {@code fallbackDistanceMeters} (3 m) cuando la distancia se salía del
     * rango, y eso convertía el borde del rango en un escalón: pasar de 0.51 a
     * 0.49 m saltaba la potencia de mínima a la de 3 m de un ciclo al otro. Con
     * el objetivo virtual ese borde se cruza mucho más seguido, porque manejar
     * hacia el HUB acerca el punto de mira.
     *
     * <p>
     * El fallback se queda sólo para NaN, que es el único caso donde de verdad
     * no hay ningún número que recortar.
     */
    private static double clampDistance(double distance) {
        if (!Double.isFinite(distance)) {
            return DemoConstants.fallbackDistanceMeters;
        }
        return MathUtil.clamp(
                distance, DemoConstants.minShotDistanceMeters, DemoConstants.maxShotDistanceMeters);
    }

    /** Publica la solución al log. Se llama una sola vez por ciclo. */
    public void log() {
        Logger.recordOutput("Demo/Shot/AimPoint", aimPoint);
        Logger.recordOutput("Demo/Shot/DistanceMeters", distanceMeters);
        Logger.recordOutput("Demo/Shot/TimeOfFlightSec", timeOfFlightSeconds);
        Logger.recordOutput("Demo/Shot/CompensationMeters", compensationMeters);
        Logger.recordOutput("Demo/Shot/AimOffsetDeg", Math.toDegrees(aimOffsetRad));
        // El dashboard NO se escribe aquí. Lo llena quien apunta, con el valor
        // que de verdad aplicó — que no siempre es éste, porque en caza libre la
        // compensación se calcula contra el HUB y no se usa. Escribirlo en los
        // dos lados dejaba el widget dependiendo del orden de los comandos.
    }
}
