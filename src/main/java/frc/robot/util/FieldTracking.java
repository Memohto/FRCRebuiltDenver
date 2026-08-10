package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.DemoConstants;
import frc.robot.constants.RobotConstants;

/**
 * Memoria de campo: dónde está el HUB y hace cuánto que la visión no confirma
 * dónde estamos.
 *
 * <h2>Por qué existe</h2>
 *
 * El código de competencia apunta al HUB por <b>odometría</b>: sabe dónde está
 * el HUB porque es una constante del campo, y sabe dónde está el robot porque el
 * pose estimator fusiona encoders + Pigeon + AprilTags. Con eso puede seguir
 * disparando aunque no vea ningún tag.
 *
 * <p>
 * La primera versión del demo tiró eso a la basura: apagó las actualizaciones de
 * pose y dejó el apuntado dependiendo 100% de ver un tag. Resultado: si el tag
 * desaparecía un segundo, la torreta se ponía a barrer como si nunca hubiera
 * visto nada, aunque tuviera toda la información para saber perfectamente dónde
 * está el HUB.
 *
 * <h2>Cómo se resuelve</h2>
 *
 * Es la misma jerarquía que usan los equipos que disparan en movimiento: la
 * visión no <i>es</i> el apuntado, la visión <b>corrige</b> la odometría, y el
 * apuntado sale de la odometría.
 *
 * <pre>
 *   1. Ve un tag del HUB  → cierra el lazo sobre tx (lo más preciso)
 *                           y de paso corrige la pose de campo
 *   2. No ve nada, pero la pose es reciente
 *                         → apunta por odometría (método de competencia)
 *   3. Ni eso            → barre buscando
 * </pre>
 *
 * <p>
 * Entre el paso 1 y el 3 hay una ventana de varios segundos donde el robot sigue
 * apuntando bien sin ver nada. Eso es lo que evita que se ponga a girar la
 * cabeza como menso.
 */
public class FieldTracking {

    private static double lastPoseUpdateTimestamp = -1000.0;

    private FieldTracking() {
    }

    /** La llama {@code Vision} cada vez que ACEPTA una corrección de pose. */
    public static void notePoseUpdate() {
        lastPoseUpdateTimestamp = Timer.getFPGATimestamp();
    }

    /** Segundos desde la última corrección de visión aceptada. */
    public static double secondsSinceUpdate() {
        return Timer.getFPGATimestamp() - lastPoseUpdateTimestamp;
    }

    /**
     * ¿Le podemos creer a la odometría para apuntar?
     *
     * <p>
     * Sí, mientras la visión la haya confirmado hace poco. Pasado ese tiempo, la
     * deriva de encoders y giroscopio empieza a acumularse y es mejor admitir que
     * no sabemos dónde estamos que disparar a un HUB imaginario.
     *
     * <p>
     * La ventana es generosa a propósito: un swerve con Pigeon 2.0 deriva muy
     * poco en unos segundos, y el costo de ser demasiado estricto es justo el
     * comportamiento que queremos evitar.
     */
    public static boolean isOdometryValid() {
        return secondsSinceUpdate() < DemoConstants.odometryTrustSeconds;
    }

    public static void reset() {
        lastPoseUpdateTimestamp = -1000.0;
    }

    /**
     * Objetivo activo del apuntado.
     *
     * <p>
     * Devuelve el objetivo virtual si el modo de cero odometría está activo, y
     * el HUB de la alianza en cualquier otro caso. Todos los consumidores
     * —torreta, cañón fijo y rumbo del chasis— llaman aquí, así que apuntan
     * siempre al mismo punto.
     */
    public static Translation2d getActiveTarget() {
        return getHubPosition();
    }

    /**
     * Posición del HUB de la alianza que reporte la Driver Station.
     *
     * <p>
     * Con {@code useAllianceHub = false} se usa {@code demoHubPosition}, para
     * cuando colocan un HUB en una posición arbitraria que ustedes midieron.
     */
    public static Translation2d getHubPosition() {
        if (!DemoConstants.useAllianceHub) {
            return DemoConstants.demoHubPosition;
        }
        boolean isRed = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
        return isRed ? RobotConstants.redHub : RobotConstants.blueHub;
    }
}
