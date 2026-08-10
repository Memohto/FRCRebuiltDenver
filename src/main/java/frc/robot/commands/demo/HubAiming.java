package frc.robot.commands.demo;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.DemoConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.DemoState;

/**
 * Apuntado vectorial al centro del HUB.
 *
 * <h2>⚠ ACTUALMENTE NO SE USA</h2>
 *
 * El rollback devolvió el apuntado al HUB a odometría pura, como en competencia:
 * la posición del HUB es una constante del campo y ya <i>es</i> su centro, así
 * que no hay nada que corregir vectorialmente. La visión sólo mantiene la pose
 * corregida.
 *
 * <p>
 * Este archivo se conserva porque el razonamiento sigue siendo válido y es la
 * base si algún día quieren refinar el apuntado con visión a corta distancia:
 * seleccionar determinísticamente entre la tag central y la izquierda de cada
 * cara ({@code priorityid}) y corregir del centro del tag al centro del HUB.
 * Para reactivarlo, llamen a estos métodos desde {@code DemoTurretCommands} y
 * súmenle la corrección a {@code tx}.
 *
 * <h2>El problema que resolvía</h2>
 *
 * Cada cara del HUB tiene dos AprilTags: una centrada y otra a su izquierda.
 * Eso crea dos problemas distintos:
 *
 * <ol>
 * <li><b>Salto entre tags.</b> La Limelight reporta {@code tx} respecto al tag
 * "primario", que por defecto es el más grande de la imagen. Con dos tags a la
 * vista, cuál es el más grande cambia según el ángulo y la distancia — así que
 * el primario salta de uno a otro y la torreta pega un brinco cada vez. Ésta es
 * buena parte del comportamiento glitchy que se vio en las pruebas.</li>
 *
 * <li><b>El centro del tag no es el centro del HUB.</b> Si ves la cara de
 * frente, apuntar al tag central es apuntar al HUB y todo bien. Pero en cuanto
 * te mueves de lado, el centro real del HUB queda <i>detrás y a un lado</i> del
 * tag, y la diferencia crece con el ángulo. Apuntar al tag manda la pelota al
 * borde de la estructura, no al hoyo.</li>
 * </ol>
 *
 * <h2>La solución</h2>
 *
 * <b>Selección determinista:</b> se le dice a la propia cámara cuál tag usar
 * ({@code priorityid}), prefiriendo siempre la central. Si sólo se ve la
 * izquierda, se trabaja con ella y se compensa el offset lateral. Ya no hay
 * saltos.
 *
 * <p>
 * <b>Corrección vectorial:</b> a partir de la pose 3D del tag se obtiene la
 * normal de la cara, y se camina desde el tag hacia adentro del HUB para
 * encontrar su eje central. La diferencia angular entre "apuntar al tag" y
 * "apuntar al centro" se devuelve como una corrección que se suma a {@code tx}.
 *
 * <p>
 * Trabajar en el espacio angular de {@code tx} —y no calculando un setpoint
 * absoluto— es deliberado: así toda la compensación de latencia del rastreador
 * sigue funcionando sin cambios, y la corrección queda como una cantidad chica y
 * lenta que se puede filtrar fuerte.
 */
public class HubAiming {

    private static final int CAM = DemoDriveCommands.TURRET_CAMERA;

    private HubAiming() {
    }

    /**
     * Decide a qué tag debe apuntar la cámara.
     *
     * <p>
     * Prioridad en modo HUB: central > izquierda > ninguna. En caza global: el
     * tag fijado si sigue visible, si no el primero que haya.
     *
     * @return ID del tag elegido, o -1 para dejar que la cámara decida sola.
     */
    public static int selectTargetTag(Vision vision) {
        int[] visible = vision.getVisibleTagIds(CAM);

        // Firmware sin rawfiducials: nos conformamos con el primario.
        if (visible.length == 0) {
            int primary = vision.getPrimaryTagId(CAM);
            return DemoState.acceptsTag(primary) ? primary : -1;
        }

        if (DemoState.isHubTargeting()) {
            // Si están las dos a la vista, la izquierda se ignora.
            for (int id : visible) {
                if (DemoState.isHubCenterTag(id)) {
                    return id;
                }
            }
            for (int id : visible) {
                if (DemoState.isHubLeftTag(id)) {
                    return id;
                }
            }
            return -1;
        }

        int locked = DemoState.getLockedTagId();
        if (locked >= 0) {
            for (int id : visible) {
                if (id == locked) {
                    return locked;
                }
            }
        }
        return visible[0];
    }

    /**
     * Corrección angular para apuntar al centro del HUB en vez de al tag.
     *
     * <p>
     * Devuelve un valor en radianes que hay que <b>sumarle a {@code tx}</b>
     * antes de pasárselo al rastreador. Devuelve 0 cuando no aplica (no es un
     * tag del HUB, no hay pose 3D confiable, o el resultado salió absurdo).
     *
     * <h2>El cálculo, paso a paso</h2>
     *
     * <pre>
     *   tagXY      posición del tag en el marco del robot
     *   normal     dirección hacia donde mira la cara del tag
     *   caraCentro tagXY, corrido lateralmente si el tag es el de la izquierda
     *   hubCentro  caraCentro + apotema · normal (hacia adentro del HUB)
     *
     *   corrección = −(rumbo(hubCentro) − rumbo(tagXY))
     * </pre>
     *
     * <h2>Auto-corrección del signo de la normal</h2>
     *
     * La convención de orientación que reporta {@code targetpose_robotspace}
     * varía entre firmwares, así que en vez de adivinarla se usa un invariante
     * físico: <b>el centro del HUB siempre está más lejos del robot que la cara
     * donde vive el tag.</b> Si al aplicar la normal el punto queda más cerca, el
     * signo era el contrario. Eso elimina por completo la adivinanza.
     */
    public static double computeAimCorrectionRad(Vision vision, int targetTagId) {
        if (!DemoState.isHubTargeting() || targetTagId < 0) {
            return 0.0;
        }
        boolean isCenter = DemoState.isHubCenterTag(targetTagId);
        boolean isLeft = DemoState.isHubLeftTag(targetTagId);
        if (!isCenter && !isLeft) {
            return 0.0;
        }

        Pose3d tagPose = vision.getTargetPoseRobotSpace(CAM);
        Translation2d tagXY = new Translation2d(tagPose.getX(), tagPose.getY());
        double distance = tagXY.getNorm();

        // Sin pose 3D confiable no hay corrección posible. Se apunta al tag y
        // se acepta el error: a distancias donde la pose 3D ya no sirve, el
        // error angular de esta corrección también es chico.
        if (distance < 0.5 || distance > DemoConstants.maxTrustedVisionDistanceMeters) {
            return 0.0;
        }

        // Normal de la cara, proyectada al plano horizontal.
        Translation2d normal = new Translation2d(1.0, new Rotation2d(tagPose.getRotation().getZ()));

        // Auto-corrección de signo con el invariante físico.
        Translation2d inward = normal.times(DemoConstants.hubFaceToCenterMeters);
        if (tagXY.plus(inward).getNorm() < distance) {
            inward = inward.unaryMinus();
            normal = normal.unaryMinus();
        }

        double bearingToTag = Math.atan2(tagXY.getY(), tagXY.getX());

        // Si el tag es el de la izquierda de la cara, primero hay que correrse
        // lateralmente hasta el centro de la cara.
        Translation2d faceCenter = tagXY;
        if (isLeft) {
            // Perpendicular a la normal, en el plano horizontal.
            Translation2d lateral = new Translation2d(-normal.getY(), normal.getX())
                    .times(DemoConstants.hubLeftTagLateralOffsetMeters);

            // "Izquierda" está definido desde el punto de vista de quien mira la
            // cara — o sea, desde la cámara. Así que el centro de la cara está a
            // la DERECHA del tag visto por la cámara, y a la derecha significa
            // menor rumbo. Elegimos el signo que cumple eso, en vez de fijar una
            // convención a mano.
            Translation2d optionA = tagXY.plus(lateral);
            Translation2d optionB = tagXY.minus(lateral);
            double relA = MathUtil.angleModulus(
                    Math.atan2(optionA.getY(), optionA.getX()) - bearingToTag);
            double relB = MathUtil.angleModulus(
                    Math.atan2(optionB.getY(), optionB.getX()) - bearingToTag);
            faceCenter = (relA < relB) ? optionA : optionB;
        }

        Translation2d hubCenter = faceCenter.plus(inward);
        double bearingToHub = Math.atan2(hubCenter.getY(), hubCenter.getX());

        // tx es positivo a la derecha; el rumbo es positivo en sentido CCW
        // (izquierda). Un punto de mira más a la izquierda significa un tx menor.
        double deltaBearing = MathUtil.angleModulus(bearingToHub - bearingToTag);
        double correction = -deltaBearing;

        // Red de seguridad: si la pose 3D vino basura, la corrección se dispara.
        return MathUtil.clamp(
                correction,
                -DemoConstants.hubAimMaxCorrectionRad,
                DemoConstants.hubAimMaxCorrectionRad);
    }
}
