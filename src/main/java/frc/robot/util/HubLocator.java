package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.DemoConstants;
import frc.robot.constants.VisionConstants;

/**
 * Centro del HUB calculado a partir del layout de AprilTags.
 *
 * <h2>Por qué</h2>
 *
 * La visión localiza al robot con {@link VisionConstants#aprilTagLayout}. Si el
 * centro del HUB al que apuntamos viene de OTRA fuente (números escritos a
 * mano), cualquier diferencia entre los dos es un error de apuntado que ni la
 * odometría ni la visión pueden corregir: el robot sabe perfecto dónde está y
 * apunta perfecto... a un punto que no es el centro. Además el layout cambia
 * entre campo soldado y AndyMark.
 *
 * <p>
 * El HUB tiene un tag centrado en cada una de sus cuatro caras, así que el
 * promedio de esos cuatro tags es su centro, en el MISMO marco que usa la
 * visión. Si algo sale raro (tags que no existen, o un centro a más de
 * {@link #maxDisagreementMeters} de los valores de respaldo) se usa el
 * respaldo y se avisa en consola.
 */
public final class HubLocator {

    /** Si el centro calculado difiere más que esto del respaldo, algo está mal. */
    private static final double maxDisagreementMeters = 0.5;

    private HubLocator() {
    }

    /**
     * @param red      {@code true} para el HUB rojo (tags 1-16), {@code false} azul.
     * @param fallback Coordenadas de respaldo.
     */
    public static Translation2d hubCenter(boolean red, Translation2d fallback) {
        String name = red ? "rojo" : "azul";
        try {
            double sx = 0.0;
            double sy = 0.0;
            int n = 0;
            for (int id : DemoConstants.hubCenterTagIds) {
                boolean isRedTag = id <= 16;
                if (isRedTag != red) {
                    continue;
                }
                Optional<Pose3d> tag = VisionConstants.aprilTagLayout.getTagPose(id);
                if (tag.isEmpty()) {
                    continue;
                }
                sx += tag.get().getX();
                sy += tag.get().getY();
                n++;
            }
            if (n < 4) {
                System.out.printf("[HubLocator] HUB %s: sólo %d tags de cara; uso respaldo %s%n",
                        name, n, fallback);
                return fallback;
            }
            Translation2d center = new Translation2d(sx / n, sy / n);
            double delta = center.getDistance(fallback);
            if (delta > maxDisagreementMeters) {
                System.out.printf("[HubLocator] HUB %s: centro por tags %s difiere %.3f m del respaldo; uso respaldo%n",
                        name, center, delta);
                return fallback;
            }
            System.out.printf("[HubLocator] HUB %s: centro por tags %s (respaldo %s, diferencia %.1f cm)%n",
                    name, center, fallback, delta * 100.0);
            return center;
        } catch (Exception e) {
            System.out.printf("[HubLocator] HUB %s: error (%s); uso respaldo%n", name, e);
            return fallback;
        }
    }
}
