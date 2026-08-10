package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.DemoConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.BomberTask;
import frc.robot.constants.RobotConstants.DemoMode;
import frc.robot.constants.RobotConstants.StrikerTargeting;

/**
 * Estado global del Demo Mode.
 *
 * <p>
 * Todo el estado de modos vive aquí y en ningún otro lado, y cada cambio se
 * loguea, así que en AdvantageScope se puede ver exactamente en qué modo estaba
 * el robot en cada frame.
 *
 * <h2>Estado al dar enable</h2>
 *
 * <pre>
 *   Modo:    STRIKER
 *   Objetivo: HUB, con los mapas de tiro de COMPETENCIA
 * </pre>
 *
 * Es decir: al habilitar, el robot está en la configuración de competencia
 * calibrada. El tiro suave y la caza libre de AprilTags son la <i>smart
 * feature</i>, no el default.
 */
public class DemoState {

    private static DemoMode mode = DemoMode.STRIKER;
    private static StrikerTargeting strikerTargeting = StrikerTargeting.HUB;
    private static BomberTask bomberTask = BomberTask.HUB_ORBIT;

    /** ID fijado en GLOBAL_HUNT / follow-me. -1 = cualquiera. */
    private static int lockedTagId = -1;

    /**
     * Orientación del manejo. Alterna con X en el control del piloto.
     *
     * <ul>
     * <li><b>FIELD</b> — igual que competencia: field-relative usando la
     * rotación de la odometría, con flip por alianza. El "adelante" del stick es
     * el adelante del campo.</li>
     * <li><b>DRIVER</b> — el piloto fija su propio frente con B, y ese frente
     * es inmune a las correcciones de visión.</li>
     * </ul>
     */
    public enum DriveOrientation {
        FIELD,
        DRIVER
    }

    private static DriveOrientation driveOrientation = DriveOrientation.FIELD;

    /**
     * Rotación que el piloto declaró como "adelante" en modo DRIVER.
     *
     * <p>
     * <b>Éste es el arreglo al bug de "presiono B y cuando ve un AprilTag se me
     * mueve el frente completo".</b>
     *
     * <p>
     * El problema era que B hacía {@code drive.setPose()}, que reescribe la
     * rotación de la odometría. Un segundo después llegaba una corrección de
     * visión y devolvía la pose a la verdad del campo — llevándose el frente que
     * el piloto acababa de fijar.
     *
     * <p>
     * La solución es no pelearse con el pose estimator: en vez de reescribir la
     * pose, se guarda un <b>offset</b>. La visión sigue corrigiendo la odometría
     * (que es lo que el apuntado necesita) y el frente del piloto se mantiene
     * estable porque se calcula como {@code rotaciónActual − offset}.
     */
    private static Rotation2d driverFrontOffset = Rotation2d.kZero;

    private DemoState() {
    }

    // ── Getters ─────────────────────────────────────────────────────────────

    public static DemoMode getMode() {
        return mode;
    }

    public static StrikerTargeting getStrikerTargeting() {
        return strikerTargeting;
    }

    public static BomberTask getBomberTask() {
        return bomberTask;
    }

    public static int getLockedTagId() {
        return lockedTagId;
    }

    public static boolean isStriker() {
        return mode == DemoMode.STRIKER;
    }

    public static boolean isBomber() {
        return mode == DemoMode.BOMBER;
    }

    public static boolean isFollowing() {
        return mode == DemoMode.BOMBER && bomberTask == BomberTask.FOLLOW_TARGET;
    }

    /**
     * ¿El volcado suave está activo?
     *
     * <p>
     * En este modo el robot NO corrige dirección: dispara con ambos cañones
     * exactamente hacia donde apunta, con hood plano y potencia fija.
     */
    public static boolean isSmoothDump() {
        return mode == DemoMode.BOMBER && bomberTask == BomberTask.SMOOTH_DUMP;
    }

    /** ¿Estamos apuntándole al HUB (y por lo tanto con potencia de competencia)? */
    public static boolean isHubTargeting() {
        if (mode == DemoMode.BOMBER) {
            // El volcado suave NO apunta a nada: tiene su propia potencia fija.
            return bomberTask == BomberTask.HUB_ORBIT;
        }
        return strikerTargeting == StrikerTargeting.HUB;
    }

    /** ¿Se permite potencia de competencia en este estado? */
    public static boolean allowsCompetitionPower() {
        // Sólo cuando le apuntamos al HUB. En cualquier estado donde la torreta
        // esté rastreando un tag suelto, del otro lado casi siempre hay una
        // persona sosteniéndolo.
        return isHubTargeting();
    }

    /**
     * ¿Este tag es un objetivo válido ahora mismo?
     *
     * <ul>
     * <li><b>HUB</b> — sólo tags del HUB. Un tag suelto que alguien traiga en la
     * mano no distrae a la torreta.</li>
     * <li><b>GLOBAL_HUNT</b> — cualquier tag, o sólo el fijado si hay uno.</li>
     * </ul>
     */
    public static boolean acceptsTag(int tagId) {
        if (tagId < 0) {
            return false;
        }
        if (isHubTargeting() && mode == DemoMode.STRIKER) {
            return isHubTag(tagId);
        }
        if (lockedTagId >= 0) {
            return tagId == lockedTagId;
        }
        return true;
    }

    // ── Clasificación de tags del HUB ───────────────────────────────────────

    public static boolean isHubCenterTag(int tagId) {
        for (int id : DemoConstants.hubCenterTagIds) {
            if (id == tagId) {
                return true;
            }
        }
        return false;
    }

    public static boolean isHubLeftTag(int tagId) {
        for (int id : DemoConstants.hubLeftTagIds) {
            if (id == tagId) {
                return true;
            }
        }
        return false;
    }

    public static boolean isHubTag(int tagId) {
        return isHubCenterTag(tagId) || isHubLeftTag(tagId);
    }

    // ── Transiciones ────────────────────────────────────────────────────────

    /** Alterna STRIKER ↔ BOMBER y vuelve a los sub-modos por defecto. */
    public static void toggleMode() {
        mode = (mode == DemoMode.STRIKER) ? DemoMode.BOMBER : DemoMode.STRIKER;
        strikerTargeting = StrikerTargeting.HUB;
        bomberTask = BomberTask.HUB_ORBIT;
        lockedTagId = -1;
        log();
    }

    /**
     * Alterna la smart feature del modo activo.
     *
     * <p>
     * <b>Siempre alterna</b>, sin condiciones. La versión anterior sólo entraba
     * a fijar tag si había uno visible en ese instante, así que presionar el
     * botón muchas veces no hacía nada visible y parecía descompuesto.
     *
     * <ul>
     * <li>STRIKER: HUB ↔ GLOBAL_HUNT</li>
     * <li>BOMBER: orbitar HUB ↔ follow-me</li>
     * </ul>
     *
     * @param currentlyVisibleTagId Si hay un tag a la vista al activar
     *                              GLOBAL_HUNT o el follow-me, se fija ese. Si no
     *                              hay ninguno, queda en modo "cualquiera" y se
     *                              engancha al primero que encuentre.
     */
    public static void toggleSmartFeature(int currentlyVisibleTagId) {
        if (mode == DemoMode.STRIKER) {
            if (strikerTargeting == StrikerTargeting.HUB) {
                strikerTargeting = StrikerTargeting.GLOBAL_HUNT;
                lockedTagId = currentlyVisibleTagId;
            } else {
                strikerTargeting = StrikerTargeting.HUB;
                lockedTagId = -1;
            }
        } else {
            if (bomberTask == BomberTask.HUB_ORBIT) {
                // En modo solo la smart feature de BOMBER es el cero odometría;
                // en dos controles sigue siendo el perrito.
                // En modo solo la smart feature de BOMBER es el volcado suave;
                // en dos controles sigue siendo el perrito.
                bomberTask = RobotConstants.isSoloDemo
                        ? BomberTask.SMOOTH_DUMP
                        : BomberTask.FOLLOW_TARGET;
                lockedTagId = currentlyVisibleTagId;
            } else {
                bomberTask = BomberTask.HUB_ORBIT;
                lockedTagId = -1;
            }
        }
        log();
    }

    /** Vuelve al estado seguro. Se llama en teleopInit y al deshabilitar. */
    public static void reset() {
        mode = DemoMode.STRIKER;
        strikerTargeting = StrikerTargeting.HUB;
        bomberTask = BomberTask.HUB_ORBIT;
        lockedTagId = -1;
        // La orientación del manejo NO se resetea: el piloto la eligió a
        // propósito y no tiene por qué perderla al deshabilitar entre demos.
        log();
    }

    // ── Orientación del manejo ──────────────────────────────────────────────

    public static DriveOrientation getDriveOrientation() {
        return driveOrientation;
    }

    public static boolean isDriverOriented() {
        return driveOrientation == DriveOrientation.DRIVER;
    }

    /** Alterna FIELD ↔ DRIVER. Botón X del piloto. */
    public static void toggleDriveOrientation() {
        driveOrientation = (driveOrientation == DriveOrientation.FIELD)
                ? DriveOrientation.DRIVER
                : DriveOrientation.FIELD;
        log();
    }

    /**
     * Fija el frente del piloto a la orientación actual del robot.
     *
     * <p>
     * No toca la odometría — sólo guarda el offset. Por eso sobrevive a las
     * correcciones de visión.
     */
    public static void captureDriverFront(Rotation2d currentRotation) {
        driverFrontOffset = currentRotation;
        log();
    }

    public static Rotation2d getDriverFrontOffset() {
        return driverFrontOffset;
    }

    public static void log() {
        Logger.recordOutput("Demo/Mode", mode);
        Logger.recordOutput("Demo/StrikerTargeting", strikerTargeting);
        Logger.recordOutput("Demo/BomberTask", bomberTask);
        Logger.recordOutput("Demo/LockedTagId", lockedTagId);
        Logger.recordOutput("Demo/HubTargeting", isHubTargeting());
        Logger.recordOutput("Demo/DriveOrientation", driveOrientation);
        Logger.recordOutput("Demo/DriverFrontOffsetDeg", driverFrontOffset.getDegrees());
    }
}
