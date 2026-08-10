package frc.robot.constants;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.geometry.Translation2d;

public class RobotConstants {
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }
    public static enum RobotMode {
        BOMBER,
        STRIKER
    }
    public static enum DriveMode {
        ORBIT,
        FEEDER,
        NORMAL
    }
    public static enum TurretMode {
        HUB_TRACKER,
        DS_TRACKER,
        NORMAL
    }

    // ════════════════════════════════════════════════════════════════════════
    // DEMO MODE
    // ════════════════════════════════════════════════════════════════════════

    /**
     * Interruptor maestro entre el robot de competencia y el de demo.
     *
     * <p>
     * {@code true} → se cargan los default commands y bindings de demo
     * (Smooth Drive, STRIKER/BOMBER, follow-me, tiro suave).
     * <br>
     * {@code false} → se carga exactamente el robot de competencia de siempre,
     * sin ningún cambio de comportamiento.
     *
     * <p>
     * Nada del código de competencia fue borrado ni modificado. Cambiar esta
     * línea a {@code false} y desplegar te devuelve el robot de temporada.
     */
    public static final boolean isDemoMode = true;

    /**
     * Un solo control para todo.
     *
     * <p>
     * {@code true} → el control del PILOTO (puerto 0) recibe el mapeo completo
     * del operador MÁS el chasis en sus joysticks. El puerto 1 queda sin usar.
     * Pensado para probar solo, sin depender de que haya dos personas.
     *
     * <p>
     * Los cambios respecto al mapeo de dos controles:
     * <ul>
     * <li>El toggle de orientación (era X del piloto) se va a <b>Start</b>, y
     * con eso desaparece el ciclado manual de pipelines de la Limelight.</li>
     * <li>Fijar el frente (era B del piloto) se va a <b>POV izquierda</b>.</li>
     * <li>El modo precisión (era LT del piloto) <b>desaparece</b>, porque LT es
     * los rodillos del intake.</li>
     * </ul>
     */
    public static final boolean isSoloDemo = true;

    /** Los dos modos de operación del demo. */
    public static enum DemoMode {
        /**
         * Torreta cazando AprilTags, chasis 100% manual.
         * La cámara va montada en la torreta, así que puede buscar en 360°
         * sin depender de la posición en cancha.
         */
        STRIKER,

        /**
         * Torreta fija en cero (la cámara queda rígida respecto al chasis),
         * chasis asistido: orbita el HUB o sigue un target.
         */
        BOMBER
    }

    /** Sub-modo de STRIKER: a qué le apunta la torreta. */
    public static enum StrikerTargeting {
        /**
         * <b>Default.</b> Sólo acepta AprilTags del HUB, apunta al centro del
         * HUB calculado vectorialmente, y dispara con los mapas de tiro de
         * COMPETENCIA. Es lo que está activo al dar enable.
         */
        HUB,

        /**
         * <b>Smart feature.</b> Caza cualquier AprilTag que aparezca y dispara
         * suave. Es el modo de "sostén un tag y te llega una pelotita".
         */
        GLOBAL_HUNT
    }

    /** Sub-modo de BOMBER: qué hace el chasis. */
    public static enum BomberTask {
        /** Orbita el HUB manteniendo la trasera apuntada. Modo volumen. */
        HUB_ORBIT,
        /** Sigue un AprilTag manteniendo distancia. El "perrito". */
        FOLLOW_TARGET,

        /**
         * Volcado suave: dispara con ambos cañones exactamente hacia donde
         * apunta el robot, sin corregir dirección.
         *
         * <p>
         * El chasis no se alinea con nada — el piloto apunta con el robot. El
         * hood se va a su ángulo más plano para que la pelota salga hacia
         * adelante y no en arco, y los dos flywheels giran a una potencia fija.
         *
         * <p>
         * Sirve para mover FUEL de un lado a otro sin que el robot esté
         * calculando nada.
         */
        SMOOTH_DUMP
    }

    public static final double robotLoopFrequencyHz = 50.0;
    public static final double highPriorityFrequencyHz = robotLoopFrequencyHz;
    public static final double lowPriorityFrequencyHz = robotLoopFrequencyHz / 2;

    public static final Translation2d  blueHub = new Translation2d(4.625, 4.035);
    public static final Translation2d  redHub = new Translation2d(11.917, 4.035);
}