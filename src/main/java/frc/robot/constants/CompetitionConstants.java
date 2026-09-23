package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;

/**
 * Todo lo ajustable del robot de COMPETENCIA (v2).
 *
 * <p>
 * Es el equivalente de {@code DemoConstants} para el modo de partido. Lo que
 * está aquí se probó primero en el demo del StingyCamp y se migró con los
 * valores cambiados a "modo partido": velocidad completa, aceleraciones más
 * agresivas y sin las protecciones que sólo tienen sentido con alumnos
 * alrededor.
 *
 * <p>
 * La física del tiro (tiempo de vuelo, disparo en movimiento, deadbands) NO
 * está aquí: vive en {@link ShotConstants} porque es la misma en los dos
 * modos.
 */
public final class CompetitionConstants {

    private CompetitionConstants() {
    }

    // ════════════════════════════════════════════════════════════════════════
    //
    //   C O N T R O L E S
    //
    // ════════════════════════════════════════════════════════════════════════

    /**
     * Un solo control para todo.
     *
     * <p>
     * {@code true} → el control del PILOTO (puerto 0) recibe el mapeo completo
     * del operador MÁS el chasis en sus joysticks. El puerto 1 queda sin usar.
     * Para probar solo en el taller sin depender de dos personas.
     *
     * <p>
     * {@code false} → dos controles: piloto en el puerto 0, operador en el 1.
     * Es lo que va en partido.
     *
     * <p>
     * Cambios en modo solo respecto al mapeo de dos controles (los mismos que en
     * el demo):
     * <ul>
     * <li>Fijar el frente (era B del piloto) se va a <b>POV izquierda</b>, porque
     * B es retraer intake.</li>
     * <li>El modo precisión (era LT del piloto) <b>desaparece</b>, porque LT es
     * los rodillos del intake.</li>
     * </ul>
     */
    public static final boolean isSoloDriver = true;

    // ════════════════════════════════════════════════════════════════════════
    //
    //   M A N E J O   —   Smooth Drive a velocidad de partido
    //
    // ════════════════════════════════════════════════════════════════════════

    /**
     * Velocidad máxima de traslación como fracción de
     * {@code TunerConstants.kSpeedAt12Volts}. 1.0 = todo lo que da el chasis.
     */
    public static final double maxSpeedFraction = 1.0;

    /** Fracción de velocidad en modo precisión (LT del piloto). */
    public static final double precisionSpeedFraction = 0.30;

    /**
     * Velocidad angular máxima comandada por el stick, en rad/s.
     *
     * <p>
     * El chasis puede dar ~20 rad/s teóricos; nadie los quiere en un stick. 8
     * rad/s son ~1.3 vueltas por segundo, suficiente para girar rápido sin que
     * el giro se sienta incontrolable.
     */
    public static final double maxAngularSpeedRadPerSec = 8.0;

    /**
     * Límites de aceleración de traslación, en fracción de velocidad máxima por
     * segundo (1.0/s = de cero a tope en un segundo).
     *
     * <p>
     * Es la protección de transmisión de la que habla la guía del demo: un
     * driver experimentado también le da jalones al stick en una final. Con
     * ~2.5/s el robot llega a tope en 0.4 s, que el driver no nota, y las
     * llantas no patinan al arrancar. Si el robot se siente "flotado" al
     * arrancar, súbelo; si patina o el driver se queja de lag, revisa primero
     * {@code translationSmoothingTau}.
     */
    public static final double translationMaxAccel = 2.5;

    /** Desaceleración. Puede ser mayor que la aceleración: frenar no patina igual. */
    public static final double translationMaxDecel = 4.0;

    /**
     * Constante de tiempo del suavizado de la tasa, en segundos. Más chico =
     * responde más rápido. 0.08 s es casi imperceptible en la mano.
     */
    public static final double translationSmoothingTau = 0.08;

    public static final double rotationMaxAccel = 8.0;
    public static final double rotationMaxDecel = 10.0;
    public static final double rotationSmoothingTau = 0.06;

    /** Deadband del stick, aplicado sobre la MAGNITUD del vector. */
    public static final double joystickDeadband = 0.08;

    /**
     * Curva del stick. 1.0 = lineal, 2.0 = cuadrática (lo que usaba Denver).
     * 1.5 da precisión a baja velocidad sin regalar la mitad del recorrido.
     */
    public static final double joystickExponent = 1.5;

    // ════════════════════════════════════════════════════════════════════════
    //
    //   R U M B O   A S I S T I D O   (BOMBER: chasis apunta al objetivo)
    //
    // ════════════════════════════════════════════════════════════════════════
    //
    // Es el controlador del demo, sin cambios de estructura: la salida ya está
    // en rad/s y NO se re-escala (el bug de Denver era multiplicarla por
    // getMaxAngularSpeedRadPerSec() ≈ 20 rad/s, que saturaba al instante).

    /** Ganancia proporcional, en rad/s por radián de error. */
    public static final double headingKp = 3.0;

    /** Ganancia derivativa. Actúa sobre el derivativo FILTRADO. */
    public static final double headingKd = 0.35;

    /** Filtro del derivativo (0-1). Menor = más filtrado. */
    public static final double headingDerivativeAlpha = 0.15;

    /** Dentro de esta tolerancia el controlador no comanda nada. */
    public static final double headingToleranceRad = Math.toRadians(1.0);

    /**
     * Ancho de la rampa suave arriba de la tolerancia. La salida se escala de 0
     * a 1 a lo largo de esta zona en vez de cortarse de golpe.
     */
    public static final double headingSoftZoneRad = Math.toRadians(3.0);

    /** Velocidad angular máxima que puede pedir la asistencia, en rad/s. */
    public static final double headingMaxOmegaRadPerSec = 5.0;

    /**
     * Distancia mínima al objetivo para calcular un rumbo por odometría.
     * Debajo, centímetros de error de pose son decenas de grados de rumbo.
     */
    public static final double headingMinTargetDistanceMeters = 0.8;

    /**
     * Límite de cambio del rumbo objetivo, en rad/s. Una pose basura de un ciclo
     * no puede mandar al robot a dar la vuelta.
     */
    public static final double headingTargetSlewRadPerSec = 4.0;

    /**
     * Mientras el piloto gira con el stick, ¿la asistencia cede?
     *
     * <p>
     * {@code true} = si el piloto mueve el stick de giro más de
     * {@link #headingOverrideThreshold}, la asistencia se apaga ese ciclo y el
     * giro es manual. Es la válvula de escape si el robot se pone a apuntar a un
     * HUB imaginario por una pose mala.
     */
    public static final boolean headingDriverOverride = true;

    public static final double headingOverrideThreshold = 0.5;

    // ════════════════════════════════════════════════════════════════════════
    //
    //   T O R R E T A   —   apuntado continuo por odometría
    //
    // ════════════════════════════════════════════════════════════════════════
    //
    // En competencia la torreta apunta POR POSE: sabe dónde está el HUB (es una
    // constante del campo) y dónde está el robot (pose estimator con encoders +
    // Pigeon + MegaTag2). La Limelight fija ya no ve lo mismo que la torreta,
    // así que no hay lazo sobre tx: la visión corrige la POSE, no la torreta.

    /**
     * Filtro del setpoint (0-1). 1.0 = sin filtrar. El ángulo viene de la pose,
     * que es suave, así que no hace falta filtrar mucho; el filtro sólo lima el
     * ruido de las correcciones de visión.
     */
    public static final double turretSetpointFilterAlpha = 0.6;

    /**
     * Velocidad máxima a la que se mueve el SETPOINT, en rad/s.
     *
     * <p>
     * Es independiente de la velocidad del mecanismo (Motion Magic). Limita qué
     * tan rápido puede cambiar lo que se le pide, para que una corrección de
     * pose de 20° no se convierta en un latigazo. 4 rad/s son ~230°/s: más de lo
     * que la torreta necesita para seguir el HUB manejando a tope.
     */
    public static final double turretMaxSetpointRateRadPerSec = 4.0;

    /**
     * Cambio mínimo de setpoint para re-comandar el motor, en radianes.
     *
     * <p>
     * Cada setpoint nuevo reinicia el perfil de Motion Magic. 0.15° es
     * suficientemente chico para que el apuntado no se note escalonado y
     * suficientemente grande para que el ruido de la pose no lo dispare.
     */
    public static final double turretSetpointDeadbandRad = Math.toRadians(0.15);

    /**
     * Distancia mínima al objetivo para recalcular el ángulo de torreta. Pegado
     * al HUB se congela el último ángulo bueno.
     */
    public static final double turretAimMinDistanceMeters = 0.7;

    /** Tolerancia para declarar "torreta en posición" (LISTO PARA TIRAR). */
    public static final double turretOnTargetToleranceRad = Math.toRadians(2.0);

    /**
     * Ganancias del motor de rotación para competencia v2.
     *
     * <p>
     * Sin término integral: el kI = 0.1 de Denver acumulaba error mientras el
     * perfil de Motion Magic estaba en camino y luego sobrepasaba — cacería de
     * baja frecuencia. Para SEGUIR un objetivo no hace falta integral; kS y kV
     * ya cubren la fricción y la velocidad. Son las ganancias del demo con un
     * poco más de kP porque en partido no hay que ser tan suave.
     */
    public static final boolean useV2TurretGains = true;

    public static final Slot0Configs turretGains = new Slot0Configs()
            .withKP(45.0)
            .withKI(0.0)
            .withKD(3.0)
            .withKS(0.35)
            .withKV(4.2);

    /** Perfil de Motion Magic de la torreta en partido, en rot/s y rot/s². */
    public static final double turretCruiseRotPerSec = 2.0;
    public static final double turretAccelRotPerSecSec = 5.0;

    // ════════════════════════════════════════════════════════════════════════
    //
    //   O B J E T I V O S
    //
    // ════════════════════════════════════════════════════════════════════════

    /**
     * Coordenada X del punto de pase (modo FEEDER), por alianza.
     *
     * <p>
     * En FEEDER el objetivo es un punto sobre la línea de la alianza a la misma Y
     * del robot: el robot "tira hacia su pared". Son los mismos números que
     * usaba Denver.
     */
    public static final double feederTargetXBlue = 2.0;
    public static final double feederTargetXRed = 14.0;

    // ════════════════════════════════════════════════════════════════════════
    //
    //   C Á M A R A   D E   P I L O T O   (HBVCAM por USB al roboRIO)
    //
    // ════════════════════════════════════════════════════════════════════════
    //
    // Una cámara USB en el roboRIO NO sirve para AprilTags: el Rio no tiene
    // CPU para detectarlos a una tasa útil sin tirar el loop del robot. Aquí es
    // sólo video para el piloto en Elastic. Si algún día va a un Orange Pi con
    // PhotonVision, se conecta con VisionIOPhotonVision y pasa a ser fuente de
    // odometría.

    /** Arrancar el stream de la cámara USB del roboRIO. */
    public static final boolean useDriverCamera = true;

    /**
     * Resolución y FPS del stream. Bajos a propósito: el FMS limita el ancho de
     * banda a 4 Mbps y el Rio comprime en software. 320x240 a 15 fps ronda 1
     * Mbps y cuesta ~10% de CPU.
     */
    public static final int driverCameraWidth = 320;
    public static final int driverCameraHeight = 240;
    public static final int driverCameraFps = 15;
}
