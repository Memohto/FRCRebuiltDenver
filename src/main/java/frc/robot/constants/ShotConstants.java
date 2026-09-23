package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Física del tiro compartida por COMPETENCIA y DEMO.
 *
 * <p>
 * Estas constantes las consumen {@code ShotSolution}, {@code Shooter} y
 * {@code Drive}, que corren en los dos modos. Antes vivían en
 * {@code DemoConstants}, y eso hacía que el robot de competencia dependiera del
 * demo para saber cuánto tarda en volar una pelota. Ahora viven aquí y
 * {@code DemoConstants} sólo las re-exporta con el mismo nombre, para que la
 * guía del demo y su código sigan siendo válidos sin cambiar una línea.
 *
 * <p>
 * Regla: si una constante describe la <b>pelota o el mecanismo</b> (tiempo de
 * vuelo, deadbands de setpoint, rango de distancias), va aquí. Si describe
 * <b>cómo se comporta el robot en un modo</b> (velocidades de manejo, ganancias
 * de rumbo, barrido de la torreta), va en {@code CompetitionConstants} o en
 * {@code DemoConstants}.
 */
public final class ShotConstants {

    private ShotConstants() {
    }

    // ════════════════════════════════════════════════════════════════════════
    // Rango de distancias
    // ════════════════════════════════════════════════════════════════════════

    /** Distancia asumida cuando la medición vino NaN. Sólo para ese caso. */
    public static final double fallbackDistanceMeters = 3.0;

    /**
     * Rango físicamente sensato de distancia de tiro, en metros.
     *
     * <p>
     * Fuera de este rango la distancia se <b>recorta</b> al borde, no se
     * sustituye. Sustituirla por el fallback volvía el borde un escalón: pasar de
     * 0.51 a 0.49 m saltaba la potencia de la mínima a la de 3 m en un ciclo.
     */
    public static final double minShotDistanceMeters = 0.5;

    public static final double maxShotDistanceMeters = 12.0;

    // ════════════════════════════════════════════════════════════════════════
    // Tiempo de vuelo
    // ════════════════════════════════════════════════════════════════════════

    /**
     * Tiempo de vuelo de la pelota por distancia, en segundos.
     *
     * <p>
     * <b>Es lo único que hay que medir para que el disparo en movimiento
     * funcione bien.</b> Graba un tiro estático desde el costado a 60 fps, cuenta
     * los cuadros entre que la pelota sale del shooter y toca el HUB, y divide
     * entre 60. Repite a 2, 3, 4 y 5 m.
     *
     * <p>
     * Los valores actuales son una estimación para un tiro en arco a las
     * velocidades de {@code ShooterConstants.kShooterFlywheelMap}. Crece con la
     * distancia porque la pelota recorre más y va perdiendo velocidad.
     */
    public static final InterpolatingDoubleTreeMap kTimeOfFlightMap = new InterpolatingDoubleTreeMap();
    static {
        kTimeOfFlightMap.put(1.0, 0.40);
        kTimeOfFlightMap.put(2.0, 0.53);
        kTimeOfFlightMap.put(3.0, 0.67);
        kTimeOfFlightMap.put(4.0, 0.81);
        kTimeOfFlightMap.put(5.0, 0.95);
    }

    // ════════════════════════════════════════════════════════════════════════
    // Disparo en movimiento (objetivo virtual)
    // ════════════════════════════════════════════════════════════════════════
    //
    //     objetivoVirtual = objetivoReal − velocidad × tiempoDeVuelo
    //
    // El ángulo se consume como DELTA (ShotSolution.aimOffsetRad) y la distancia
    // al objetivo virtual va a los mapas de hood y flywheel. Ver ShotSolution.

    /**
     * Interruptor maestro del disparo en movimiento.
     *
     * <p>
     * Quedó encendido desde la última iteración del demo. <b>Verifícalo en el
     * robot antes de competir</b> con el procedimiento de la guía: manejando a
     * la derecha frente al HUB, {@code Shot/AimOffsetDeg} (y {@code Match/Turret/AimOffsetDeg}, lo que la torreta aplicó) debe ser
     * positivo. Si dudas, apágalo: con esto en false el robot tira exactamente
     * como en Denver.
     */
    public static final boolean shootWhileMovingEnabled = true;

    /** Ganancia. 1.0 = compensación teórica completa. Se sube de 0.2 en 0.2. */
    public static final double shootWhileMovingGain = 1.0;

    /** Bajo esta velocidad (m/s) no se compensa nada. Evita ruido en reposo. */
    public static final double shootWhileMovingMinSpeed = 0.1;

    /**
     * Signo de la corrección angular. <b>Aquí se invierte, y sólo aquí.</b>
     *
     * <p>
     * Con {@code 1.0}, moverte a la derecha corre el apuntado a la izquierda.
     * Antes de invertirlo mira el número: si {@code AimOffsetDeg} sale POSITIVO
     * manejando a la derecha y los tiros aún se van a la derecha, el signo está
     * bien y falta ganancia. Si sale NEGATIVO, pon {@code -1.0} aquí.
     */
    public static final double shootWhileMovingAimSign = 1.0;

    /** Iteraciones de convergencia del objetivo virtual. Dos sobran en FRC. */
    public static final int shootWhileMovingIterations = 2;

    /** Tope de la corrección, en metros. Red de seguridad contra velocidad basura. */
    public static final double shootWhileMovingMaxCompensationMeters = 2.5;

    /**
     * Tope de la corrección, en ángulo.
     *
     * <p>
     * 2.5 m son ~15° a 9 m pero más de 60° a 2 m. Este tope evita que la torreta
     * se vaya a medio campo cuando estás pegado al HUB.
     */
    public static final double shootWhileMovingMaxAimOffsetRad = Math.toRadians(20.0);

    // ════════════════════════════════════════════════════════════════════════
    // Filtros y deadbands de mecanismo
    // ════════════════════════════════════════════════════════════════════════

    /**
     * Filtro pasa-bajas de la velocidad de campo. 1.0 = sin filtrar.
     *
     * <p>
     * A 50 Hz la constante de tiempo es {@code 0.02/alpha}: 0.2 son ~100 ms. Si
     * la compensación tiembla, bájalo; si se siente retrasada al arrancar y
     * frenar, súbelo.
     */
    public static final double fieldVelocityFilterAlpha = 0.2;

    /**
     * Deadband del setpoint del hood, en grados.
     *
     * <p>
     * Cada setpoint nuevo reinicia el perfil de Motion Magic. Re-comandar a 50 Hz
     * con cambios de milésimas de grado hace que el perfil nunca se complete y el
     * mecanismo tiemble en su lugar.
     */
    public static final double hoodSetpointDeadbandDeg = 0.25;

    /** Deadband del setpoint del flywheel, en RPS. Misma razón que el hood. */
    public static final double flywheelSetpointDeadbandRPS = 0.25;

    /**
     * Cuánto tiempo se le sigue creyendo a la odometría sin una corrección de
     * visión aceptada, en segundos.
     *
     * <p>
     * En competencia la Limelight ve tags casi todo el tiempo y esto casi nunca
     * expira. Cuando expira, el dashboard lo avisa ("ODOMETRÍA VIEJA") pero la
     * torreta <b>sigue apuntando por odometría</b>: un swerve con Pigeon 2.0
     * deriva poco y disparar a la última pose conocida es mejor que no disparar.
     */
    public static final double odometryTrustSeconds = 8.0;
}
