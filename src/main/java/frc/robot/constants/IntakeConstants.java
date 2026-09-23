package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class IntakeConstants {
    public static final double pinionRadiusMeters = Units.inchesToMeters(0.5);

    // ── Rollers ────────────────────────────────────────────────────────────────
    public static final int rollersCanId = 21;
    public static final Slot0Configs rollersGains =
        new Slot0Configs().withKP(0.1).withKI(0).withKD(0);
    public static final double rollersGearRatio = (20/12);
    public static final double rollersSpeed = 0.8;
    public static final double rollersStatorCurrentLimitAmps = 100;
    public static final double rollersSupplyCurrentLimitAmps = 30;
    public static final boolean rollersInverted = false;

    // ── Extensor ───────────────────────────────────────────────────────────────
    public static final int extensorCanId = 20;
    public static final Slot0Configs extensorGains =
        new Slot0Configs().withKP(10).withKI(0).withKD(0).withKS(0).withKV(0.124);
    public static final double extensorGearRatio = (54/12) * (36/18);
    public static final double extensorSpeed = 0.24;
    public static final double extensorStatorCurrentLimitAmps = 40;
    public static final double extensorSupplyCurrentLimitAmps = 20;
    public static final boolean extensorInverted = false;

    // ════════════════════════════════════════════════════════════════════════
    //  L Í M I T E S   D E L   I N T A K E
    //
    //  TODO ESTO SE TUNEA AQUÍ. Antes los soft limits estaban escritos a mano
    //  dentro de IntakeIOTalonFX, en DOS lugares distintos (el constructor y
    //  setSoftwareLimit), y no coincidían con estas constantes. Ya no: el IO
    //  lee de aquí.
    //
    //  Todo está en RADIANES DEL EXTENSOR.
    // ════════════════════════════════════════════════════════════════════════

    /** Posición objetivo al extender. Súbela si al intake le falta salida. */
    public static final Rotation2d extendedRotation = Rotation2d.fromRadians(20);

    /** Posición objetivo al retraer (caja cerrada). */
    public static final Rotation2d extendedRotationReversed = Rotation2d.fromRadians(3.5);

    /**
     * Soft limit hacia afuera. Tiene que ser MAYOR que {@code extendedRotation},
     * si no el mecanismo nunca llega a la posición comandada.
     */
    public static final double extensorForwardLimitRad = 21.5;

    /**
     * Soft limit hacia adentro. Tiene que ser MENOR que
     * {@code extendedRotationReversed}.
     */
    public static final double extensorReverseLimitRad = 2.0;

    // ── Homing por corriente (extensión) ────────────────────────────────────
    //
    // En vez de confiar en una posición calibrada a mano, el robot empuja el
    // extensor hacia afuera hasta que siente el tope mecánico y se queda ahí.
    // Resuelve el problema de "si no sale por completo no le deja meter bien
    // las pelotas" sin depender de que nadie recalibre nada.

    /** Activa el remate por corriente al final de cada extensión. */
    public static final boolean useStallHoming = true;

    /**
     * Duty cycle del empuje final contra el tope. Bajo a propósito.
     *
     * <p>
     * No es para mover el mecanismo rápido —para eso está el lazo de posición—
     * sino para recorrer los últimos milímetros y detectar el tope sin
     * golpearlo. 0.12 mueve el rack despacio y hace el pico de corriente
     * claramente distinguible del arranque.
     */
    public static final double stallHomingSpeed = 0.12;

    /**
     * Corriente de estator a partir de la cual se considera que topó.
     *
     * <p>
     * Tiene que estar cómodamente arriba de la corriente de movimiento libre y
     * cómodamente abajo del límite de estator (40 A). <b>Cómo medirlo:</b> extiende
     * el intake al aire y mira {@code Intake/extensorCurrentAmps} en
     * AdvantageScope; luego bloquéalo con la mano y mira el pico. Pon este valor
     * a la mitad entre ambos.
     */
    public static final double stallCurrentAmps = 18.0;

    /**
     * Cuánto tiempo debe sostenerse la corriente para declarar tope.
     *
     * <p>
     * Sin este debounce, el pico de arranque del motor se confundiría con el
     * tope y el homing terminaría de inmediato sin haberse movido.
     */
    public static final double stallDebounceSeconds = 0.12;

    /** Tiempo máximo del empuje. Si no topa en este tiempo, se rinde. */
    public static final double stallTimeoutSeconds = 1.2;

    /**
     * Cuánto se retrae después de topar, para no quedar forzando el tope.
     *
     * <p>
     * Dejar el mecanismo empujando contra su propio límite calienta el motor y
     * desgasta el rack. Esto lo despega apenas.
     */
    public static final double stallBackoffRad = 0.3;

    // ════════════════════════════════════════════════════════════════════════
    //  B O O S T   D E   R O D I L L O S   (más potencia cuando la pide)
    //
    //  El rodillo gira a rollersSpeed (0.8) de base. Cuando la caja va llena y
    //  el rodillo tiene que empujar una pelota contra las demás, se frena y la
    //  corriente sube: eso es "está pidiendo potencia". Mientras dure, el
    //  duty cycle sube a rollersBoostSpeed, y cuando la carga se va regresa
    //  solo a la base. Es el mismo principio del remate por corriente del
    //  extensor, pero continuo en vez de una sola vez.
    //
    //  Por qué ayuda: a duty fijo, el par disponible cae conforme el motor se
    //  frena (menos voltaje neto sobre la resistencia). Subir el duty al 100%
    //  cuando ya está frenado da ~25% más de par justo cuando hace falta, sin
    //  correr al 100% todo el tiempo (que se come las pelotas de más y calienta).
    // ════════════════════════════════════════════════════════════════════════

    /** Activa el boost por carga en los rodillos. */
    public static final boolean rollersBoostEnabled = true;

    /** Duty cycle durante el boost. 1.0 = todo lo que da la batería. */
    public static final double rollersBoostSpeed = 1.0;

    /**
     * Corriente de estator a partir de la cual el rodillo "está pidiendo".
     *
     * <p>
     * <b>Cómo medirlo:</b> corre el intake al aire (LT) y mira
     * {@code Intake/rollersCurrentAmps} en AdvantageScope: eso es la corriente
     * libre (típicamente 8-20 A en un Kraken X44). Luego mete pelotas hasta que
     * la caja esté llena y mira cuánto sube cuando le cuesta agarrar una. Pon
     * este valor a la mitad entre los dos. Tiene que quedar abajo del límite de
     * estator (100 A) por bastante margen.
     */
    public static final double rollersBoostCurrentAmps = 45.0;

    /**
     * Cuánto debe sostenerse la corriente para activar el boost. Filtra el
     * pico de arranque y el golpe de una sola pelota que ya entró sola.
     */
    public static final double rollersBoostDebounceSeconds = 0.06;

    /**
     * Segunda señal de carga: velocidad. Si el rodillo cae por debajo de esta
     * fracción de su velocidad libre (aprendida los primeros
     * {@code rollersBoostLearnSeconds} de cada activación), también se activa
     * el boost. Es más rápida que la corriente cuando la batería anda baja.
     * 0.0 = desactiva esta señal.
     */
    public static final double rollersBoostVelocityDroopFraction = 0.6;

    /** Ventana para aprender la velocidad libre al arrancar el intake. */
    public static final double rollersBoostLearnSeconds = 0.4;

    /**
     * Cuánto se mantiene el boost después de que la carga desaparece. Evita
     * que parpadee entre pelota y pelota.
     */
    public static final double rollersBoostHoldSeconds = 0.35;

    /** Rampa de subida y bajada del duty cycle, en duty/s. 5.0 = 0.8→1.0 en 40 ms. */
    public static final double rollersBoostRampUpPerSec = 5.0;
    public static final double rollersBoostRampDownPerSec = 2.0;

    /**
     * Tope de boost continuo. Si la carga no cede en este tiempo, algo está
     * atorado y seguir empujando al 100% sólo calienta el motor: se baja a la
     * base durante {@code rollersBoostCooldownSeconds} y se vuelve a intentar.
     */
    public static final double rollersBoostMaxSeconds = 2.5;
    public static final double rollersBoostCooldownSeconds = 0.75;

    // ════════════════════════════════════════════════════════════════════════
    //  A G I T A C I Ó N   D E   L A   C A J A
    //
    //  Sacudida del extensor con envolvente decreciente para romper puentes de
    //  pelotas. La usan competencia y demo por igual (antes vivía en
    //  DemoConstants; ahí quedan alias con el mismo nombre).
    //
    //      envolvente(t) = e^(−t / τ)
    //      setpoint(t)   = centro(t) + amplitud(t) · sin(2π f t)
    // ════════════════════════════════════════════════════════════════════════

    /** Posición central de la oscilación al inicio (radianes de extensor). */
    public static final double agitateStartCenterRad = 11.0;

    /** Posición a la que converge la caja al final. */
    public static final double agitateEndCenterRad = 4.0;

    /** Amplitud inicial de la oscilación. */
    public static final double agitateAmplitudeRad = 7.5;

    /** Frecuencia de la agitación. */
    public static final double agitateFrequencyHz = 1.2;

    /** Constante de tiempo del decaimiento (la amplitud cae a ~37% en este tiempo). */
    public static final double agitateDecaySeconds = 10;

    /** Velocidad del rodillo durante la agitación. Lento, hacia adentro. */
    public static final double agitateRollerSpeed = 0.375;

    /**
     * Pre-roll: cuánto gira el rodillo ANTES de empezar a sacudir, para que las
     * pelotas a medio camino en la rampa terminen de entrar en vez de salir
     * disparadas con la primera sacudida.
     */
    public static final double agitatePrerollSeconds = 1.5;

    /** Límites duros. Deben quedar dentro de los soft limits del extensor. */
    public static final double agitateMinRad = 3.0;
    public static final double agitateMaxRad = 17.0;

    /** Si es true, la agitación arranca sola al alimentar (LB/RB). */
    public static final boolean autoAgitateWhileFeeding = true;
}