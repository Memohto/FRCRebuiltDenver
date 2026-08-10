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
}