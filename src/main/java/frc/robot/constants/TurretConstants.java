package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.geometry.Translation2d;

public class TurretConstants {
    // ── Flywheel ───────────────────────────────────────────────────────────────
    public static final int flywheelCanId      = 28;

    // ── Hood ───────────────────────────────────────────────────────────────────
    public static final int hoodCanId          = 27;

    // ── Rotation Motor ─────────────────────────────────────────────────────────
    public static final int rotationMotorCanId = 26;
    public static final double rotationMotorGearRatio = (50.0 / 12.0) * (82.0 / 10.0);

    public static final boolean rotationMotorInverted = false;
    public static final Slot0Configs rotationMotorGains =
        new Slot0Configs()
            .withKP(50.0)
            .withKI(0.1)
            .withKD(2.0)
            .withKS(0.35)
            .withKV(4.2);

    // Motion magic speed limiter
    public static final double maxVelocityRotPerSec = 2.0;   // [0.3 – 2.4] rot/s — cruise speed
    public static final double maxAccelerationRotPerSecSec = 5.0;   // [0.5 – 5.0] rot/s² — ramp rate

    // Soft limits
    public static final double maxRotationDeg =  243.6;
    public static final double minRotationDeg = -116.4;
    public static final double maxRotationRad = Math.toRadians(maxRotationDeg);
    public static final double minRotationRad = Math.toRadians(minRotationDeg);

    // Turret zero-angle offset
    public static final double turretZeroOffsetRad = Math.PI;

    // ── Geometría de apuntado ──────────────────────────────────────────────────

    /**
     * Centro del robot → eje de giro de la torreta, en el plano (metros).
     *
     * <p>
     * Convención WPILib: +X hacia el frente del robot, +Y hacia la izquierda.
     * La torreta NO está en el centro del robot (va atrás a la derecha), así que
     * apuntar desde el centro mete un error de paralaje que cambia con la
     * distancia y con el rumbo: con ~24 cm de excentricidad son ~4.5° a 3 m y
     * ~2.7° a 5 m. Ningún offset constante lo corrige.
     *
     * <p>
     * Se toma de {@code DemoConstants.robotToTurretPivot} para que haya una sola
     * fuente de verdad (la usa también el visualizador). <b>Medir en el robot
     * real</b> con cinta desde el centro del chasis (centro del cuadro de los
     * módulos) al eje de la torreta, y corregir allá.
     */
    public static final Translation2d robotToTurretPivot2d =
        DemoConstants.robotToTurretPivot.toTranslation2d();

    /**
     * Trim angular fino de la torreta, en grados (CCW positivo, visto desde arriba).
     *
     * <p>
     * Éste SÍ es un offset constante, y sólo debe corregir errores constantes:
     * el cero mecánico/encoder de la torreta o un cañón que tira ligeramente
     * chueco. Calibrar DESPUÉS de medir {@link #robotToTurretPivot2d}: si el tiro
     * se va a la misma dirección (vista desde la torreta) a 2 m y a 5 m, es trim;
     * si el error cambia con la distancia o el lado del campo, es geometría.
     * Si la pelota cae a la DERECHA del HUB, súbelo; a la IZQUIERDA, bájalo.
     */
    public static final double turretAimTrimDeg = 0.0;
}