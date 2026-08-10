package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Todo lo ajustable del Demo Mode vive aquí.
 *
 * <p>
 * Regla de oro de este archivo: si un mentor te pide cambiar algo en medio de
 * una demo en la prepa, debe poder hacerlo tocando una sola línea de aquí y
 * volviendo a desplegar. Ninguna de estas constantes está duplicada en otro
 * lado.
 */
public class DemoConstants {

    // ════════════════════════════════════════════════════════════════════════
    // SMOOTH DRIVE SYSTEM
    // ════════════════════════════════════════════════════════════════════════

    /**
     * Velocidad libre REAL del robot, medida.
     *
     * <p>
     * Deliberadamente NO usamos {@code TunerConstants.kSpeedAt12Volts} aquí,
     * porque está en 10 m/s y el robot físicamente hace ~6.07 m/s (19.9 ft/s
     * según el Tech Binder). Usar el valor inflado haría que el 70% del demo
     * fuera en realidad el 115% de la capacidad real — o sea, sin límite.
     *
     * <p>
     * Después de correr "Drive Simple FF Characterization", pon aquí el valor
     * medido.
     */
    public static final double measuredFreeSpeedMetersPerSec = 6.07;

    /** Fracción de velocidad para el demo. 0.70 = 70%. */
    public static final double maxSpeedFraction = 0.50;

    /** Fracción con el gatillo de precisión presionado (multitudes, pasillos). */
    public static final double creepSpeedFraction = 0.25;

    /** Velocidad angular máxima del chasis en modo demo (rad/s). */
    public static final double maxAngularSpeedRadPerSec = 6.0;

    // ── Límites del filtro (unidades normalizadas 0..1 por segundo) ─────────
    //
    // Referencia rápida:
    //   accel 2.6  → de parado a full en ~0.38 s
    //   decel 3.2  → de full a parado en ~0.31 s
    //   tau   0.10 → la aceleración tarda ~0.3 s en establecerse; es lo que
    //                redondea las esquinas sin que se sienta retardo
    //
    // Estos valores son más agresivos que los de la primera versión a
    // propósito. Aquel filtro era un controlador bang-bang que aplicaba
    // aceleración máxima ante cualquier error, así que números "suaves" daban
    // un robot brusco. Éste es un limitador: sólo recorta lo que excede el
    // límite, así que números más altos se sienten más directos, no más
    // violentos.

    public static final double translationMaxAccel = 2.6;
    public static final double translationMaxDecel = 3.2;

    /**
     * Constante de tiempo del suavizado de la aceleración, en segundos.
     *
     * <p>
     * Es <b>la</b> perilla del tacto del robot:
     *
     * <ul>
     * <li>{@code 0.00} — trapecio puro. Máxima respuesta, se siente el tirón.</li>
     * <li>{@code 0.10} — default. Suave y sin retardo perceptible.</li>
     * <li>{@code 0.20} — muy suave, el driver empieza a notar lag.</li>
     * <li>{@code 0.30+} — se siente como manejar en gelatina. No.</li>
     * </ul>
     */
    public static final double translationSmoothingTau = 0.15;

    public static final double rotationMaxAccel = 4.0;
    public static final double rotationMaxDecel = 5.0;
    public static final double rotationSmoothingTau = 0.12;

    // ── Respuesta del joystick ──────────────────────────────────────────────
    public static final double joystickDeadband = 0.08;

    /**
     * Exponente de la curva de respuesta. 1.0 = lineal, 2.0 = cuadrática.
     *
     * <p>
     * Bajado de 1.7 a 1.4. Con el filtro anterior el exponente alto disimulaba
     * el problema de que cualquier entrada disparaba aceleración máxima; ahora
     * que eso está arreglado, una curva más plana se siente más directa y el
     * driver percibe mejor la relación entre lo que hace con el pulgar y lo que
     * hace el robot.
     *
     * <p>
     * Si un novato sigue sintiéndolo nervioso cerca del centro, súbelo a 1.7.
     * Si un driver con experiencia lo siente perezoso, bájalo a 1.0.
     */
    public static final double joystickExponent = 1.5;

    // ════════════════════════════════════════════════════════════════════════
    // CÁMARA EN LA TORRETA
    // ════════════════════════════════════════════════════════════════════════

    /**
     * Nombre NetworkTables de la Limelight montada en la torreta.
     *
     * <p>
     * <b>Es el hostname configurado en la web UI de la Limelight, no una
     * etiqueta nuestra.</b>
     *
     * <p>
     * De vuelta a la Limelight 2/2+ de siempre, que ya estaba dada de alta como
     * {@code "limelight-fixed"} — el mismo nombre que usa el código de
     * competencia. Físicamente se mueve del shooter fijo a la torreta, pero el
     * hostname no cambia.
     */
    public static final String turretCameraName = "limelight-fixed";

    /**
     * Traslación del centro del robot al eje de giro de la torreta, en metros.
     *
     * <p>
     * Convención WPILib: +X hacia adelante, +Y hacia la izquierda, +Z hacia
     * arriba. Los valores actuales vienen de {@code TurretVisualizer}
     * (torreta trasera derecha) y son un punto de partida, NO una medición.
     *
     * <p>
     * <b>TODO — medir:</b> ver la guía de calibración en
     * {@code DEMO_MODE_GUIA.md}, sección "Calibración de la Limelight".
     */
    public static final Translation3d robotToTurretPivot = new Translation3d(-0.165, 0.170, 0.210);

    /**
     * Transformada del eje de giro de la torreta al lente de la Limelight.
     *
     * <p>
     * Se aplica DESPUÉS de la rotación de la torreta, así que está expresada en
     * el marco de la torreta: +X es hacia donde apunta el cañón.
     *
     * <p>
     * <b>TODO — medir.</b> Valores actuales: cámara 12 cm por delante del eje,
     * centrada, 8 cm arriba, inclinada 15° hacia arriba.
     */
    public static final Transform3d turretPivotToCamera = new Transform3d(
            new Translation3d(0.120, 0.0, 0.080),
            new Rotation3d(0.0, Math.toRadians(-15.0), 0.0));

    /**
     * Invierte el signo del eje Y al publicar la pose de cámara a la Limelight.
     *
     * <p>
     * Algunas versiones de firmware usan "side positive = derecha" en vez de la
     * convención WPILib "+Y = izquierda". Si al mover la torreta a la izquierda
     * la pose estimada se va a la derecha, pon esto en {@code true}.
     */
    public static final boolean limelightInvertSideAxis = false;

    /**
     * Signo del ángulo de torreta al construir la transformada robot→cámara.
     *
     * <p>
     * El sentido de giro positivo del mecanismo depende de cuántas etapas tiene
     * la caja y de si hay un engrane loco. No se puede deducir del código.
     *
     * <p>
     * <b>Esta constante es la única que hay que ajustar</b>, y tiene una prueba
     * decisiva: deja un tag fijo, gira la torreta 45° a mano y observa
     * {@code Vision/Camera0/targetPoseRobotSpace}. El tag no se movió, así que su
     * pose relativa al ROBOT no debe cambiar. Si cambia (y sobre todo si gira al
     * doble), invierte esto.
     *
     * <p>
     * De ella dependen el follow-me, la corrección al centro del HUB y la
     * localización por MegaTag. Un solo número, una sola prueba.
     */
    public static final double turretCameraYawSign = 1.0;

    // ── Modelo de cámara ───────────────────────────────────────────────────

    /**
     * ¿Es una Limelight 4?
     *
     * <p>
     * {@code false} = Limelight 2 / 2+ / 3. Es lo que tenemos montado.
     *
     * <p>
     * Con esto en false, el código no escribe las claves de NetworkTables que
     * sólo existen en la LL4 ({@code imumode_set}, {@code throttle_set}).
     * Escribirlas en una LL2 no rompe nada — simplemente las ignora — pero deja
     * basura en la tabla que confunde cuando alguien anda depurando.
     *
     * <p>
     * Todo lo demás que usa el demo ({@code camerapose_robotspace_set},
     * {@code priorityid}, {@code tl}/{@code cl}) funciona igual en la LL2
     * siempre que tenga LimelightOS 2024 o más reciente.
     */
    public static final boolean isLimelight4 = false;

    // ── Limelight 4 (inactivo mientras isLimelight4 sea false) ─────────────

    /**
     * Modo de IMU de la Limelight 4. <b>DEBE quedarse en 0 con la cámara en la
     * torreta.</b>
     *
     * <p>
     * La LL4 trae una IMU interna que normalmente mejora MegaTag2. El problema
     * es que esa IMU está atornillada a la cámara, y la cámara está atornillada
     * a la torreta. Cuando la torreta gira 90°, la IMU cree que el ROBOT giró
     * 90°. Cualquier modo que use la IMU interna (1 a 4) le daría a MegaTag2 un
     * yaw que no tiene nada que ver con la orientación real del chasis, y la
     * localización se destruye.
     *
     * <p>
     * Modo 0 = usar exclusivamente la orientación externa que le publicamos
     * desde el Pigeon 2.0, que sí está fijo al chasis.
     *
     * <p>
     * Si algún día regresan la cámara a un montaje fijo, el modo 4 con
     * {@code imuAssistAlpha} vale mucho la pena.
     */
    public static final int limelight4ImuMode = 0;

    /**
     * Throttle de la LL4 mientras el robot está deshabilitado.
     *
     * <p>
     * La LL4 consume hasta 12 W y se calienta. La documentación recomienda
     * 100-200 en reposo y 0 habilitado. El código lo cambia solo. Importa más
     * de lo que parece en una demo: el robot pasa mucho más tiempo deshabilitado
     * en un pasillo que en una cancha.
     */
    public static final int limelight4ThrottleDisabled = 150;

    /** Throttle con el robot habilitado. 0 = máximo rendimiento. */
    public static final int limelight4ThrottleEnabled = 0;

    /**
     * Usar {@code priorityid} para el modo de tag fijado.
     *
     * <p>
     * Le dice a la Limelight cuál tag debe usar para calcular tx/ty. Sin esto,
     * la cámara siempre reporta el tag más grande de la imagen, así que fijar un
     * tag por software sólo funciona mientras ese tag sea el más cercano. Con
     * priorityid, el enganche se mantiene aunque otro alumno se acerque más con
     * otro target.
     */
    public static final boolean useLimelightPriorityId = true;

    // ════════════════════════════════════════════════════════════════════════
    // TORRETA — BÚSQUEDA Y SEGUIMIENTO DE APRILTAGS
    // ════════════════════════════════════════════════════════════════════════

    /**
     * Signo de la corrección de tx. La Limelight reporta tx positivo cuando el
     * target está a la DERECHA; la torreta positiva gira a la IZQUIERDA (CCW).
     * Si la torreta se aleja del tag en vez de acercarse, invierte esto.
     */
    public static final double turretTxSign = 1.0;

    /**
     * Compensación de latencia de visión.
     *
     * <p>
     * <b>Ésta es la causa principal del wobble que vieron.</b> Cuando la
     * Limelight reporta {@code tx}, ese número describe dónde estaba el target
     * hace 20-40 ms. Si aplicas esa corrección sobre el ángulo ACTUAL de la
     * torreta y la torreta se está moviendo, estás corrigiendo dos veces el
     * mismo error: una parte ya la recorrió el mecanismo mientras la imagen
     * viajaba. El resultado es sobrepaso, regreso, sobrepaso — oscilación.
     *
     * <p>
     * Con esto activado, el código calcula dónde estaba la torreta cuando se
     * capturó la imagen ({@code ánguloActual − velocidad × latencia}) y aplica
     * tx sobre ESE ángulo. Déjalo en true; el false es sólo para comparar.
     */
    public static final boolean turretLatencyCompensation = true;

    /**
     * Latencia extra además de la que reporta la Limelight (tl + cl).
     *
     * <p>
     * Cubre lo que la cámara no puede conocer: el viaje por NetworkTables, el
     * ciclo del scheduler, y sobre todo el <b>intervalo de muestreo</b>.
     *
     * <p>
     * Este último importa mucho con una Limelight 2/2+: a 960x720 corre a ~22
     * FPS, o sea un frame cada 45 ms. En promedio, cuando el código lee un
     * frame ese frame ya tiene media periodo de antigüedad además de la
     * latencia de pipeline — unos 22 ms extra que {@code tl} no reporta.
     *
     * <p>
     * De ahí los 45 ms por defecto. Si bajan la resolución a 640x480 (90 FPS),
     * bajen esto a ~0.020. Es la perilla principal para el wobble: si al girar
     * rápido la torreta se sigue de largo, súbanlo de 5 en 5 ms; si llega
     * arrastrándose, bájenlo.
     */
    public static final double turretExtraLatencySeconds = 0.045;

    /**
     * Filtro exponencial sobre la estimación ABSOLUTA del ángulo del target.
     *
     * <p>
     * Nota importante: se filtra el ángulo absoluto estimado, no {@code tx}.
     * Filtrar tx (una cantidad relativa que cambia con el movimiento de la
     * propia torreta) hace que el filtro pelee contra el mecanismo. Filtrar la
     * estimación absoluta es estable porque el target real no se mueve rápido.
     *
     * <p>
     * Menor = más suave y más lento. 0.25 es un buen punto de partida.
     */
    public static final double turretTrackFilterAlpha = 0.12;

    /**
     * Cambio mínimo del setpoint para re-comandar el Motion Magic.
     *
     * <p>
     * Cada vez que mandas un setpoint nuevo, el Talon reinicia su perfil de
     * movimiento. Re-comandar 50 veces por segundo con cambios de milésimas de
     * grado hace que el perfil nunca se complete y el mecanismo tiemble. Este
     * umbral hace que sólo se re-comande cuando de verdad hay algo que corregir.
     */
    public static final double turretSetpointDeadbandRad = Math.toRadians(0.8);

    /**
     * Velocidad máxima a la que puede moverse el SETPOINT (no el mecanismo).
     *
     * <p>
     * Limita el salto del setpoint entre ciclos. Evita que una lectura basura
     * de un solo frame mande la torreta volando al otro lado. El Motion Magic
     * del Talon sigue teniendo sus propios límites de crucero y aceleración.
     */
    public static final double turretMaxSetpointRateRadPerSec = 2.5;

    /** Error bajo el cual se considera que la torreta está apuntada. */
    public static final double turretOnTargetToleranceRad = Math.toRadians(1.5);

    // ── Histéresis de enganche ─────────────────────────────────────────────
    //
    // Ésta es LA solución al temblor de la torreta cuando ya está viendo su
    // objetivo.
    //
    // El problema: por buena que sea la ganancia, tx nunca es exactamente cero.
    // Oscila un par de décimas de grado por ruido de píxel. Con cualquier lazo
    // continuo, esas décimas se convierten en comandos, cada comando reinicia
    // el perfil de Motion Magic, y el mecanismo tiembla eternamente persiguiendo
    // ruido que no significa nada.
    //
    // La solución no es más tuning: es dejar de comandar. Cuando el error cae
    // bajo turretLockThresholdRad, la torreta se DECLARA enganchada y congela
    // su setpoint. No vuelve a moverse hasta que el error supere
    // turretUnlockThresholdRad — que es notablemente mayor, y por eso el ruido
    // nunca alcanza a desenganchar.
    //
    // Efecto visible: la torreta se mueve, se planta, y se queda absolutamente
    // quieta hasta que el objetivo se mueve de verdad.

    /** Bajo este error, la torreta se engancha y congela el setpoint. */
    public static final double turretLockThresholdRad = Math.toRadians(1.0);

    /**
     * Sobre este error, la torreta se desengancha y vuelve a seguir.
     *
     * <p>
     * Tiene que ser cómodamente mayor que el umbral de enganche. La diferencia
     * entre ambos ES la histéresis: si son muy parecidos, el ruido hace que
     * enganche y desenganche todo el tiempo y vuelve el temblor.
     */
    public static final double turretUnlockThresholdRad = Math.toRadians(2.5);

    // ── Ganancias específicas de demo para la rotación ─────────────────────

    /**
     * Usar ganancias de rotación propias del demo en vez de las de competencia.
     *
     * <p>
     * Las de competencia traen {@code kI = 0.1}. Un término integral en un
     * mecanismo posicional con Motion Magic acumula error mientras el perfil
     * está en camino y luego sobrepasa — es una fuente clásica de cacería de
     * baja frecuencia. Para seguir un objetivo suavemente no hace falta
     * integral: el feedforward de Motion Magic ya se encarga del error estático.
     *
     * <p>
     * Ponlo en false para volver exactamente a las de competencia.
     */
    public static final boolean useDemoTurretGains = true;

    public static final double turretDemoKp = 38.0;
    public static final double turretDemoKi = 0.0;
    public static final double turretDemoKd = 3.5;
    public static final double turretDemoKs = 0.35;
    public static final double turretDemoKv = 4.2;

    /**
     * Motion Magic más lento que en competencia.
     *
     * <p>
     * Competencia usa crucero 2.0 rot/s y aceleración 5.0 rot/s², que están
     * pensados para llegar rápido a un setpoint. Para <i>seguir</i> un objetivo
     * suavemente conviene lo contrario: perfiles lentos que no alcancen a
     * saturar entre comando y comando.
     */
    public static final double turretDemoCruiseRotPerSec = 1.2;
    public static final double turretDemoAccelRotPerSecSec = 2.5;

    /** Ciclos consecutivos dentro de tolerancia para declarar "apuntada". */
    public static final int turretOnTargetCycles = 4;

    /** Tras perder el target, cuánto tiempo mantiene el último ángulo. */
    public static final double turretTargetGraceSeconds = 0.6;

    /**
     * Usar {@code txnc} en vez de {@code tx}.
     *
     * <p>
     * {@code tx} se mide desde la crosshair (que tú calibras, típicamente para
     * alinear el tiro). {@code txnc} se mide desde el centro óptico real del
     * lente y es independiente de esa calibración.
     *
     * <p>
     * Déjalo en false si vas a calibrar la crosshair para compensar el offset
     * mecánico entre el lente y el cañón. Ponlo en true si prefieres que el
     * apuntado sea puramente geométrico y compensar el offset en código.
     */
    public static final boolean useTxNoCrosshair = false;

    // ════════════════════════════════════════════════════════════════════════
    // GEOMETRÍA DEL HUB — apuntado vectorial
    // ════════════════════════════════════════════════════════════════════════
    //
    // Cada cara del HUB tiene DOS AprilTags: una centrada y otra a su
    // izquierda. Apuntar al centro de un tag no es apuntar al centro del HUB —
    // y la diferencia crece mientras más de lado veas la cara.
    //
    // Peor todavía, con dos tags visibles la cámara puede saltar entre uno y
    // otro según cuál se vea más grande, y la torreta pega un brinco cada vez.
    // Eso es buena parte del comportamiento "glitchy" que reportaron.

    /** Tags centradas en cada cara del HUB. Rojas 2/4/8/10, azules 18/20/24/26. */
    public static final int[] hubCenterTagIds = { 2, 4, 8, 10, 18, 20, 24, 26 };

    /** Tags a la izquierda de cada cara. Rojas 3/5/9/11, azules 19/21/25/27. */
    public static final int[] hubLeftTagIds = { 3, 5, 9, 11, 19, 21, 25, 27 };

    /**
     * Distancia del plano de la cara del HUB a su eje central, en metros.
     *
     * <p>
     * Es el apotema de la sección del HUB: cuánto hay que "meterse" desde la
     * cara donde vive el tag hasta el centro real de la estructura.
     *
     * <p>
     * <b>TODO — medir.</b> Con el HUB enfrente, mide de la superficie de una
     * cara al centro. El valor actual es una estimación.
     *
     * <p>
     * Cómo verificar: párate en diagonal a una cara, a unos 3 m. La torreta debe
     * apuntar visiblemente <i>hacia adentro</i> respecto al tag, no al tag. Si
     * apunta al tag, este número está en cero o el signo se está auto-corrigiendo
     * mal.
     */
    public static final double hubFaceToCenterMeters = 0.60;

    /**
     * Distancia lateral de la tag izquierda al centro de su cara, en metros.
     *
     * <p>
     * Sólo se usa cuando la tag central no está visible. <b>TODO — medir.</b>
     */
    public static final double hubLeftTagLateralOffsetMeters = 0.35;

    /**
     * Corrección angular máxima que se le permite al apuntado vectorial.
     *
     * <p>
     * Red de seguridad: si la pose 3D del tag viene basura, la corrección
     * calculada se dispara. Esto la recorta a algo físicamente razonable.
     */
    public static final double hubAimMaxCorrectionRad = Math.toRadians(25.0);

    /**
     * Filtro sobre la corrección angular al centro del HUB.
     *
     * <p>
     * Muy agresivo a propósito. La corrección depende de la orientación 3D del
     * tag, que es el dato más ruidoso que da la cámara. Como además cambia
     * lentamente (sólo si el robot se mueve alrededor del HUB), se puede filtrar
     * fuerte sin perder nada.
     */
    public static final double hubAimFilterAlpha = 0.08;

    // ── Barrido de búsqueda ────────────────────────────────────────────────
    //
    // IMPORTANTE: el barrido ya NO corre solo al habilitar. Únicamente ocurre
    // mientras el operador mantiene APUNTAR o CARGAR y no hay ningún tag a la
    // vista. Sin botón presionado, la torreta se queda quieta en cero.

    /** Si es false, al no ver nada la torreta simplemente se queda quieta. */
    public static final boolean searchSweepEnabled = true;

    /** Amplitud del barrido (a cada lado del cero). */
    public static final double turretSweepAmplitudeRad = Math.toRadians(100.0);

    /** Frecuencia del barrido. 0.25 Hz ≈ un ciclo completo cada 4 s. */
    public static final double turretSweepFrequencyHz = 0.25;

    /**
     * Cada cuánto cambia de pipeline mientras busca sin encontrar nada.
     *
     * <p>
     * Ésta es la adaptación a iluminación variable: en vez de intentar
     * auto-exponer por software, configuras varios pipelines en la web UI de la
     * Limelight y el código los va rotando hasta que uno vea el tag.
     *
     * <p>
     * Con 2.5 s da tiempo a que el cambio de pipeline surta efecto (la Limelight
     * tarda ~200 ms en recargar) y a que el detector alcance un par de frames
     * buenos antes de descartarlo.
     */
    public static final double pipelineCycleSeconds = 2.5;

    /**
     * Pipelines a rotar durante la búsqueda.
     *
     * <p>
     * Configuración recomendada para salones de escuela (ver DEMO_MODE_GUIA.md):
     *
     * <pre>
     *   0 → Largo alcance   960x720, downscale 1x, exposición baja
     *   1 → Cerca y rápido  640x480, downscale 1x, exposición baja
     *   2 → Anti-parpadeo   960x720, exposición ~8.3 ms (múltiplo del
     *                       parpadeo de 120 Hz de las lámparas de 60 Hz)
     * </pre>
     */
    public static final int[] searchPipelines = { 0, 1, 2 };

    // ════════════════════════════════════════════════════════════════════════
    // BOMBER — ALINEACIÓN DEL CHASIS
    // ════════════════════════════════════════════════════════════════════════

    /**
     * Ganancias del PID de rumbo del chasis.
     *
     * <p>
     * El código de competencia usa kP = 0.5 y luego multiplica la salida por
     * {@code getMaxAngularSpeedRadPerSec()} (~20 rad/s), o sea una ganancia
     * efectiva de ~10. Por eso el orbit se sentía violento: saturaba de
     * inmediato y se comportaba como bang-bang.
     *
     * <p>
     * Aquí la salida del PID ya está en rad/s y NO se re-escala. kP = 4.0
     * significa: 0.25 rad (14°) de error → 1.0 rad/s de corrección. Suave y
     * predecible.
     */
    public static final double headingKp = 2.2;
    public static final double headingKd = 0.45;

    /**
     * Filtro sobre el término derivativo del control de rumbo.
     *
     * <p>
     * Un derivativo crudo amplifica el ruido de la medición, así que hay que
     * dejar kD tan bajo que no amortigua nada — y sin amortiguamiento el lazo
     * oscila. Filtrarlo primero es lo que permite subir kD lo suficiente para
     * que de verdad frene el sobrepaso.
     *
     * <p>
     * Menor = más suave y más lento en reaccionar.
     */
    public static final double headingDerivativeAlpha = 0.15;

    /**
     * Zona muerta angular del auto-alineado.
     *
     * <p>
     * Dentro de este error, la salida es cero.
     */
    public static final double headingToleranceRad = Math.toRadians(1.5);

    /**
     * Ancho de la rampa de entrada a la zona muerta.
     *
     * <p>
     * Antes se usaba {@code atSetpoint()} para cortar la salida de golpe al
     * llegar a la tolerancia. Ese corte es una discontinuidad: el chasis pasa de
     * corregir a no corregir en un ciclo, se pasa un poco, vuelve a corregir de
     * golpe. Es exactamente el "glitchy y medio agresivo" que reportaron.
     *
     * <p>
     * Ahora la salida se escala suavemente de 0 a 1 entre
     * {@code headingToleranceRad} y {@code headingToleranceRad + este valor}.
     * Sin escalón, sin buzz.
     */
    public static final double headingSoftZoneRad = Math.toRadians(4.0);

    /**
     * Exigir odometría confirmada por visión para asistir el rumbo.
     *
     * <p>
     * <b>Déjalo en false.</b> Ponerlo en true creaba un bloqueo circular: la
     * asistencia no arrancaba sin odometría confirmada, la odometría sólo se
     * confirma cuando la cámara ve un tag del HUB, y con la torreta congelada en
     * cero la cámara sólo apunta al HUB <i>si el chasis ya se alineó</i>. El
     * modo nunca podía arrancar.
     *
     * <p>
     * El código de competencia tampoco verificaba nada: calculaba el rumbo desde
     * la pose y lo aplicaba. Si la pose está mal, el piloto lo ve y suelta el
     * gatillo.
     */
    public static final boolean requireFreshOdometryForAssist = false;

    /**
     * En BOMBER, cerrar el lazo sobre la cámara cuando haya un tag del HUB.
     *
     * <p>
     * Con la torreta congelada en cero, la cámara mira exactamente por donde
     * salen los cañones — así que {@code tx} <b>es</b> el error de rumbo del
     * chasis. No hace falta odometría ni pose 3D: sólo llevar tx a cero.
     *
     * <p>
     * Es también lo que rompe el bloqueo del arranque: en cuanto el robot ve un
     * tag del HUB se alinea sin depender de saber dónde está, y esa alineación
     * es la que después le permite a la visión corregir la pose.
     */
    public static final boolean bomberUsesVisionWhenTagVisible = true;

    // ── Traspaso visión → odometría ────────────────────────────────────────
    //
    // El problema: cerca del HUB la visión alinea perfecto, pero al perder el
    // tag el error se recalcula desde la POSICIÓN estimada del robot — y esa
    // posición nunca es exacta. El salto entre las dos respuestas hace que el
    // chasis pegue un tirón y se quede persiguiendo un rumbo equivocado.
    //
    // La solución es aprender el sesgo. Mientras la visión funciona, se mide
    // cuánto se equivoca la odometría:
    //
    //     sesgo = errorVisión − errorOdometría
    //
    // y al perder el tag se usa "errorOdometría + sesgo". El traspaso queda
    // continuo POR CONSTRUCCIÓN: en el instante del cambio las dos respuestas
    // son idénticas. Y de paso corrige el error sistemático de la pose.

    /**
     * Filtro del sesgo de rumbo. Lento a propósito: es una cantidad que casi no
     * cambia, y filtrarla fuerte evita que el ruido de la visión la contamine.
     */
    public static final double hubAlignBiasAlpha = 0.05;

    /**
     * Velocidad máxima a la que puede moverse el RUMBO OBJETIVO, en rad/s.
     *
     * <p>
     * No limita al chasis: limita cuánto puede moverse el punto al que el chasis
     * quiere apuntar. Una pose basura de un solo ciclo ya no puede mandar al
     * robot a dar la vuelta — el objetivo se mueve despacio y el error se
     * corrige antes de que llegue a ningún lado.
     */
    public static final double hubAlignMaxTargetRateRadPerSec = 2.0;

    /**
     * Distancia mínima al objetivo para confiar en el rumbo de odometría.
     *
     * <p>
     * Muy cerca del objetivo el rumbo se vuelve numéricamente inestable:
     * centímetros de error de posición se convierten en decenas de grados de
     * error de rumbo, y el robot gira sin parar persiguiendo una dirección que
     * brinca. Por debajo de esto se mantiene el último rumbo bueno.
     */
    public static final double hubAlignMinPoseDistanceMeters = 0.8;

    /** Velocidad angular máxima que el auto-alineado puede pedir. */
    public static final double headingMaxOmegaRadPerSec = 2.5;

    // ════════════════════════════════════════════════════════════════════════
    // FOLLOW-ME ("perrito") — ENVOLVENTE CONSERVADORA
    // ════════════════════════════════════════════════════════════════════════

    /** Distancia a la que el robot se estaciona frente al target. */
    public static final double followDistanceMeters = 0.6;

    /**
     * Distancia dura de seguridad. Por debajo de esto el robot SÓLO puede
     * alejarse, nunca acercarse, sin importar lo que diga el controlador.
     */
    public static final double followMinSafeDistanceMeters = 0.3;

    /** Velocidad máxima del seguimiento. Deliberadamente lenta. */
    public static final double followMaxSpeedMetersPerSec = 1.6;

    /** Ganancia P sobre el error de posición, en (m/s) por metro. */
    public static final double followTranslationKp = 1.1;

    /** Bajo este error de posición el robot se queda quieto. */
    public static final double followPositionToleranceMeters = 0.12;

    /**
     * Zona muerta del cuerpo. Mientras el target esté dentro de este ángulo,
     * SÓLO se mueve la torreta y el chasis se queda quieto.
     *
     * <p>
     * Esta constante es la que produce el efecto "la cabeza va primero": los
     * movimientos pequeños del alumno los absorbe la torreta sola, y el chasis
     * sólo se anima a girar cuando el alumno se mueve de verdad. Súbelo para
     * que el cuerpo sea más perezoso, bájalo para que siga más de cerca.
     */
    public static final double followBodyDeadbandRad = Math.toRadians(9.0);

    /** Ganancia del chasis siguiendo a la torreta, en rad/s por radián. */
    public static final double followBodyKp = 1.6;

    /**
     * Si la torreta pasa de este ángulo, el chasis deja de girar.
     *
     * <p>
     * Red de seguridad contra el modo de falla que vieron: si por cualquier
     * razón el cuerpo empuja la torreta hacia su límite en vez de aliviarla,
     * esto corta antes de que llegue al tope y tenga que dar la vuelta completa.
     */
    public static final double followBodyMaxTurretAngleRad = Math.toRadians(120.0);

    /**
     * Límites de aceleración angular del cuerpo durante el follow.
     * Mucho más lentos que los del manejo normal — ese retardo es justamente
     * lo que hace que se vea como una criatura girando y no como un robot.
     */
    public static final double followBodyMaxAccel = 1.2;
    public static final double followBodyMaxDecel = 1.8;
    public static final double followBodySmoothingTau = 0.25;

    /** Filtro exponencial sobre la pose del tag. Absorbe el ruido de la LL. */
    public static final double followTargetFilterAlpha = 0.25;

    /**
     * Qué tanto orbita el robot alrededor del target cuando éste gira.
     *
     * <p>
     * {@code 0.0} = sólo mantiene distancia. El robot se queda en la línea recta
     * entre él y el tag, sin importar hacia dónde mire el tag. Funciona siempre.
     * <br>
     * {@code 1.0} = orbita completo. El robot se mueve para quedar frente a la
     * CARA del tag: si el alumno gira, el robot le da la vuelta. Es lo
     * espectacular, pero depende de que el yaw del tag venga con el signo que
     * esperamos.
     *
     * <p>
     * <b>Si el robot se va hacia el lado equivocado cuando giras el target</b>,
     * prueba primero con {@link #followTagYawOffsetRad}; si sigue raro, baja
     * esto a 0.0 y el seguimiento funcionará igual, sólo sin el orbitado.
     */
    public static final double followOrbitStrength = 0.0;

    /**
     * Offset aplicado al yaw reportado del tag.
     *
     * <p>
     * Existe porque la convención de orientación de
     * {@code targetpose_robotspace} varía entre versiones de firmware de
     * Limelight. Procedimiento para calibrarlo: pon el tag apuntando
     * directamente al robot y verifica que el robot se estacione ENFRENTE del
     * tag. Si se va detrás, cambia esto entre {@code Math.PI} y {@code 0.0}.
     */
    public static final double followTagYawOffsetRad = Math.PI;

    /** Sin ver el target más de esto, el follow se detiene por completo. */
    public static final double followLostTargetTimeoutSeconds = 0.5;

    /**
     * Si el piloto mueve el stick más de esto, el follow se cancela al instante
     * y el control vuelve a ser manual. Es el "botón de pánico" implícito.
     */
    public static final double followDriverOverrideThreshold = 0.15;

    // ════════════════════════════════════════════════════════════════════════
    // AGITACIÓN DEL INTAKE
    // ════════════════════════════════════════════════════════════════════════

    /** Posición central de la oscilación al inicio (radianes de extensor). */
    public static final double agitateStartCenterRad = 11.0;

    /** Posición a la que converge la caja al final. */
    public static final double agitateEndCenterRad = 4.0;

    /** Amplitud inicial de la oscilación. */
    public static final double agitateAmplitudeRad = 7.5;

    /** Frecuencia de la agitación. */
    public static final double agitateFrequencyHz = 1.2;

    /**
     * Constante de tiempo del decaimiento. La amplitud cae a ~37% en este
     * tiempo, y a ~5% en 3× este tiempo. Con 2.5 s, la caja está prácticamente
     * cerrada a los 7.5 s.
     */
    public static final double agitateDecaySeconds = 10;

    /** Velocidad del rodillo durante la agitación. Lento, hacia adentro. */
    public static final double agitateRollerSpeed = 0.375;

    /**
     * Pre-roll: cuánto gira el rodillo ANTES de empezar a sacudir.
     *
     * <p>
     * Sirve para las pelotas que quedaron a medio camino en la rampa. Si empiezas
     * a sacudir de inmediato, esas pelotas salen disparadas hacia afuera en vez
     * de terminar de entrar. Medio segundo de rodillo solo las acomoda adentro y
     * después ya se puede agitar sin perder nada.
     */
    public static final double agitatePrerollSeconds = 1.5;

    /** Límites duros. Deben quedar dentro de los soft limits del extensor. */
    public static final double agitateMinRad = 3.0;
    public static final double agitateMaxRad = 17.0;

    /** Si es true, la agitación arranca sola al alimentar (LB/RB). */
    public static final boolean autoAgitateWhileFeeding = true;

    // ════════════════════════════════════════════════════════════════════════
    // DISPARO SUAVE (STRIKER — hacia personas)
    // ════════════════════════════════════════════════════════════════════════

    /**
     * Tope duro de RPS del tiro suave.
     *
     * <p>
     * Ningún camino de código puede exceder esto mientras el robot esté
     * apuntándole a un AprilTag suelto. Es la red de seguridad: aunque alguien
     * edite mal el mapa de abajo, el robot no puede dispararle un torpedo a un
     * alumno.
     */
    public static final double gentleMaxFlywheelRPS = 40.0;

    // ════════════════════════════════════════════════════════════════════════
    //
    //   M A P A   D E   T I R O   S U A V E   —   T U N É A L O   A Q U Í
    //
    //   Mismo formato que ShooterConstants: un punto por distancia, con su
    //   potencia y su ángulo de hood. Agrega, quita o mueve puntos libremente;
    //   entre puntos se interpola lineal y fuera del rango se hace clamp.
    //
    //   ┌──────────┬───────────┬────────────┐
    //   │ Distancia│    RPS    │ Hood (deg) │
    //   ├──────────┼───────────┼────────────┤
    //   │  1.0 m   │    20     │    10.0    │
    //   │  2.0 m   │    24     │    13.0    │
    //   │  3.0 m   │    28     │    15.0    │
    //   │  4.0 m   │    31     │    16.0    │
    //   │  5.0 m   │    34     │    16.5    │
    //   └──────────┴───────────┴────────────┘
    //
    //   Estos valores son ~65% de los de competencia. La versión anterior
    //   estaba en ~40% y la pelota apenas salía del robot. Si sigue corta,
    //   sube los RPS de 2 en 2; si llega muy fuerte, bájalos.
    //
    //   El hood se mide en grados de offset desde la posición de reposo, igual
    //   que kShooterHoodMap. Más grados = más arco.
    //
    // ════════════════════════════════════════════════════════════════════════

    public static final InterpolatingDoubleTreeMap kGentleFlywheelMap = new InterpolatingDoubleTreeMap();
    static {
        kGentleFlywheelMap.put(1.0, 24.0);
        kGentleFlywheelMap.put(2.0, 28.0);
        kGentleFlywheelMap.put(3.0, 32.0);
        kGentleFlywheelMap.put(4.0, 36.0);
        kGentleFlywheelMap.put(5.0, 40.0);
    }

    public static final InterpolatingDoubleTreeMap kGentleHoodMap = new InterpolatingDoubleTreeMap();
    static {
        kGentleHoodMap.put(1.0, 10.0);
        kGentleHoodMap.put(2.0, 13.0);
        kGentleHoodMap.put(3.0, 15.0);
        kGentleHoodMap.put(4.0, 16.0);
        kGentleHoodMap.put(5.0, 16.5);
    }

    /** Distancia asumida cuando no hay medición de visión disponible. */
    public static final double fallbackDistanceMeters = 3.0;

    // ════════════════════════════════════════════════════════════════════════
    //
    //   D I S P A R O   E N   M O V I M I E N T O
    //
    // ════════════════════════════════════════════════════════════════════════
    //
    // La pelota sale del robot cargando la velocidad del robot. A 2 m/s con un
    // vuelo de 0.6 s, se desvía 1.2 m del punto al que apuntaste.
    //
    // El método es el del "objetivo virtual": en vez de compensar el ángulo con
    // una fórmula aparte, se corre el punto de mira en dirección contraria a la
    // velocidad y se apunta ahí normalmente.
    //
    //     objetivoVirtual = objetivoReal − velocidad × tiempoDeVuelo
    //
    // Resuelve el ángulo Y la distancia de una sola vez: si te mueves
    // acercándote, el objetivo virtual queda más lejos y el mapa de tiro pide
    // más potencia solo.

    /**
     * Interruptor maestro.
     *
     * <p>
     * <b>Arranca apagado a propósito.</b> La compensación depende del mapa de
     * tiempo de vuelo, que todavía no está medido; con valores estimados podría
     * empeorar los tiros en vez de mejorarlos. Enciéndelo cuando vayas a tunear,
     * con el procedimiento de la guía.
     */
    public static final boolean shootWhileMovingEnabled = false;

    /**
     * Ganancia de la compensación. 1.0 = compensación teórica completa.
     *
     * <p>
     * <b>Empieza en 0.3 y sube de 0.2 en 0.2.</b> Así ves crecer el efecto en
     * vez de descubrir de golpe que el signo estaba invertido. Si al moverte
     * lateralmente los tiros se van MÁS al lado en vez de corregirse, el signo
     * está al revés y hay que revisarlo antes de seguir subiendo.
     */
    public static final double shootWhileMovingGain = 0.3;

    /** Bajo esta velocidad no se compensa nada. Evita ruido en reposo. */
    public static final double shootWhileMovingMinSpeed = 0.2;

    /**
     * Iteraciones de convergencia.
     *
     * <p>
     * Mover el objetivo cambia la distancia, y la distancia cambia el tiempo de
     * vuelo. Dos iteraciones convergen de sobra a distancias de FRC.
     */
    public static final int shootWhileMovingIterations = 2;

    /**
     * Tope de la corrección, en metros.
     *
     * <p>
     * Red de seguridad: si la velocidad estimada viene basura, sin esto la
     * torreta se iría a apuntar a un punto absurdo.
     */
    public static final double shootWhileMovingMaxCompensationMeters = 2.5;

    /**
     * Tiempo de vuelo de la pelota por distancia, en segundos.
     *
     * <p>
     * <b>Es lo único que hay que medir para que esto funcione bien.</b> Dos
     * formas:
     *
     * <ol>
     * <li><b>Video a 60 fps.</b> Graba un tiro estático desde el costado, cuenta
     * los cuadros entre que la pelota sale del shooter y toca el HUB, divide
     * entre 60. Repite a 2, 3, 4 y 5 m.</li>
     * <li><b>Empírica.</b> Deja estos valores, sube {@code shootWhileMovingGain}
     * poco a poco y ajusta el mapa hasta que los tiros en movimiento caigan
     * donde deben. Más lento pero funciona.</li>
     * </ol>
     *
     * <p>
     * Los valores actuales son una estimación razonable para un tiro en arco a
     * las velocidades de {@code kShooterFlywheelMap}: crece con la distancia
     * porque la pelota recorre más y va perdiendo velocidad.
     */
    public static final InterpolatingDoubleTreeMap kTimeOfFlightMap = new InterpolatingDoubleTreeMap();
    static {
        kTimeOfFlightMap.put(1.0, 0.35);
        kTimeOfFlightMap.put(2.0, 0.48);
        kTimeOfFlightMap.put(3.0, 0.62);
        kTimeOfFlightMap.put(4.0, 0.76);
        kTimeOfFlightMap.put(5.0, 0.90);
    }

    // ════════════════════════════════════════════════════════════════════════
    //
    //   S M O O T H   D U M P I N G   —   smart feature de BOMBER
    //
    // ════════════════════════════════════════════════════════════════════════
    //
    // Volcado suave: el robot NO corrige dirección. Dispara con ambos cañones
    // exactamente hacia donde está apuntando, con el hood plano y potencia
    // fija. El piloto apunta con el robot; el código no calcula nada.

    /**
     * Ángulo del hood para el volcado, en grados de offset desde reposo.
     *
     * <p>
     * <b>Más grados = trayectoria más plana</b>, que es lo que queremos aquí:
     * la pelota sale hacia adelante en vez de subir en arco. Es el mismo
     * criterio de {@code kShooterHoodMap}, donde los tiros lejanos (más planos y
     * más rápidos) usan más offset.
     *
     * <p>
     * 16.5° es el valor más alto que aparece en el mapa de competencia, así que
     * es terreno conocido. El recorrido mecánico llega a ~17°; si quieres el
     * máximo absoluto súbelo con cuidado y escucha que el hood no tope.
     */
    public static final double dumpHoodDegrees = 16.5;

    /**
     * Potencia del volcado, como fracción de la velocidad libre del motor.
     *
     * <p>
     * 0.40 = 40%. Con un Kraken X60 (~100 RPS libres a 1:1) eso son <b>40 RPS</b>.
     *
     * <p>
     * Ojo con la referencia: 40 RPS queda muy cerca de la velocidad de
     * competencia (33-45 RPS). Si lo que querías era el 40% de la potencia de
     * competencia, eso serían ~18 RPS, o sea {@code 0.18} aquí. Cambia el número
     * y listo — por eso está en una sola constante.
     */
    public static final double dumpPowerFraction = 0.25;

    /**
     * Velocidad libre del flywheel, en RPS. Kraken X60 a 1:1 ≈ 100 RPS.
     *
     * <p>
     * Es sólo la referencia contra la que se aplica {@code dumpPowerFraction}.
     */
    public static final double flywheelFreeSpeedRPS = 100.0;

    /** RPS resultante del volcado. Se calcula, no se edita. */
    public static double dumpFlywheelRPS() {
        return dumpPowerFraction * flywheelFreeSpeedRPS;
    }

    /**
     * Distancia máxima a la que le creemos a la solución 3D de la cámara.
     *
     * <p>
     * Punto importante y poco intuitivo: <b>el rastreo 2D ({@code tx}) y la
     * pose 3D ({@code targetpose_robotspace}) no dejan de funcionar a la misma
     * distancia.</b> Para apuntar sólo hace falta ver el tag; para resolver su
     * pose 3D hace falta medir con precisión la posición de sus cuatro esquinas,
     * y eso se degrada mucho antes.
     *
     * <p>
     * Con una Limelight 2/2+ (sin calibración ChArUco, intrínsecos de fábrica) y
     * tags oficiales de 16.5 cm, la pose 3D empieza a dar distancias ruidosas
     * alrededor de los 4 m aunque tx siga perfectamente utilizable a 5 o 6.
     *
     * <p>
     * Si le creyéramos a esa distancia ruidosa, el mapa de tiro estaría saltando
     * entre RPS distintos frame a frame. Pasado este umbral se usa la odometría
     * o el valor de respaldo, que son estables aunque menos exactos.
     */
    public static final double maxTrustedVisionDistanceMeters = 4.0;

    /**
     * Filtro sobre la distancia medida antes de meterla al mapa de tiro.
     *
     * <p>
     * Evita que el ruido de la solución 3D haga que el flywheel esté cambiando
     * de objetivo de velocidad constantemente. Menor = más estable, más lento en
     * reaccionar cuando el robot se mueve.
     */
    public static final double distanceFilterAlpha = 0.20;

    /**
     * Posición del HUB para el modo BOMBER durante demos.
     *
     * <p>
     * En una demo no hay alianza, así que no se hace flip rojo/azul: el HUB está
     * donde tú lo pongas. Si lo colocan en el gimnasio, midan su posición
     * respecto al origen que fija el botón de reset del piloto y pónganla aquí.
     * Por defecto es la posición azul de cancha.
     */
    public static final Translation2d demoHubPosition = new Translation2d(4.625, 4.035);

    // ════════════════════════════════════════════════════════════════════════
    // VISIÓN Y ODOMETRÍA
    // ════════════════════════════════════════════════════════════════════════

    /**
     * Permite que los AprilTags del HUB corrijan la odometría.
     *
     * <p>
     * Es lo que le devuelve al demo la capacidad de competencia de "seguir
     * apuntando aunque ya no vea el tag". Sólo se aceptan correcciones cuando
     * TODOS los tags visibles son del HUB, así que un tag suelto que alguien
     * traiga en la mano nunca teletransporta la odometría — que era el riesgo
     * por el que la primera versión apagaba esto por completo.
     */
    public static final boolean useVisionOdometry = true;

    /**
     * Cuánto tiempo se le sigue creyendo a la odometría sin confirmación de
     * visión, en segundos.
     *
     * <p>
     * Dentro de esta ventana, perder el tag no hace que la torreta se ponga a
     * barrer: sigue apuntando por odometría. Un swerve con Pigeon 2.0 deriva muy
     * poco en unos segundos, así que ser generoso aquí sale barato.
     */
    public static final double odometryTrustSeconds = 8.0;

    /**
     * Elegir el HUB según la alianza que reporte la Driver Station.
     *
     * <p>
     * Ponlo en false sólo si colocaron un HUB en una posición arbitraria que
     * midieron ustedes, y entonces usa {@code demoHubPosition}.
     */
    public static final boolean useAllianceHub = true;

    private DemoConstants() {
    }
}
