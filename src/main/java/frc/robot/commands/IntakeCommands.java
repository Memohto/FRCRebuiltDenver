package frc.robot.commands;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

/**
 * Comandos de intake compartidos por competencia y demo: agitación de la caja
 * y extensión con remate por corriente.
 *
 * <p>
 * Se probaron en el demo del StingyCamp y se migraron tal cual; sus constantes
 * viven en {@link IntakeConstants}. {@code DemoIntakeCommands} delega aquí.
 */
public class IntakeCommands {

    private IntakeCommands() {
    }

    /** Comando de Denver (no está en uso en ningún binding, se conserva). */
    public static Command joystickIntakeCmd(
        Intake intake,
        BooleanSupplier intakeupplier,
        BooleanSupplier outtakeupplier) {
        return Commands.run(
            () -> {
                boolean in = intakeupplier.getAsBoolean() && !outtakeupplier.getAsBoolean();
                boolean out = outtakeupplier.getAsBoolean() && !intakeupplier.getAsBoolean();
                boolean extend = in || out;

                if (extend) {
                    intake.setExtended();
                    if (in) {
                        intake.intake();
                    } else {
                        intake.outtake();
                    }
                } else {
                    intake.setExtendedReset();
                    intake.stopRollers();
                }
            },
        intake);
    }

    // ════════════════════════════════════════════════════════════════════════
    // RODILLOS CON BOOST POR CARGA
    // ════════════════════════════════════════════════════════════════════════

    /**
     * Rodillos de intake a velocidad base, con boost automático cuando "piden".
     *
     * <h2>El problema que resuelve</h2>
     *
     * A 0.8 de duty el intake traga bien con la caja vacía, pero cuando va llena
     * el rodillo tiene que empujar cada pelota nueva contra las que ya están, se
     * frena, y a duty fijo el par que le queda no alcanza. Subir la base a 1.0
     * todo el tiempo no es la respuesta: come las pelotas de más y calienta.
     *
     * <h2>Cómo funciona</h2>
     *
     * <ol>
     * <li><b>Base:</b> gira a {@code rollersSpeed} y los primeros
     * {@code rollersBoostLearnSeconds} aprende su velocidad libre.</li>
     * <li><b>Detección:</b> corriente de estator por arriba de
     * {@code rollersBoostCurrentAmps} sostenida {@code rollersBoostDebounceSeconds},
     * <i>o</i> velocidad por debajo de {@code rollersBoostVelocityDroopFraction}
     * de la libre. Cualquiera de las dos = está pidiendo.</li>
     * <li><b>Boost:</b> el duty sube en rampa a {@code rollersBoostSpeed} y se
     * queda ahí mientras dure la carga más {@code rollersBoostHoldSeconds},
     * para no parpadear entre pelota y pelota.</li>
     * <li><b>Protección:</b> si el boost lleva {@code rollersBoostMaxSeconds}
     * seguidos sin que la carga ceda, algo está atorado: baja a la base
     * {@code rollersBoostCooldownSeconds} y vuelve a intentar.</li>
     * </ol>
     *
     * <p>
     * Con {@code rollersBoostEnabled = false} es exactamente {@code intake()} de
     * siempre. Se loguea en {@code Intake/Boost/...} para tunear los umbrales.
     */
    public static Command intakeWithBoost(Intake intake) {
        Timer runTimer = new Timer();
        Timer loadTimer = new Timer();
        Timer holdTimer = new Timer();
        Timer boostTimer = new Timer();
        Timer cooldownTimer = new Timer();
        // [0] duty actual, [1] velocidad libre aprendida
        double[] state = { 0.0, 0.0 };
        boolean[] boosting = { false };
        boolean[] coolingDown = { false };

        return Commands.runEnd(
                () -> {
                    double base = IntakeConstants.rollersSpeed;
                    if (!IntakeConstants.rollersBoostEnabled) {
                        intake.intake();
                        Logger.recordOutput("Intake/Boost/Phase", "OFF");
                        return;
                    }

                    double dt = 0.02;
                    double current = intake.getRollersCurrentAmps();
                    double velocity = Math.abs(intake.getRollersVelocityRadPerSec());

                    // ── Velocidad libre: el máximo visto en la ventana de aprendizaje.
                    // Si arranca con la caja ya llena, aprende una referencia baja
                    // y la señal de velocidad simplemente no dispara; la de
                    // corriente sigue funcionando.
                    if (!runTimer.hasElapsed(IntakeConstants.rollersBoostLearnSeconds)) {
                        state[1] = Math.max(state[1], velocity);
                    }

                    // ── Detección de carga ───────────────────────────────────
                    boolean overCurrent = current > IntakeConstants.rollersBoostCurrentAmps;
                    if (overCurrent) {
                        if (!loadTimer.isRunning()) {
                            loadTimer.restart();
                        }
                    } else {
                        loadTimer.stop();
                        loadTimer.reset();
                    }
                    boolean currentConfirmed = loadTimer.isRunning()
                            && loadTimer.hasElapsed(IntakeConstants.rollersBoostDebounceSeconds);

                    boolean droop = IntakeConstants.rollersBoostVelocityDroopFraction > 0.0
                            && runTimer.hasElapsed(IntakeConstants.rollersBoostLearnSeconds)
                            && state[1] > 1.0
                            && velocity < state[1] * IntakeConstants.rollersBoostVelocityDroopFraction;

                    boolean loaded = currentConfirmed || droop;

                    // ── Cooldown: atorado demasiado tiempo, descansar en la base ──
                    if (coolingDown[0]) {
                        if (cooldownTimer.hasElapsed(IntakeConstants.rollersBoostCooldownSeconds)) {
                            coolingDown[0] = false;
                            boostTimer.stop();
                            boostTimer.reset();
                        } else {
                            loaded = false;
                        }
                    }

                    // ── Máquina de boost ─────────────────────────────────────
                    if (loaded) {
                        holdTimer.restart();
                        if (!boosting[0]) {
                            boosting[0] = true;
                            boostTimer.restart();
                        }
                    } else if (boosting[0]
                            && holdTimer.hasElapsed(IntakeConstants.rollersBoostHoldSeconds)) {
                        boosting[0] = false;
                        boostTimer.stop();
                        boostTimer.reset();
                    }

                    if (boosting[0] && boostTimer.hasElapsed(IntakeConstants.rollersBoostMaxSeconds)) {
                        // Lleva demasiado al 100% sin que ceda: descanso.
                        boosting[0] = false;
                        coolingDown[0] = true;
                        cooldownTimer.restart();
                    }

                    // ── Rampa del duty ───────────────────────────────────────
                    double target = boosting[0] ? IntakeConstants.rollersBoostSpeed : base;
                    double rate = target > state[0]
                            ? IntakeConstants.rollersBoostRampUpPerSec
                            : IntakeConstants.rollersBoostRampDownPerSec;
                    double step = MathUtil.clamp(target - state[0], -rate * dt, rate * dt);
                    state[0] = MathUtil.clamp(state[0] + step, 0.0, 1.0);

                    intake.setRollersOpenLoop(state[0]);

                    Logger.recordOutput("Intake/Boost/Phase",
                            coolingDown[0] ? "COOLDOWN" : boosting[0] ? "BOOST" : "BASE");
                    Logger.recordOutput("Intake/Boost/Duty", state[0]);
                    Logger.recordOutput("Intake/Boost/Loaded", loaded);
                    Logger.recordOutput("Intake/Boost/OverCurrent", overCurrent);
                    Logger.recordOutput("Intake/Boost/VelocityDroop", droop);
                    Logger.recordOutput("Intake/Boost/FreeVelocityRadPerSec", state[1]);
                },
                () -> {
                    intake.stopRollers();
                    Logger.recordOutput("Intake/Boost/Phase", "IDLE");
                    Logger.recordOutput("Intake/Boost/Duty", 0.0);
                },
                intake)
                .beforeStarting(() -> {
                    // Arranca directo en la base: la rampa es para el boost, no
                    // para el arranque, que en Denver siempre fue un escalón.
                    state[0] = IntakeConstants.rollersSpeed;
                    state[1] = 0.0;
                    boosting[0] = false;
                    coolingDown[0] = false;
                    runTimer.restart();
                    loadTimer.stop();
                    loadTimer.reset();
                    holdTimer.reset();
                    boostTimer.stop();
                    boostTimer.reset();
                    cooldownTimer.reset();
                });
    }

    // ════════════════════════════════════════════════════════════════════════
    // AGITACIÓN
    // ════════════════════════════════════════════════════════════════════════

    /**
     * Agita la caja para romper puentes de pelotas.
     *
     * <h2>Fase 1 — Pre-roll ({@code agitatePrerollSeconds}, medio segundo)</h2>
     *
     * Sólo gira el rodillo, sin mover el extensor. Es para las pelotas que
     * quedaron a medio camino en la rampa: si empiezas a sacudir de inmediato,
     * esas pelotas salen disparadas hacia afuera en vez de terminar de entrar.
     * Medio segundo de rodillo solo las acomoda adentro.
     *
     * <h2>Fase 2 — Sacudida con envolvente decreciente</h2>
     *
     * <pre>
     *   envolvente(t) = e^(−t / τ)
     *   setpoint(t)   = centro(t) + amplitud(t) · sin(2π f t)
     * </pre>
     *
     * El centro converge a la posición cerrada mientras la amplitud se apaga, así
     * que la caja termina cerrada con todas las pelotas asentadas. El rodillo
     * sigue girando hacia adentro todo el tiempo para barrer de vuelta lo que la
     * sacudida empuje hacia afuera.
     *
     * <p>
     * Con los valores por defecto (τ = 2.5 s): a los 0.5 s arranca la sacudida
     * oscilando entre 6 y 16 rad, a los 3 s ya está entre 4.3 y 7, y a los 8 s
     * está prácticamente cerrada en 4 rad.
     */
    public static Command agitate(Intake intake) {
        Timer timer = new Timer();

        return Commands.runEnd(
                () -> {
                    double elapsed = timer.get();
                    intake.setRollersOpenLoop(IntakeConstants.agitateRollerSpeed);

                    if (elapsed < IntakeConstants.agitatePrerollSeconds) {
                        // Fase 1: sólo rodillo. El extensor se queda donde está
                        // para no empujar hacia afuera lo que está entrando.
                        intake.holdExtensorHere();
                        Logger.recordOutput("Intake/Agitate/Phase", "PREROLL");
                        return;
                    }

                    double t = elapsed - IntakeConstants.agitatePrerollSeconds;
                    double envelope = Math.exp(-t / IntakeConstants.agitateDecaySeconds);

                    double center = IntakeConstants.agitateEndCenterRad
                            + (IntakeConstants.agitateStartCenterRad - IntakeConstants.agitateEndCenterRad)
                                    * envelope;
                    double amplitude = IntakeConstants.agitateAmplitudeRad * envelope;

                    double setpoint = center + amplitude
                            * Math.sin(2.0 * Math.PI * IntakeConstants.agitateFrequencyHz * t);
                    setpoint = MathUtil.clamp(
                            setpoint, IntakeConstants.agitateMinRad, IntakeConstants.agitateMaxRad);

                    intake.setExtensorPositionRad(setpoint);

                    Logger.recordOutput("Intake/Agitate/Phase", "SHAKE");
                    Logger.recordOutput("Intake/Agitate/SetpointRad", setpoint);
                    Logger.recordOutput("Intake/Agitate/Envelope", envelope);
                },
                () -> {
                    intake.setExtendedReset();
                    intake.stopRollers();
                    Logger.recordOutput("Intake/Agitate/Phase", "IDLE");
                },
                intake)
                .beforeStarting(timer::restart);
    }

    // ════════════════════════════════════════════════════════════════════════
    // EXTENSIÓN CON REMATE POR CORRIENTE
    // ════════════════════════════════════════════════════════════════════════

    /**
     * Extiende el intake y remata contra el tope mecánico detectándolo por
     * corriente.
     *
     * <h2>El problema que resuelve</h2>
     *
     * Con un objetivo de posición fijo, si la calibración se corre unos grados
     * —porque el encoder arrancó en otro lado, porque el rack se movió, porque
     * el ratio del gearbox tiene división entera— el intake se queda corto. Y un
     * intake que no sale por completo no traga bien.
     *
     * <h2>Cómo funciona</h2>
     *
     * <ol>
     * <li><b>Fase rápida:</b> lazo cerrado a {@code extendedRotation}. Recorre
     * casi todo el camino en el menor tiempo posible.</li>
     * <li><b>Fase de remate:</b> apaga los soft limits y empuja hacia afuera en
     * lazo abierto a {@code stallHomingSpeed} (12%). Despacio a propósito: el
     * objetivo es sentir el tope, no golpearlo.</li>
     * <li><b>Detección:</b> cuando la corriente de estator supera
     * {@code stallCurrentAmps} sostenida por {@code stallDebounceSeconds}, es el
     * tope. El debounce existe porque el pico de arranque del motor se vería
     * igual que un tope.</li>
     * <li><b>Retroceso:</b> se despega {@code stallBackoffRad} y se queda ahí en
     * lazo cerrado. Dejar el mecanismo forzando su propio tope calienta el motor
     * y desgasta el rack.</li>
     * </ol>
     *
     * <p>
     * La gran ventaja es que <b>no depende de calibración</b>: el tope mecánico
     * está donde está, y el robot lo encuentra solo cada vez.
     *
     * <p>
     * Si {@code IntakeConstants.useStallHoming} está en false, se comporta como
     * la extensión de siempre.
     */
    public static Command extendWithStallHoming(Intake intake) {
        Timer phaseTimer = new Timer();
        Timer stallTimer = new Timer();
        // Índice 0 = fase actual (0 posición, 1 remate, 2 sostener)
        int[] phase = { 0 };
        double[] holdPosition = { 0.0 };

        return Commands.runEnd(
                () -> {
                    if (!IntakeConstants.useStallHoming) {
                        intake.setExtended();
                        Logger.recordOutput("Intake/Homing/HomingPhase", "POSITION_ONLY");
                        return;
                    }

                    switch (phase[0]) {
                        case 0 -> {
                            // Fase rápida: lazo cerrado hasta cerca del tope.
                            intake.setExtended();
                            Logger.recordOutput("Intake/Homing/HomingPhase", "POSITION");
                            boolean closeEnough = intake.getExtensorPositionRad()
                                    > IntakeConstants.extendedRotation.getRadians() - 1.0;
                            if (closeEnough || phaseTimer.hasElapsed(1.5)) {
                                phase[0] = 1;
                                phaseTimer.restart();
                                stallTimer.reset();
                                intake.setSoftwareLimit(false);
                            }
                        }
                        case 1 -> {
                            // Remate: empuje lento hasta sentir el tope.
                            intake.extendAtSpeed(IntakeConstants.stallHomingSpeed);
                            Logger.recordOutput("Intake/Homing/HomingPhase", "SEEKING");

                            boolean stalled =
                                    intake.getExtensorCurrentAmps() > IntakeConstants.stallCurrentAmps;
                            if (stalled) {
                                if (!stallTimer.isRunning()) {
                                    stallTimer.restart();
                                }
                            } else {
                                stallTimer.stop();
                                stallTimer.reset();
                            }

                            boolean confirmed = stallTimer.isRunning()
                                    && stallTimer.hasElapsed(IntakeConstants.stallDebounceSeconds);

                            if (confirmed || phaseTimer.hasElapsed(IntakeConstants.stallTimeoutSeconds)) {
                                holdPosition[0] = intake.getExtensorPositionRad()
                                        - IntakeConstants.stallBackoffRad;
                                intake.stopExtensor();
                                intake.setSoftwareLimit(true);
                                phase[0] = 2;
                                Logger.recordOutput("Intake/Homing/StallFoundRad", holdPosition[0]);
                                Logger.recordOutput("Intake/Homing/StallConfirmed", confirmed);
                            }
                        }
                        default -> {
                            // Sostener despegado del tope.
                            intake.setExtensorPositionRad(holdPosition[0]);
                            Logger.recordOutput("Intake/Homing/HomingPhase", "HOLDING");
                        }
                    }
                },
                () -> {
                    // Los soft limits SIEMPRE se restauran, sin importar cómo
                    // termine el comando. Si se quedaran apagados, el siguiente
                    // jog manual podría destrozar el rack.
                    intake.setSoftwareLimit(true);
                    intake.stopExtensor();
                    Logger.recordOutput("Intake/Homing/HomingPhase", "IDLE");
                },
                intake)
                .beforeStarting(() -> {
                    phase[0] = 0;
                    phaseTimer.restart();
                    stallTimer.reset();
                });
    }
}
