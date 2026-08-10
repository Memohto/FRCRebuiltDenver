package frc.robot.commands.demo;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.DemoConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

/**
 * Comandos de intake del Demo Mode: agitación y extensión con remate por
 * corriente.
 */
public class DemoIntakeCommands {

    private DemoIntakeCommands() {
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
                    intake.setRollersOpenLoop(DemoConstants.agitateRollerSpeed);

                    if (elapsed < DemoConstants.agitatePrerollSeconds) {
                        // Fase 1: sólo rodillo. El extensor se queda donde está
                        // para no empujar hacia afuera lo que está entrando.
                        intake.holdExtensorHere();
                        Logger.recordOutput("Demo/Agitate/Phase", "PREROLL");
                        return;
                    }

                    double t = elapsed - DemoConstants.agitatePrerollSeconds;
                    double envelope = Math.exp(-t / DemoConstants.agitateDecaySeconds);

                    double center = DemoConstants.agitateEndCenterRad
                            + (DemoConstants.agitateStartCenterRad - DemoConstants.agitateEndCenterRad)
                                    * envelope;
                    double amplitude = DemoConstants.agitateAmplitudeRad * envelope;

                    double setpoint = center + amplitude
                            * Math.sin(2.0 * Math.PI * DemoConstants.agitateFrequencyHz * t);
                    setpoint = MathUtil.clamp(
                            setpoint, DemoConstants.agitateMinRad, DemoConstants.agitateMaxRad);

                    intake.setExtensorPositionRad(setpoint);

                    Logger.recordOutput("Demo/Agitate/Phase", "SHAKE");
                    Logger.recordOutput("Demo/Agitate/SetpointRad", setpoint);
                    Logger.recordOutput("Demo/Agitate/Envelope", envelope);
                },
                () -> {
                    intake.setExtendedReset();
                    intake.stopRollers();
                    Logger.recordOutput("Demo/Agitate/Phase", "IDLE");
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
                        Logger.recordOutput("Demo/Intake/HomingPhase", "POSITION_ONLY");
                        return;
                    }

                    switch (phase[0]) {
                        case 0 -> {
                            // Fase rápida: lazo cerrado hasta cerca del tope.
                            intake.setExtended();
                            Logger.recordOutput("Demo/Intake/HomingPhase", "POSITION");
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
                            Logger.recordOutput("Demo/Intake/HomingPhase", "SEEKING");

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
                                Logger.recordOutput("Demo/Intake/StallFoundRad", holdPosition[0]);
                                Logger.recordOutput("Demo/Intake/StallConfirmed", confirmed);
                            }
                        }
                        default -> {
                            // Sostener despegado del tope.
                            intake.setExtensorPositionRad(holdPosition[0]);
                            Logger.recordOutput("Demo/Intake/HomingPhase", "HOLDING");
                        }
                    }
                },
                () -> {
                    // Los soft limits SIEMPRE se restauran, sin importar cómo
                    // termine el comando. Si se quedaran apagados, el siguiente
                    // jog manual podría destrozar el rack.
                    intake.setSoftwareLimit(true);
                    intake.stopExtensor();
                    Logger.recordOutput("Demo/Intake/HomingPhase", "IDLE");
                },
                intake)
                .beforeStarting(() -> {
                    phase[0] = 0;
                    phaseTimer.restart();
                    stallTimer.reset();
                });
    }
}
