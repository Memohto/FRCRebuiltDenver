package frc.robot.subsystems.tejuino;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.TejuinoConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem wrapping the Tejuino Board V1.1 — a CAN-connected LED controller
 * by IOT4U Technology (https://www.iot4utechnology.com).
 *
 * <p>Strips are addressed by the constants:
 * <ul>
 *   <li>{@link TejuinoConstants#ONBOARD}  – onboard LEDs (strip 0)</li>
 *   <li>{@link TejuinoConstants#EXTERNAL1} – first external strip (strip 1)</li>
 *   <li>{@link TejuinoConstants#EXTERNAL2} – second external strip (strip 2)</li>
 * </ul>
 *
 * <p>Usage example in RobotContainer (REAL case):
 * <pre>
 *   tejuino = new Tejuino(new TejuinoIOReal(), TejuinoConstants.DEVICE_NUMBER);
 * </pre>
 */
public class Tejuino extends SubsystemBase {

    /** Built-in firmware animations supported by the Tejuino Board. */
    public enum Effect {
        RAINBOW,
        STINGBOT,
        ESCUDERIA
    }

    private final TejuinoIO io;
    private final TejuinoIOInputsAutoLogged inputs = new TejuinoIOInputsAutoLogged();

    /**
     * @param io           IO implementation (TejuinoIOReal or TejuinoIOSim)
     * @param deviceNumber CAN device number of the board (0 or 1)
     */
    public Tejuino(TejuinoIO io, int deviceNumber) {
        this.io = io;
        io.init(deviceNumber);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Tejuino", inputs);
    }

    // ─────────────────────────────────────────────────────────
    // Solid color helpers
    // ─────────────────────────────────────────────────────────

    /** Set all LEDs on a strip to a WPILib {@link Color}. */
    public void setAll(int strip, Color color) {
        io.setAll(strip, toInt(color.red), toInt(color.green), toInt(color.blue));
    }

    /** Set all LEDs on a strip to raw RGB (0–255). */
    public void setAll(int strip, int r, int g, int b) {
        io.setAll(strip, r, g, b);
    }

    /** Set a single LED on a strip to a WPILib {@link Color}. */
    public void setSingle(int strip, int ledIndex, Color color) {
        io.setSingle(strip, ledIndex, toInt(color.red), toInt(color.green), toInt(color.blue));
    }

    /** Set a single LED on a strip to raw RGB (0–255). */
    public void setSingle(int strip, int ledIndex, int r, int g, int b) {
        io.setSingle(strip, ledIndex, r, g, b);
    }

    /** Turn off all LEDs on a strip. */
    public void turnOff(int strip) {
        io.turnOff(strip);
    }

    /** Turn off all three strips at once. */
    public void turnOffAll() {
        io.turnOff(TejuinoConstants.ONBOARD);
        io.turnOff(TejuinoConstants.EXTERNAL1);
        io.turnOff(TejuinoConstants.EXTERNAL2);
    }

    // ─────────────────────────────────────────────────────────
    // Built-in firmware effects
    // ─────────────────────────────────────────────────────────

    /** Play a built-in firmware animation on a strip. */
    public void setEffect(int strip, Effect effect) {
        io.setEffect(strip, effect);
    }

    // ─────────────────────────────────────────────────────────
    // Convenience preset methods
    // ─────────────────────────────────────────────────────────

    /** Signal robot is disabled — all strips off. */
    public void signalDisabled() {
        turnOffAll();
    }

    /** Signal robot is enabled in teleop — stingbot effect on all strips. */
    public void signalTeleop() {
        io.setEffect(TejuinoConstants.ONBOARD,  Effect.STINGBOT);
        io.setEffect(TejuinoConstants.EXTERNAL1, Effect.STINGBOT);
        io.setEffect(TejuinoConstants.EXTERNAL2, Effect.STINGBOT);
    }

    /** Signal autonomous is running — rainbow on all strips. */
    public void signalAuto() {
        io.setEffect(TejuinoConstants.ONBOARD,  Effect.RAINBOW);
        io.setEffect(TejuinoConstants.EXTERNAL1, Effect.RAINBOW);
        io.setEffect(TejuinoConstants.EXTERNAL2, Effect.RAINBOW);
    }

    /** Signal a game piece has been intaked — brief green flash (call once). */
    public void signalIntake() {
        io.setAll(TejuinoConstants.EXTERNAL1, 0, 255, 0);
        io.setAll(TejuinoConstants.EXTERNAL2, 0, 255, 0);
    }

    /** Signal the shooter is spun up and ready to fire — solid orange. */
    public void signalShooterReady() {
        io.setAll(TejuinoConstants.EXTERNAL1, 255, 80, 0);
        io.setAll(TejuinoConstants.EXTERNAL2, 255, 80, 0);
    }

    // ─────────────────────────────────────────────────────────
    // Internal helper
    // ─────────────────────────────────────────────────────────

    /** Convert WPILib's 0.0–1.0 color channel to 0–255. */
    private int toInt(double channel) {
        return (int) Math.round(channel * 255.0);
    }
}
