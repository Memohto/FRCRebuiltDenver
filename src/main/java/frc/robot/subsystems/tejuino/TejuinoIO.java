package frc.robot.subsystems.tejuino;

import org.littletonrobotics.junction.AutoLog;

public interface TejuinoIO {

    @AutoLog
    class TejuinoIOInputs {
        // Tejuino is a pure-output device; we log what we last commanded
        // so replays can reconstruct LED state.
        public int[] lastStripIndex  = new int[]{0};
        public int[] lastR           = new int[]{0};
        public int[] lastG           = new int[]{0};
        public int[] lastB           = new int[]{0};
        public String[] lastEffect   = new String[]{""};
    }

    /** Initialize the CAN handle for the given device number (0 or 1). */
    default void init(int deviceNumber) {}

    /** Set all LEDs on a strip to an RGB color. */
    default void setAll(int strip, int r, int g, int b) {}

    /** Set a single LED on a strip to an RGB color. */
    default void setSingle(int strip, int ledIndex, int r, int g, int b) {}

    /** Turn off all LEDs on a strip. */
    default void turnOff(int strip) {}

    /** Play a built-in named effect on a strip. */
    default void setEffect(int strip, Tejuino.Effect effect) {}

    /** Update logged inputs (called every loop). */
    default void updateInputs(TejuinoIOInputs inputs) {}
}
