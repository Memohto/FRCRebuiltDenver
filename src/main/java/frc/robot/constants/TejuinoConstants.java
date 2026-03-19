package frc.robot.constants;

/**
 * Constants for the Tejuino Board V1.1 LED controller.
 * Adjust DEVICE_NUMBER to match the physical CAN ID set on the board.
 */
public class TejuinoConstants {

    // ── CAN device number (set via the Tejuino config tool, default 0) ──
    public static final int DEVICE_NUMBER = 0;

    // ── Strip indices (match Tejuino firmware) ──
    public static final int ONBOARD   = 0;   // Onboard LEDs
    public static final int EXTERNAL1 = 1;   // External strip 1
    public static final int EXTERNAL2 = 2;   // External strip 2
}
