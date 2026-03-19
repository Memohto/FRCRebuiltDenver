package frc.robot.subsystems.tejuino;

import edu.wpi.first.hal.CANAPIJNI;

/**
 * Real hardware implementation for the Tejuino Board V1.1.
 * Wraps the official IOT4U CAN packet API.
 */
public class TejuinoIOReal implements TejuinoIO {

    private static final int TEJUINO_MANUFACTURER = 8;
    private static final int TEJUINO_DEVICE_TYPE  = 10;
    private static final int TEJUINO_LED_API_ID   = 0;

    private int canHandle = 0;

    // Last commanded state for logging
    private int logStrip  = 0;
    private int logR      = 0;
    private int logG      = 0;
    private int logB      = 0;
    private String logEffect = "";

    @Override
    public void init(int deviceNumber) {
        canHandle = CANAPIJNI.initializeCAN(TEJUINO_MANUFACTURER, deviceNumber, TEJUINO_DEVICE_TYPE);
    }

    @Override
    public void setAll(int strip, int r, int g, int b) {
        byte[] data = new byte[8];
        data[0] = 0;
        data[1] = 0;
        data[2] = (byte) r;
        data[3] = (byte) g;
        data[4] = (byte) b;
        data[5] = 0;
        data[6] = 1;           // "all" flag
        data[7] = (byte) strip;
        CANAPIJNI.writeCANPacket(canHandle, data, TEJUINO_LED_API_ID);
        logStrip = strip; logR = r; logG = g; logB = b; logEffect = "";
    }

    @Override
    public void setSingle(int strip, int ledIndex, int r, int g, int b) {
        byte[] data = new byte[8];
        data[0] = (byte) ledIndex;
        data[1] = 0;
        data[2] = (byte) r;
        data[3] = (byte) g;
        data[4] = (byte) b;
        data[5] = 0;
        data[6] = 0;           // single-LED flag
        data[7] = (byte) strip;
        CANAPIJNI.writeCANPacket(canHandle, data, TEJUINO_LED_API_ID);
        logStrip = strip; logR = r; logG = g; logB = b; logEffect = "";
    }

    @Override
    public void turnOff(int strip) {
        setAll(strip, 0, 0, 0);
        logEffect = "OFF";
    }

    @Override
    public void setEffect(int strip, Tejuino.Effect effect) {
        byte effectByte = switch (effect) {
            case RAINBOW  -> 1;
            case STINGBOT -> 2;
            case ESCUDERIA -> 3;
        };
        byte[] data = new byte[8];
        data[0] = 0;
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;
        data[4] = 0;
        data[5] = effectByte;
        data[6] = 0;
        data[7] = (byte) strip;
        CANAPIJNI.writeCANPacket(canHandle, data, TEJUINO_LED_API_ID);
        logStrip = strip; logR = 0; logG = 0; logB = 0; logEffect = effect.name();
    }

    @Override
    public void updateInputs(TejuinoIOInputs inputs) {
        inputs.lastStripIndex = new int[]{ logStrip };
        inputs.lastR          = new int[]{ logR };
        inputs.lastG          = new int[]{ logG };
        inputs.lastB          = new int[]{ logB };
        inputs.lastEffect     = new String[]{ logEffect };
    }
}
