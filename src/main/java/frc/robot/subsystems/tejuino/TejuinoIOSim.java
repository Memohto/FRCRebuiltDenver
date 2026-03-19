package frc.robot.subsystems.tejuino;

/**
 * Sim stub for the Tejuino Board.
 * No CAN hardware available in simulation — state is still logged
 * so LED commands show up in AdvantageScope replays.
 */
public class TejuinoIOSim implements TejuinoIO {

    private int logStrip     = 0;
    private int logR         = 0;
    private int logG         = 0;
    private int logB         = 0;
    private String logEffect = "";

    @Override
    public void init(int deviceNumber) { /* no-op in sim */ }

    @Override
    public void setAll(int strip, int r, int g, int b) {
        logStrip = strip; logR = r; logG = g; logB = b; logEffect = "";
    }

    @Override
    public void setSingle(int strip, int ledIndex, int r, int g, int b) {
        logStrip = strip; logR = r; logG = g; logB = b; logEffect = "";
    }

    @Override
    public void turnOff(int strip) {
        logStrip = strip; logR = 0; logG = 0; logB = 0; logEffect = "OFF";
    }

    @Override
    public void setEffect(int strip, Tejuino.Effect effect) {
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
