package frc.robot.subsystems.tejuino;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Manually written AutoLogged class for TejuinoIO.
 * This replaces what @AutoLog would generate, so no build step is needed.
 */
public class TejuinoIOInputsAutoLogged extends TejuinoIO.TejuinoIOInputs
    implements LoggableInputs, Cloneable {

  @Override
  public void toLog(LogTable table) {
    table.put("LastStripIndex", lastStripIndex);
    table.put("LastR",          lastR);
    table.put("LastG",          lastG);
    table.put("LastB",          lastB);
    table.put("LastEffect",     lastEffect);
  }

  @Override
  public void fromLog(LogTable table) {
    lastStripIndex = table.get("LastStripIndex", lastStripIndex);
    lastR          = table.get("LastR",          lastR);
    lastG          = table.get("LastG",          lastG);
    lastB          = table.get("LastB",          lastB);
    lastEffect     = table.get("LastEffect",     lastEffect);
  }

  @Override
  public TejuinoIOInputsAutoLogged clone() {
    TejuinoIOInputsAutoLogged copy = new TejuinoIOInputsAutoLogged();
    copy.lastStripIndex = this.lastStripIndex.clone();
    copy.lastR          = this.lastR.clone();
    copy.lastG          = this.lastG.clone();
    copy.lastB          = this.lastB.clone();
    copy.lastEffect     = this.lastEffect.clone();
    return copy;
  }
}