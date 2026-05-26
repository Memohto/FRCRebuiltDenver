package frc.robot;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Timer extends SubsystemBase implements LoggableInputs {


    double t = 0;
    String phaseString = "N/A";
    

    @Override
    public void periodic() {
        t = DriverStation.getMatchTime();

        if (t < 0) return;

        if (DriverStation.isTeleop()) {
            if (t > 130) {
                phaseString = "Transition & Shift start";
            } else if (t > 105) {
                phaseString = "Alliance Shift 1";
            } else if (t > 80) {
                phaseString = "Alliance Shift 2";
            } else if (t > 55) {
                phaseString = "Alliance Shift 3";
            } else if (t > 30) {
                phaseString = "Alliance Shift 4";
            } else {
                phaseString = "Endgame";
            }
        }

        Logger.processInputs("Timer", this);
    }

    @Override
    public void toLog(LogTable table) {
        table.put("matchTime", t);
        table.put("phase", phaseString);
    }

    @Override
    public void fromLog(LogTable table) {
        t = table.get("matchTime", 0.0);
        phaseString = table.get("phase", "N/A");
    }
}