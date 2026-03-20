package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TransitionShift {

    private static final double SHIFT_1_START = 130.0;  // 2:10
    private static final double SHIFT_2_START = 105.0;  // 1:45
    private static final double SHIFT_3_START = 80.0;   // 1:20
    private static final double SHIFT_4_START = 55.0;   // 0:55
    private static final double END_GAME_START = 30.0;  // 0:30

    private boolean wonAuto = false;

    public void setWonAuto(boolean won) {
        this.wonAuto = won;
    }

    public void update() {
        double timeLeft = DriverStation.getMatchTime();
        String currentShift = getCurrentShift(timeLeft);
        boolean hubActive = isHubActive(currentShift);
        SmartDashboard.putString("Current Shift", currentShift);
        SmartDashboard.putBoolean("Hub Active", hubActive);
        SmartDashboard.putNumber("Time Left", timeLeft);
    }

    private String getCurrentShift(double timeLeft) {

        if (!SmartDashboard.getBoolean("Match Running", false)) return "NO MATCH";
        if (timeLeft > SHIFT_1_START) return "TRANSITION SHIFT";
        if (timeLeft > SHIFT_2_START) return "SHIFT 1";
        if (timeLeft > SHIFT_3_START) return "SHIFT 2";
        if (timeLeft > SHIFT_4_START) return "SHIFT 3";
        if (timeLeft > END_GAME_START) return "SHIFT 4";
        return "END GAME";
    }

    private boolean isHubActive(String shift) {
        if (shift.equals("NO MATCH")) return false;
        if (shift.equals("TRANSITION SHIFT") || shift.equals("END GAME")) return true;
        boolean isOddShift = shift.equals("SHIFT 1") || shift.equals("SHIFT 3");
        if (wonAuto) {
            return !isOddShift;
        } else {
            return isOddShift;
        }
    }
}