package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;

public class TransitionShift {

    private static final double SHIFT_1_START = 130.0;
    private static final double SHIFT_2_START = 105.0;
    private static final double SHIFT_3_START = 80.0;
    private static final double SHIFT_4_START = 55.0;
    private static final double END_GAME_START = 30.0;

    private boolean wonAuto = false;
    private final NetworkTable table;

    public TransitionShift() {
        table = NetworkTableInstance.getDefault().getTable("TransitionShift");
    }

    public void setWonAuto(boolean won) {
        this.wonAuto = won;
    }

    public void update() {
        double timeLeft = DriverStation.getMatchTime();
        String currentShift = getCurrentShift(timeLeft);
        boolean hubActive = isHubActive(currentShift);
        double timeToNextShift = getTimeToNextShift(timeLeft, currentShift);

        table.getEntry("Current Shift").setString(currentShift);
        table.getEntry("Hub Active").setBoolean(hubActive);
        table.getEntry("Time Left").setDouble(timeLeft);
        table.getEntry("Time To Next Shift").setDouble(timeToNextShift);
        table.getEntry("Won Auto").setBoolean(wonAuto);
    }

    private String getCurrentShift(double timeLeft) {
        if (timeLeft <= 0) return "NO MATCH";
        if (timeLeft > SHIFT_1_START) return "TRANSITION SHIFT";
        if (timeLeft > SHIFT_2_START) return "SHIFT 1";
        if (timeLeft > SHIFT_3_START) return "SHIFT 2";
        if (timeLeft > SHIFT_4_START) return "SHIFT 3";
        if (timeLeft > END_GAME_START) return "SHIFT 4";
        return "END GAME";
    }

    private double getTimeToNextShift(double timeLeft, String currentShift) {
        switch (currentShift) {
            case "TRANSITION SHIFT": return timeLeft - SHIFT_1_START;
            case "SHIFT 1": return timeLeft - SHIFT_2_START;
            case "SHIFT 2": return timeLeft - SHIFT_3_START;
            case "SHIFT 3": return timeLeft - SHIFT_4_START;
            case "SHIFT 4": return timeLeft - END_GAME_START;
            default: return 0;
        }
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