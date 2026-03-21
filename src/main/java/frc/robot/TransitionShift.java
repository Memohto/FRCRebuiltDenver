package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TransitionShift {

    private static final double MatchLenght = 130.0;  // 2:10

    private static final double SHIFT_1_START = 130.0;  // 2:10
    private static final double SHIFT_2_START = 105.0;  // 1:45
    private static final double SHIFT_3_START = 80.0;   // 1:20
    private static final double SHIFT_4_START = 55.0;   // 0:55
    private static final double END_GAME_START = 30.0;  // 0:30

    private static final double TimePerShift = 25;  // 0:25

   

    public void update() {
        double timeLeft = DriverStation.getMatchTime();
        String currentShift = getCurrentShift(timeLeft);
        //boolean hubActive = isHubActive(currentShift);
        SmartDashboard.putString("Current Shift", currentShift);
       // SmartDashboard.putBoolean("Hub Active", hubActive);
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



    }
