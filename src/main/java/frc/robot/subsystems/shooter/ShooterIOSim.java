package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.ShooterConstants;

public class ShooterIOSim implements ShooterIO {
    private final DCMotorSim flywheelSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44(1), 0.001, ShooterConstants.flywheelGearRatio), 
            DCMotor.getKrakenX44(1));
    
    private double flywheelAppliedVolts = 0.0;
    private double targetVelocityRadPerSec = 0.0;

    // Simple proportional sim for closed-loop — not perfectly accurate
    // but gives you a realistic feel of the flywheel spinning up in simulation.
    private static final double kSimVoltagePerRadPerSec = 12.0 / (80.0 * 2 * Math.PI);

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        flywheelSim.setInputVoltage(flywheelAppliedVolts);
        flywheelSim.update(0.02);
        
        inputs.flywheelConnected = true;
        inputs.flywheelVelocityRadPerSec = flywheelSim.getAngularVelocityRadPerSec();
        inputs.flywheelAppliedVolts = flywheelAppliedVolts;
        inputs.flywheelCurrentAmps = flywheelSim.getCurrentDrawAmps();
        inputs.flywheelTargetVelocityRadPerSec = targetVelocityRadPerSec;
    }

    @Override
    public void setFlywheelOpenLoop(double speed) {
        targetVelocityRadPerSec = 0.0;
        flywheelAppliedVolts = speed * 12.0;
    }

    /**
     * Simulated closed-loop: approximates the voltage the feedforward would apply
     * to reach the target speed. Not a real PID sim, but good enough to test
     * command logic and AdvantageKit logging in simulation.
     */
    @Override
    public void setFlywheelVelocity(double velocityRadPerSec) {
        targetVelocityRadPerSec = velocityRadPerSec;
        // Approximate: kV * targetRPS gives the feedforward voltage
        flywheelAppliedVolts = Math.min(velocityRadPerSec * kSimVoltagePerRadPerSec, 12.0);
    }

    @Override
    public void setHoodOpenLoop(double speed) { }

    @Override
    public void setHoodPosition(Rotation2d rotation) { }
}
