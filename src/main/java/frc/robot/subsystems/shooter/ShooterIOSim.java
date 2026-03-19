package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.ShooterConstants;

public class ShooterIOSim implements ShooterIO {
    private final DCMotorSim flywheelSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44(1), 0.001, ShooterConstants.flywheelGearRatio), 
            DCMotor.getKrakenX44(1));
    
    private double flywheelAppliedVolts = 0.0;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        flywheelSim.setInputVoltage(flywheelAppliedVolts);
        flywheelSim.update(0.02);
        
        inputs.flywheelConnected = true;
        inputs.flywheelVelocityRadPerSec = flywheelSim.getAngularVelocityRadPerSec();
        inputs.flywheelAppliedVolts = flywheelAppliedVolts;
        inputs.flywheelCurrentAmps = flywheelSim.getCurrentDrawAmps();
    }

    @Override
    public void setFlywheelOpenLoop(double speed) {
        flywheelAppliedVolts = speed * 12;
    }

    @Override
    public void setHoodOpenLoop(double speed) { }

    @Override
    public void setHoodPosition(Rotation2d rotation) { }
}
