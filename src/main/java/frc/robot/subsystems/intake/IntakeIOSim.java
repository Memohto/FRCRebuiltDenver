package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {
    private final DCMotorSim rollersSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44(1), 0.001, IntakeConstants.rollersGearRatio), 
            DCMotor.getKrakenX44(1));
    private final DCMotorSim extensorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44(1), 0.025, IntakeConstants.extensorGearRatio),
            DCMotor.getKrakenX44(1));

    private final PIDController extensorController = new PIDController(100, 0.0, 0.0);

    private double rollersAppliedVolts = 0.0;
    private double extensorAppliedVolts = 0.0;

    private boolean closedLoop = false;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        if (closedLoop) {
            extensorAppliedVolts = extensorController.calculate(extensorSim.getAngularPositionRad() / IntakeConstants.extensorGearRatio);
        } else {
            extensorController.reset();
        }

        rollersSim.setInputVoltage(rollersAppliedVolts);
        rollersSim.update(0.02);
        
        inputs.rollersConnected = true;
        inputs.rollersVelocityRadPerSec = rollersSim.getAngularVelocityRadPerSec();
        inputs.rollersAppliedVolts = rollersAppliedVolts;
        inputs.rollersCurrentAmps = rollersSim.getCurrentDrawAmps();
        
        extensorSim.setInputVoltage(extensorAppliedVolts);
        extensorSim.update(0.02);

        inputs.extensorConnected = true;
        inputs.extensorPosition = Rotation2d.fromRadians(extensorSim.getAngularPositionRad() / IntakeConstants.extensorGearRatio);
        inputs.extensorVelocityRadPerSec = extensorSim.getAngularVelocityRadPerSec();
        inputs.extensorAppliedVolts = extensorAppliedVolts;
        inputs.extensorCurrentAmps = extensorSim.getCurrentDrawAmps();
    }

    @Override
    public void setRollersOpenLoop(double speed) {
        rollersAppliedVolts = speed * 12;   
    }

    @Override
    public void setExtensorOpenLoop(double speed) {
        closedLoop = false;
        extensorAppliedVolts = speed * 12;
    }

    @Override
    public void setExtensorPosition(Rotation2d rotation) {
        closedLoop = true;
        extensorController.setSetpoint(rotation.getRadians());
    }
}