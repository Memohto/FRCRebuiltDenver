package frc.robot.subsystems.indexer;

import frc.robot.constants.IndexerConstants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO {
    private final DCMotorSim rollersSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44(1), 0.001, IndexerConstants.rollersGearRatio), 
            DCMotor.getKrakenX44(1));
    private final DCMotorSim shooterWeelsSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44(1), 0.001, IndexerConstants.shooterWheelsGearRatio),
            DCMotor.getKrakenX44(1));
    private final DCMotorSim turretWeelsSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44(1), 0.001, IndexerConstants.shooterWheelsGearRatio),
            DCMotor.getKrakenX44(1));
    private final DCMotorSim feederSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44(1), 0.001, IndexerConstants.shooterWheelsGearRatio),
            DCMotor.getKrakenX44(1));

    private double rollersAppliedVolts = 0.0;
    private double shooterWheelsAppliedVolts = 0.0;
    private double turretWheelsAppliedVolts = 0.0;
    private double feederAppliedVolts = 0.0;

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        rollersSim.setInputVoltage(rollersAppliedVolts);
        rollersSim.update(0.02);
        inputs.rollersConnected = true;
        inputs.rollersVelocityRadPerSec = rollersSim.getAngularVelocityRadPerSec();
        inputs.rollersAppliedVolts = rollersAppliedVolts;
        inputs.rollersCurrentAmps = rollersSim.getCurrentDrawAmps();

        shooterWeelsSim.setInputVoltage(shooterWheelsAppliedVolts);
        shooterWeelsSim.update(0.02);
        inputs.shooterWheelsConnected = true;
        inputs.shooterWheelsVelocityRadPerSec = shooterWeelsSim.getAngularVelocityRadPerSec();
        inputs.shooterWheelsAppliedVolts = shooterWheelsAppliedVolts;
        inputs.shooterWheelsCurrentAmps = shooterWeelsSim.getCurrentDrawAmps();

        turretWeelsSim.setInputVoltage(turretWheelsAppliedVolts);
        turretWeelsSim.update(0.02);
        inputs.turretWheelsConnected = true;
        inputs.turretWheelsVelocityRadPerSec = turretWeelsSim.getAngularVelocityRadPerSec();
        inputs.turretWheelsAppliedVolts = turretWheelsAppliedVolts;
        inputs.turretWheelsCurrentAmps = turretWeelsSim.getCurrentDrawAmps();

        feederSim.setInputVoltage(feederAppliedVolts);
        feederSim.update(0.02);
        inputs.feederConnected = true;
        inputs.feederVelocityRadPerSec = feederSim.getAngularVelocityRadPerSec();
        inputs.feederAppliedVolts = feederAppliedVolts;
        inputs.feederCurrentAmps = feederSim.getCurrentDrawAmps();
    }

    @Override
    public void setRollersOpenLoop(double speed) {
        rollersAppliedVolts = speed * 12;
    }

    @Override
    public void setShooterWheelsOpenLoop(double speed) {
        shooterWheelsAppliedVolts = speed * 12;
    }

    @Override
    public void setTurretWheelsOpenLoop(double speed) {
        turretWheelsAppliedVolts = speed * 12;
    }

    @Override
    public void setFeederOpenLoop(double speed) {
        feederAppliedVolts = speed * 12;
    }
    
}
