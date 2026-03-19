package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.shooter.ShooterIO.*;

public class TurretIOSim implements TurretIO {
    private final DCMotorSim rotationMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), 0.000001, TurretConstants.rotationMotorGearRatio),
        DCMotor.getKrakenX44(1));

    private final PIDController rotationMotorController = new PIDController(50, 0, 10);

    private double appliedVolts = 0.0;
    private boolean closedLoop = false;

    @Override
    public void updateInputs(ShooterIOInputs shooterInputs, TurretIOInputs turretInputs) {
        if (closedLoop) {
            appliedVolts = rotationMotorController.calculate(rotationMotorSim.getAngularPositionRad() / TurretConstants.rotationMotorGearRatio);
        } else {
            rotationMotorController.reset();
        }

        rotationMotorSim.setInputVoltage(appliedVolts);
        rotationMotorSim.update(0.02);

        // Integrate position and clamp to soft limits
        double rotationPositionRad = rotationMotorSim.getAngularPositionRad() / TurretConstants.rotationMotorGearRatio;
        rotationPositionRad = MathUtil.clamp(
            rotationPositionRad,
            TurretConstants.minRotationRad,
            TurretConstants.maxRotationRad);

        turretInputs.rotationMotorConnected = true;
        turretInputs.rotationMotorPosition = Rotation2d.fromRadians(rotationPositionRad);
        turretInputs.rotationMotorVelocityRadPerSec = rotationMotorSim.getAngularVelocityRadPerSec();
        turretInputs.rotationMotorAppliedVolts = appliedVolts;
        turretInputs.rotationMotorCurrentAmps = rotationMotorSim.getCurrentDrawAmps();
    }

    @Override
    public void setRotationMotorPosition(Rotation2d rotation) {
        closedLoop = true;
        rotationMotorController.setSetpoint(rotation.getRadians());
    }

    @Override
    public void setRotationMotorOpenLoop(double speed) {
        closedLoop = false;
        appliedVolts = speed * 12;
    }
}
