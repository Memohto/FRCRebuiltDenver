package frc.robot.subsystems.turret;

import static frc.robot.util.PhoenixUtil.*;

import frc.robot.constants.RobotConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class TurretIOTalonFX extends ShooterIOTalonFX implements TurretIO {
    private final TalonFX rotationMotor;

    private final StatusSignal<Angle> rotationPosition;
    private final StatusSignal<AngularVelocity> rotationVelocity;
    private final StatusSignal<Voltage> rotationAppliedVolts;
    private final StatusSignal<Current> rotationCurrentAmps;

    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);

    public TurretIOTalonFX(int flywheelCanId, int hoodCanId, int rotationMotorCanId) {
        super(flywheelCanId, hoodCanId);
        rotationMotor = new TalonFX(rotationMotorCanId);

        // Rotation motor config
        TalonFXConfiguration rotationConfig = new TalonFXConfiguration();
        rotationConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rotationConfig.Slot0 = TurretConstants.rotationMotorGains;
        rotationConfig.Feedback.SensorToMechanismRatio = TurretConstants.rotationMotorGearRatio;
        rotationConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        rotationConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        rotationConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
            Units.radiansToRotations(TurretConstants.maxRotationRad);
        rotationConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
            Units.radiansToRotations(TurretConstants.minRotationRad);
        tryUntilOk(5, () -> rotationMotor.getConfigurator().apply(rotationConfig, 0.25));
        tryUntilOk(5, () -> rotationMotor.setPosition(0.0, 0.25));

        rotationPosition = rotationMotor.getPosition();
        rotationVelocity = rotationMotor.getVelocity();
        rotationAppliedVolts = rotationMotor.getMotorVoltage();
        rotationCurrentAmps = rotationMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            RobotConstants.robotLoopFrequencyHz,
            rotationPosition, 
            rotationVelocity, 
            rotationAppliedVolts, 
            rotationCurrentAmps);
        ParentDevice.optimizeBusUtilizationForAll(rotationMotor);
    }

    @Override
    public void updateInputs(ShooterIOInputs shooterInputs, TurretIOInputs turretInputs) {
        super.updateInputs(shooterInputs);
        turretInputs.rotationMotorConnected = BaseStatusSignal.refreshAll(
            rotationPosition, rotationVelocity, rotationAppliedVolts, rotationCurrentAmps).equals(StatusCode.OK);
        turretInputs.rotationMotorPosition = Rotation2d.fromRotations(rotationPosition.getValueAsDouble());
        turretInputs.rotationMotorVelocityRadPerSec = Units.rotationsToRadians(rotationVelocity.getValueAsDouble());
        turretInputs.rotationMotorAppliedVolts = rotationAppliedVolts.getValueAsDouble();
        turretInputs.rotationMotorCurrentAmps = rotationCurrentAmps.getValueAsDouble();
    }

    @Override
    public void setRotationMotorPosition(Rotation2d rotation) {
        rotationMotor.setControl(positionVoltageRequest.withPosition(rotation.getRotations()));
    }

    @Override
    public void setRotationMotorOpenLoop(double speed) {
        rotationMotor.set(speed);
    }
}