package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.*;

import frc.robot.constants.IntakeConstants;
import frc.robot.constants.RobotConstants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOTalonFX implements IntakeIO {
    // Hardware objects
    private final TalonFX rollers;
    private final TalonFX extensor;

    // Control requests
    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);

    // Inputs from rollers motor
    private final StatusSignal<AngularVelocity> rollersVelocity;
    private final StatusSignal<Voltage> rollersAppliedVolts;
    private final StatusSignal<Current> rollersCurrent;

    // Inputs from extensor motor
    private final StatusSignal<Angle> extensorPosition;
    private final StatusSignal<AngularVelocity> extensorVelocity;
    private final StatusSignal<Voltage> extensorAppliedVolts;
    private final StatusSignal<Current> extensorCurrent;

    // Connection debouncers
    private final Debouncer rollersConnectedDebounce =
        new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer extensorConnectedDebounce =
        new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    public IntakeIOTalonFX(int rollersCanId, int extensorCanId) {
        this.rollers = new TalonFX(rollersCanId);
        this.extensor = new TalonFX(extensorCanId);

        // Configure rollers motor
        var rollersConfig = new TalonFXConfiguration();
        rollersConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollersConfig.Slot0 = IntakeConstants.rollersGains;
        rollersConfig.Feedback.SensorToMechanismRatio = IntakeConstants.rollersGearRatio;
        rollersConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollersConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollersConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.rollersStatorCurrentLimitAmps;
        rollersConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.rollersSupplyCurrentLimitAmps;
        rollersConfig.MotorOutput.Inverted =  IntakeConstants.rollersInverted
            ? InvertedValue.CounterClockwise_Positive 
            : InvertedValue.Clockwise_Positive;
        tryUntilOk(5, () -> rollers.getConfigurator().apply(rollersConfig, 0.25));
        tryUntilOk(5, () -> rollers.setPosition(0.0, 0.25));

        // Configure extensor motor
        var extensorConfig = new TalonFXConfiguration();
        extensorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        extensorConfig.Slot0 = IntakeConstants.extensorGains;
        extensorConfig.Feedback.SensorToMechanismRatio = IntakeConstants.extensorGearRatio;
        extensorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        extensorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        extensorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Rotation2d.fromRadians(10).getRotations(); // 20.5
        extensorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Rotation2d.fromRadians(3).getRotations(); // 3 
        rollersConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollersConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollersConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.extensorStatorCurrentLimitAmps;
        rollersConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.extensorSupplyCurrentLimitAmps;
        extensorConfig.MotorOutput.Inverted = IntakeConstants.extensorInverted 
            ? InvertedValue.CounterClockwise_Positive 
            : InvertedValue.Clockwise_Positive;
        tryUntilOk(5, () -> extensor.getConfigurator().apply(extensorConfig, 0.25));
        tryUntilOk(5, () -> extensor.setPosition(0.0, 0.25));

        // Create rollers status signals
        rollersVelocity = rollers.getVelocity();
        rollersAppliedVolts = rollers.getMotorVoltage();
        rollersCurrent = rollers.getStatorCurrent();
        
        // Create turn status signals
        extensorPosition = extensor.getPosition();
        extensorVelocity = extensor.getVelocity();
        extensorAppliedVolts = extensor.getMotorVoltage();
        extensorCurrent = extensor.getStatorCurrent();
        
        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(
            RobotConstants.highPriorityFrequencyHz, 
            extensorPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
            RobotConstants.lowPriorityFrequencyHz,
            rollersVelocity,
            rollersAppliedVolts,
            rollersCurrent,
            extensorVelocity,
            extensorAppliedVolts,
            extensorCurrent);
        ParentDevice.optimizeBusUtilizationForAll(rollers, extensor);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // Refresh all signals
        var rollersStatus =
            BaseStatusSignal.refreshAll(rollersVelocity, rollersAppliedVolts, rollersCurrent);
        var extensorStatus =
            BaseStatusSignal.refreshAll(extensorPosition, extensorVelocity, extensorAppliedVolts, extensorCurrent);

        // Update rollers inputs
        inputs.rollersConnected = rollersConnectedDebounce.calculate(rollersStatus.isOK());
        inputs.rollersVelocityRadPerSec = Units.rotationsToRadians(rollersVelocity.getValueAsDouble());
        inputs.rollersAppliedVolts = rollersAppliedVolts.getValueAsDouble();
        inputs.rollersCurrentAmps = rollersCurrent.getValueAsDouble();

        // Update extensor inputs
        inputs.extensorConnected = extensorConnectedDebounce.calculate(extensorStatus.isOK());
        inputs.extensorPosition = Rotation2d.fromRotations(extensorPosition.getValueAsDouble());
        inputs.extensorVelocityRadPerSec = Units.rotationsToRadians(extensorVelocity.getValueAsDouble());
        inputs.extensorAppliedVolts = extensorAppliedVolts.getValueAsDouble();
        inputs.extensorCurrentAmps = extensorCurrent.getValueAsDouble();
    }

    @Override
    public void setRollersOpenLoop(double speed) {
        rollers.set(speed);
    }

    @Override
    public void setExtensorOpenLoop(double speed) {
        extensor.set(speed);
    }

    @Override
    public void setExtensorPosition(Rotation2d rotation) {
        extensor.setControl(positionVoltageRequest.withPosition(rotation.getRotations()));
    }
}
