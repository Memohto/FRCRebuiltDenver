package frc.robot.subsystems.indexer;

import static frc.robot.util.PhoenixUtil.*;

import frc.robot.constants.IndexerConstants;
import frc.robot.constants.RobotConstants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IndexerIOTalonFX implements IndexerIO {
    // Hardware objects
    private final TalonFX rollers;
    private final TalonFX shooterWheels;
    private final TalonFX turretWheels;
    private final TalonFX feeder;
    
    // Inputs from rollers motor
    private final StatusSignal<AngularVelocity> rollersVelocity;
    private final StatusSignal<Voltage> rollersAppliedVolts;
    private final StatusSignal<Current> rollersCurrent;

    // Inputs from left wheels motor
    private final StatusSignal<AngularVelocity> shooterWheelsVelocity;
    private final StatusSignal<Voltage> shooterWheelsAppliedVolts;
    private final StatusSignal<Current> shooterWheelsCurrent;

    // Inputs from right wheels motor
    private final StatusSignal<AngularVelocity> turretWheelsVelocity;
    private final StatusSignal<Voltage> turretWheelsAppliedVolts;
    private final StatusSignal<Current> turretWheelsCurrent;

    // Inputs from feeder motor
    private final StatusSignal<AngularVelocity> feederVelocity;
    private final StatusSignal<Voltage> feederAppliedVolts;
    private final StatusSignal<Current> feederCurrent;

    // Connection debouncers
    private final Debouncer rollersConnectedDebounce =
        new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer shooterWheelsConnectedDebounce =
        new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer turretWheelsConnectedDebounce =
        new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer feederConnectedDebounce =
        new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    public IndexerIOTalonFX(int rollersCanId, int shooterWheelsCanId, int turretWheelsCanId, int feederCanId) {
        rollers = new TalonFX(rollersCanId);
        shooterWheels = new TalonFX(shooterWheelsCanId);
        turretWheels = new TalonFX(turretWheelsCanId);
        feeder = new TalonFX(feederCanId);

        // Configure rollers motor
        var rollersConfig = new TalonFXConfiguration();
        rollersConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollersConfig.Slot0 = IndexerConstants.rollersGains;
        rollersConfig.Feedback.SensorToMechanismRatio = IndexerConstants.rollersGearRatio;
        rollersConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollersConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollersConfig.CurrentLimits.StatorCurrentLimit = IndexerConstants.rollersStatorCurrentLimitAmps;
        rollersConfig.CurrentLimits.SupplyCurrentLimit = IndexerConstants.rollersSupplyCurrentLimitAmps;
        rollersConfig.MotorOutput.Inverted =
            IndexerConstants.rollersInverted
                ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        tryUntilOk(5, () -> rollers.getConfigurator().apply(rollersConfig, 0.25));
        tryUntilOk(5, () -> rollers.setPosition(0.0, 0.25));

        // Configure left wheels motor
        var shooterWheelsConfig = new TalonFXConfiguration();
        shooterWheelsConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterWheelsConfig.Slot0 = IndexerConstants.shooterWheelsGains;
        shooterWheelsConfig.Feedback.SensorToMechanismRatio = IndexerConstants.shooterWheelsGearRatio;
        shooterWheelsConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterWheelsConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterWheelsConfig.CurrentLimits.StatorCurrentLimit = IndexerConstants.shooterWheelsStatorCurrentLimitAmps;
        shooterWheelsConfig.CurrentLimits.SupplyCurrentLimit = IndexerConstants.shooterWheelsSupplyCurrentLimitAmps;
        shooterWheelsConfig.MotorOutput.Inverted =
            IndexerConstants.shooterWheelsInverted
                ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        tryUntilOk(5, () -> shooterWheels.getConfigurator().apply(shooterWheelsConfig, 0.25));
        tryUntilOk(5, () -> shooterWheels.setPosition(0.0, 0.25));

        // Configure right wheels motor
        var turretWheelsConfig = new TalonFXConfiguration();
        turretWheelsConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        turretWheelsConfig.Slot0 = IndexerConstants.turretWheelsGains;
        turretWheelsConfig.Feedback.SensorToMechanismRatio = IndexerConstants.turretWheelsGearRatio;
        turretWheelsConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turretWheelsConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turretWheelsConfig.CurrentLimits.StatorCurrentLimit = IndexerConstants.turretWheelsStatorCurrentLimitAmps;
        turretWheelsConfig.CurrentLimits.SupplyCurrentLimit = IndexerConstants.turretWheelsSupplyCurrentLimitAmps;
        turretWheelsConfig.MotorOutput.Inverted =
            IndexerConstants.turretWheelsInverted
                ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        tryUntilOk(5, () -> turretWheels.getConfigurator().apply(turretWheelsConfig, 0.25));
        tryUntilOk(5, () -> turretWheels.setPosition(0.0, 0.25));

        // Configure rollers motor
        var feederConfig = new TalonFXConfiguration();
        feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        feederConfig.Slot0 = IndexerConstants.feederGains;
        feederConfig.Feedback.SensorToMechanismRatio = IndexerConstants.feederGearRatio;
        feederConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        feederConfig.CurrentLimits.StatorCurrentLimit = IndexerConstants.feederStatorCurrentLimitAmps;
        feederConfig.CurrentLimits.SupplyCurrentLimit = IndexerConstants.feederSupplyCurrentLimitAmps;
        feederConfig.MotorOutput.Inverted =
            IndexerConstants.feederInverted
                ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        tryUntilOk(5, () -> feeder.getConfigurator().apply(feederConfig, 0.25));
        tryUntilOk(5, () -> feeder.setPosition(0.0, 0.25));

        // Create rollers status signals
        rollersVelocity = rollers.getVelocity();
        rollersAppliedVolts = rollers.getMotorVoltage();
        rollersCurrent = rollers.getStatorCurrent();

        // Create shooterWheels status signals
        shooterWheelsVelocity = shooterWheels.getVelocity();
        shooterWheelsAppliedVolts = shooterWheels.getMotorVoltage();
        shooterWheelsCurrent = shooterWheels.getStatorCurrent();

        // Create turretWheels status signals
        turretWheelsVelocity = turretWheels.getVelocity();
        turretWheelsAppliedVolts = turretWheels.getMotorVoltage();
        turretWheelsCurrent = turretWheels.getStatorCurrent();

        // Create feeder status signals
        feederVelocity = feeder.getVelocity();
        feederAppliedVolts = feeder.getMotorVoltage();
        feederCurrent = feeder.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            RobotConstants.lowPriorityFrequencyHz,
            rollersVelocity,
            rollersAppliedVolts,
            rollersCurrent,
            shooterWheelsVelocity,
            shooterWheelsAppliedVolts,
            shooterWheelsCurrent,
            turretWheelsVelocity,
            turretWheelsAppliedVolts,
            turretWheelsCurrent,
            feederVelocity,
            feederAppliedVolts,
            feederCurrent);
        ParentDevice.optimizeBusUtilizationForAll(rollers, shooterWheels, turretWheels, feeder);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        // Refresh all signals
        var rollersStatus =
            BaseStatusSignal.refreshAll(rollersVelocity, rollersAppliedVolts, rollersCurrent);
        var shooterWheelsStatus =
            BaseStatusSignal.refreshAll(shooterWheelsVelocity, shooterWheelsAppliedVolts, shooterWheelsCurrent);
        var turretWheelsStatus =
            BaseStatusSignal.refreshAll(turretWheelsVelocity, turretWheelsAppliedVolts, turretWheelsCurrent);
        var feederStatus =
            BaseStatusSignal.refreshAll(feederVelocity, feederAppliedVolts, feederCurrent);

        // Update rpllers inputs
        inputs.rollersConnected = rollersConnectedDebounce.calculate(rollersStatus.isOK());
        inputs.rollersVelocityRadPerSec = Units.rotationsToRadians(rollersVelocity.getValueAsDouble());
        inputs.rollersAppliedVolts = rollersAppliedVolts.getValueAsDouble();
        inputs.rollersCurrentAmps = rollersCurrent.getValueAsDouble();

        // Update left wheels inputs
        inputs.shooterWheelsConnected = shooterWheelsConnectedDebounce.calculate(shooterWheelsStatus.isOK());
        inputs.shooterWheelsVelocityRadPerSec = Units.rotationsToRadians(shooterWheelsVelocity.getValueAsDouble());
        inputs.shooterWheelsAppliedVolts = shooterWheelsAppliedVolts.getValueAsDouble();
        inputs.shooterWheelsCurrentAmps = shooterWheelsCurrent.getValueAsDouble();

        // Update right wheels inputs
        inputs.turretWheelsConnected = turretWheelsConnectedDebounce.calculate(turretWheelsStatus.isOK());
        inputs.turretWheelsVelocityRadPerSec = Units.rotationsToRadians(turretWheelsVelocity.getValueAsDouble());
        inputs.turretWheelsAppliedVolts = turretWheelsAppliedVolts.getValueAsDouble();
        inputs.turretWheelsCurrentAmps = turretWheelsCurrent.getValueAsDouble();

        // Update right wheels inputs
        inputs.feederConnected = feederConnectedDebounce.calculate(feederStatus.isOK());
        inputs.feederVelocityRadPerSec = Units.rotationsToRadians(feederVelocity.getValueAsDouble());
        inputs.feederAppliedVolts = feederAppliedVolts.getValueAsDouble();
        inputs.feederCurrentAmps = feederCurrent.getValueAsDouble();
    }

    @Override
    public void setRollersOpenLoop(double speed) {
        rollers.set(speed);
    }

    @Override
    public void setShooterWheelsOpenLoop(double speed) {
        shooterWheels.set(speed);
    }

    @Override
    public void setTurretWheelsOpenLoop(double speed) {
        turretWheels.set(speed);
    }

    @Override
    public void setFeederOpenLoop(double speed) {
        feeder.set(speed);
    }
}
