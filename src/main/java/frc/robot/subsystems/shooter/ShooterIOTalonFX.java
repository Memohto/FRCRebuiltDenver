package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.*;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.RobotConstants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
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

public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX flywheel;
    private final TalonFX hood;

    private final StatusSignal<AngularVelocity> flywheelVelocity;
    private final StatusSignal<Voltage> flywheelAppliedVolts;
    private final StatusSignal<Current> flywheelCurrentAmps;

    private final StatusSignal<Angle> hoodPosition;
    private final StatusSignal<AngularVelocity> hoodVelocity;
    private final StatusSignal<Voltage> hoodAppliedVolts;
    private final StatusSignal<Current> hoodCurrentAmps;

    private final Debouncer flywheelConnectedDebounce =
        new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer hoodConnectedDebounce =
        new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    // Closed-loop velocity request — this is the key change.
    // VelocityVoltage tells the TalonFX to maintain a target RPS using
    // its onboard PID + feedforward loop, compensating for voltage sag automatically.
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0)
        .withSlot(0)         // Use Slot0 gains from ShooterConstants.flywheelGains
        .withEnableFOC(true); // Field-Oriented Control for smoother torque on Kraken

    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);

    // Track the last commanded velocity so we can log it
    private double lastTargetVelocityRadPerSec = 0.0;

    public ShooterIOTalonFX(int flywheelCanId, int hoodCanId) {
        flywheel = new TalonFX(flywheelCanId);
        hood = new TalonFX(hoodCanId);

        // Configure flywheel motor
        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelConfig.Slot0 = ShooterConstants.flywheelGains;
        // NOTE: Slot0 should have kV tuned so the flywheel reaches target speed accurately.
        // Recommended starting values in ShooterConstants:
        //   kV = 0.12  (volts per RPS — empirically tune this first)
        //   kP = 0.10  (small correction for steady-state error)
        //   kI = 0.00
        //   kD = 0.00
        flywheelConfig.Feedback.SensorToMechanismRatio = ShooterConstants.flywheelGearRatio;
        flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        flywheelConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.flywheelStatorCurrentLimitAmps;
        flywheelConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.flywheelSupplyCurrentLimitAmps;
        flywheelConfig.MotorOutput.Inverted = ShooterConstants.flywheelInverted
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;
        tryUntilOk(5, () -> flywheel.getConfigurator().apply(flywheelConfig, 0.25));
        tryUntilOk(5, () -> flywheel.setPosition(0.0, 0.25));

        // Configure hood motor (unchanged)
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodConfig.Slot0 = ShooterConstants.hoodGains;
        hoodConfig.Feedback.SensorToMechanismRatio = ShooterConstants.hoodGearRatio;
        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.hoodStatorCurrentLimitAmps;
        hoodConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.hoodSupplyCurrentLimitAmps;
        hoodConfig.MotorOutput.Inverted = ShooterConstants.hoodInverted
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;
        tryUntilOk(5, () -> hood.getConfigurator().apply(hoodConfig, 0.25));
        tryUntilOk(5, () -> hood.setPosition(0.0, 0.25));

        // Status signals
        flywheelVelocity = flywheel.getVelocity();
        flywheelAppliedVolts = flywheel.getMotorVoltage();
        flywheelCurrentAmps = flywheel.getStatorCurrent();

        hoodPosition = hood.getPosition();
        hoodVelocity = hood.getVelocity();
        hoodAppliedVolts = hood.getMotorVoltage();
        hoodCurrentAmps = hood.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            RobotConstants.highPriorityFrequencyHz,
            flywheelVelocity,  // Promote flywheel velocity to high priority for tight speed tracking
            hoodPosition
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
            RobotConstants.lowPriorityFrequencyHz,
            flywheelAppliedVolts,
            flywheelCurrentAmps,
            hoodVelocity,
            hoodAppliedVolts,
            hoodCurrentAmps);
        ParentDevice.optimizeBusUtilizationForAll(flywheel, hood);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        var flyWheelStatus =    
            BaseStatusSignal.refreshAll(flywheelVelocity, flywheelAppliedVolts, flywheelCurrentAmps);
        var hoodStatus = 
            BaseStatusSignal.refreshAll(hoodPosition, hoodVelocity, hoodAppliedVolts, hoodCurrentAmps);

        inputs.flywheelConnected = flywheelConnectedDebounce.calculate(flyWheelStatus.equals(StatusCode.OK));
        inputs.flywheelVelocityRadPerSec = Units.rotationsToRadians(flywheelVelocity.getValueAsDouble());
        inputs.flywheelAppliedVolts = flywheelAppliedVolts.getValueAsDouble();
        inputs.flywheelCurrentAmps = flywheelCurrentAmps.getValueAsDouble();
        inputs.flywheelTargetVelocityRadPerSec = lastTargetVelocityRadPerSec; // Log target for tuning

        inputs.hoodConnected = hoodConnectedDebounce.calculate(hoodStatus.equals(StatusCode.OK));
        inputs.hoodPosition = Rotation2d.fromRotations(hoodPosition.getValueAsDouble());
        inputs.hoodVelocityRadPerSec = Units.rotationsToRadians(hoodVelocity.getValueAsDouble());
        inputs.hoodAppliedVolts = hoodAppliedVolts.getValueAsDouble();
        inputs.hoodCurrentAmps = hoodCurrentAmps.getValueAsDouble();
    }

    /** Open-loop fallback: raw duty cycle, NOT battery-compensated. Avoid during matches. */
    @Override
    public void setFlywheelOpenLoop(double speed) {
        lastTargetVelocityRadPerSec = 0.0;
        flywheel.set(speed);
    }

    /**
     * Closed-loop velocity control via VelocityVoltage.
     * The TalonFX PID + kV feedforward will continuously adjust output voltage
     * to maintain the target speed even as battery voltage drops during a match.
     *
     * @param velocityRadPerSec Target in rad/s.
     *                          ShooterConstants.kShooterFlywheelMap should store RPS values;
     *                          multiply by 2π before calling here, or store rad/s directly.
     */
    @Override
    public void setFlywheelVelocity(double velocityRadPerSec) {
        lastTargetVelocityRadPerSec = velocityRadPerSec;
        // VelocityVoltage expects rotations per second, so convert from rad/s
        double velocityRPS = Units.radiansToRotations(velocityRadPerSec);
        flywheel.setControl(velocityVoltageRequest.withVelocity(velocityRPS));
    }

    @Override
    public void setHoodOpenLoop(double speed) {
        hood.set(speed);
    }

    @Override
    public void setHoodPosition(Rotation2d rotation) {
        hood.setControl(positionVoltageRequest.withPosition(rotation.getRotations()));
    }
}
