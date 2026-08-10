package frc.robot.subsystems.turret;

import static frc.robot.util.PhoenixUtil.*;

import frc.robot.constants.DemoConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0);

    public TurretIOTalonFX(int flywheelCanId, int hoodCanId, int rotationMotorCanId) {
        super(flywheelCanId, hoodCanId);
        rotationMotor = new TalonFX(rotationMotorCanId);

        // Rotation motor config
        TalonFXConfiguration rotationConfig = new TalonFXConfiguration();
        rotationConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // Ganancias: en demo se usan unas propias, sin término integral y con
        // más derivativo. El kI = 0.1 de competencia acumula error mientras el
        // perfil de Motion Magic está en camino y luego sobrepasa — es una
        // fuente clásica de cacería de baja frecuencia, y para SEGUIR un
        // objetivo suavemente no hace falta integral.
        if (RobotConstants.isDemoMode && DemoConstants.useDemoTurretGains) {
            rotationConfig.Slot0 = new Slot0Configs()
                    .withKP(DemoConstants.turretDemoKp)
                    .withKI(DemoConstants.turretDemoKi)
                    .withKD(DemoConstants.turretDemoKd)
                    .withKS(DemoConstants.turretDemoKs)
                    .withKV(DemoConstants.turretDemoKv);
        } else {
            rotationConfig.Slot0 = TurretConstants.rotationMotorGains;
        }
        rotationConfig.Feedback.SensorToMechanismRatio = TurretConstants.rotationMotorGearRatio;
        rotationConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        rotationConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        rotationConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
            Units.radiansToRotations(TurretConstants.maxRotationRad);
        rotationConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
            Units.radiansToRotations(TurretConstants.minRotationRad);
        // Perfiles más lentos en demo: para seguir un objetivo suavemente
        // conviene que el perfil no alcance a saturar entre comando y comando.
        boolean demoProfile = RobotConstants.isDemoMode && DemoConstants.useDemoTurretGains;
        rotationConfig.MotionMagic.MotionMagicCruiseVelocity = demoProfile
                ? DemoConstants.turretDemoCruiseRotPerSec
                : TurretConstants.maxVelocityRotPerSec;
        rotationConfig.MotionMagic.MotionMagicAcceleration = demoProfile
                ? DemoConstants.turretDemoAccelRotPerSecSec
                : TurretConstants.maxAccelerationRotPerSecSec;
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
        
        double positionRotations = rotationPosition.getValueAsDouble();
        
        turretInputs.rotationMotorConnected = BaseStatusSignal.refreshAll(
            rotationPosition, rotationVelocity, rotationAppliedVolts, rotationCurrentAmps).equals(StatusCode.OK);
        turretInputs.rotationMotorPositionRad = Units.rotationsToRadians(positionRotations);
        turretInputs.rotationMotorPosition = Rotation2d.fromRotations(positionRotations);
        turretInputs.rotationMotorVelocityRadPerSec = Units.rotationsToRadians(rotationVelocity.getValueAsDouble());
        turretInputs.rotationMotorAppliedVolts = rotationAppliedVolts.getValueAsDouble();
        turretInputs.rotationMotorCurrentAmps = rotationCurrentAmps.getValueAsDouble();
    }

    @Override
    public void setRotationMotorPosition(double positionRad) {
        // Convert radians → mechanism rotations directly (NO Rotation2d — avoids ±180° wrap)
        double positionRotations = positionRad / (2.0 * Math.PI);
        rotationMotor.setControl(motionMagicRequest.withPosition(positionRotations));
    }

    @Override
    public void setRotationMotorOpenLoop(double speed) {
        rotationMotor.set(speed);
    }
}