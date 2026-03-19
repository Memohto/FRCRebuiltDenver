package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

public interface TurretIO {
    public static class TurretIOInputs {
        public boolean rotationMotorConnected = false;
        public Rotation2d rotationMotorPosition = new Rotation2d();
        public double rotationMotorVelocityRadPerSec = 0.0;
        public double rotationMotorAppliedVolts = 0.0;
        public double rotationMotorCurrentAmps = 0.0;
    }

    public static class TurretIOInputsAutoLogged extends TurretIOInputs implements LoggableInputs {
        @Override
        public void toLog(LogTable table) {
            table.put("rotationMotorConnected", rotationMotorConnected);
            table.put("rotationMotorPosition", rotationMotorPosition);
            table.put("rotationMotorVelocityRadPerSec", rotationMotorVelocityRadPerSec);
            table.put("rotationMotorAppliedVolts", rotationMotorAppliedVolts);
            table.put("rotationMotorCurrentAmps", rotationMotorCurrentAmps);
        }

        @Override
        public void fromLog(LogTable table) {
            rotationMotorConnected = table.get("rotationMotorConnected", rotationMotorConnected);
            rotationMotorPosition = table.get("rotationMotorPosition", rotationMotorPosition);
            rotationMotorVelocityRadPerSec = table.get("rotationMotorVelocityRadPerSec", rotationMotorVelocityRadPerSec);
            rotationMotorAppliedVolts = table.get("rotationMotorAppliedVolts", rotationMotorAppliedVolts);
            rotationMotorCurrentAmps = table.get("rotationMotorCurrentAmps", rotationMotorCurrentAmps);
        }
    }

    public void updateInputs(ShooterIOInputs shooterInputs, TurretIOInputs turretInputs);
    public void setRotationMotorPosition(Rotation2d rotation);
    public void setRotationMotorOpenLoop(double speed);
}
