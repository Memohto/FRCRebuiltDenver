package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakeIO {
    public static class IntakeIOInputs {
        public boolean rollersConnected = false;
        public double rollersVelocityRadPerSec = 0.0;
        public double rollersAppliedVolts = 0.0;
        public double rollersCurrentAmps = 0.0;

        public boolean extensorConnected = false;
        public Rotation2d extensorPosition = new Rotation2d();
        public double extensorVelocityRadPerSec = 0.0;
        public double extensorAppliedVolts = 0.0;
        public double extensorCurrentAmps = 0.0;
    }

    public static class IntakeIOInputsAutoLogged extends IntakeIOInputs implements org.littletonrobotics.junction.inputs.LoggableInputs {
        @Override
        public void toLog(LogTable table) {
            table.put("rollersConnected", rollersConnected);
            table.put("rollersVelocityRadPerSec", rollersVelocityRadPerSec);
            table.put("rollersAppliedVolts", rollersAppliedVolts);
            table.put("rollersCurrentAmps", rollersCurrentAmps);

            table.put("extensorConnected", extensorConnected);
            table.put("extensorPosition", extensorPosition);
            table.put("extensorVelocityRadPerSec", extensorVelocityRadPerSec);
            table.put("extensorAppliedVolts", extensorAppliedVolts);
            table.put("extensorCurrentAmps", extensorCurrentAmps);
        }

        @Override
        public void fromLog(LogTable table) {
            rollersConnected = table.get("rollersConnected", rollersConnected);
            rollersVelocityRadPerSec = table.get("rollersVelocityRadPerSec", rollersVelocityRadPerSec);
            rollersAppliedVolts = table.get("rollersAppliedVolts", rollersAppliedVolts);
            rollersCurrentAmps = table.get("rollersCurrentAmps", rollersCurrentAmps);

            extensorConnected = table.get("extensorConnected", extensorConnected);
            extensorPosition = table.get("extensorPosition", extensorPosition);
            extensorVelocityRadPerSec = table.get("extensorVelocityRadPerSec", extensorVelocityRadPerSec);
            extensorAppliedVolts = table.get("extensorAppliedVolts", extensorAppliedVolts);
            extensorCurrentAmps = table.get("extensorCurrentAmps", extensorCurrentAmps);
        }
    }

    public void updateInputs(IntakeIOInputs inputs);
    public void setRollersOpenLoop(double speed);
    public void setExtensorOpenLoop(double speed);
    public void setExtensorPosition(Rotation2d rotation);
}
