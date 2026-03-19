package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IndexerIO {
    public static class IndexerIOInputs {
        public boolean rollersConnected = false;
        public double rollersVelocityRadPerSec = 0.0;
        public double rollersAppliedVolts = 0.0;
        public double rollersCurrentAmps = 0.0;

        public boolean shooterWheelsConnected = false;
        public double shooterWheelsVelocityRadPerSec = 0.0;
        public double shooterWheelsAppliedVolts = 0.0;
        public double shooterWheelsCurrentAmps = 0.0;

        public boolean turretWheelsConnected = false;
        public double turretWheelsVelocityRadPerSec = 0.0;
        public double turretWheelsAppliedVolts = 0.0;
        public double turretWheelsCurrentAmps = 0.0;
        
        public boolean feederConnected = false;
        public double feederVelocityRadPerSec = 0.0;
        public double feederAppliedVolts = 0.0;
        public double feederCurrentAmps = 0.0;
    }

    public static class IndexerIOInputsAutoLogged extends IndexerIOInputs implements LoggableInputs {
        @Override
        public void toLog(LogTable table) {
            table.put("rollersConnected", rollersConnected);
            table.put("rollersVelocityRadPerSec", rollersVelocityRadPerSec);
            table.put("rollersAppliedVolts", rollersAppliedVolts);
            table.put("rollersCurrentAmps", rollersCurrentAmps);

            table.put("shooterWheelsConnected", shooterWheelsConnected);
            table.put("shooterWheelsVelocityRadPerSec", shooterWheelsVelocityRadPerSec);
            table.put("shooterWheelsAppliedVolts", shooterWheelsAppliedVolts);
            table.put("shooterWheelsCurrentAmps", shooterWheelsCurrentAmps);

            table.put("turretWheelsConnected", turretWheelsConnected);
            table.put("turretWheelsVelocityRadPerSec", turretWheelsVelocityRadPerSec);
            table.put("turretWheelsAppliedVolts", turretWheelsAppliedVolts);
            table.put("turretWheelsCurrentAmps", turretWheelsCurrentAmps);

            table.put("feederConnected", feederConnected);
            table.put("feederVelocityRadPerSec", feederVelocityRadPerSec);
            table.put("feederAppliedVolts", feederAppliedVolts);
            table.put("feederCurrentAmps", feederCurrentAmps);
        }

        @Override
        public void fromLog(LogTable table) {
            rollersConnected = table.get("rollersConnected", rollersConnected);
            rollersVelocityRadPerSec = table.get("rollersVelocityRadPerSec", rollersVelocityRadPerSec);
            rollersAppliedVolts = table.get("rollersAppliedVolts", rollersAppliedVolts);
            rollersCurrentAmps = table.get("rollersCurrentAmps", rollersCurrentAmps);

            shooterWheelsConnected = table.get("shooterWheelsConnected", shooterWheelsConnected);
            shooterWheelsVelocityRadPerSec = table.get("shooterWheelsVelocityRadPerSec", shooterWheelsVelocityRadPerSec);
            shooterWheelsAppliedVolts = table.get("shooterWheelsAppliedVolts", shooterWheelsAppliedVolts);
            shooterWheelsCurrentAmps = table.get("shooterWheelsCurrentAmps", shooterWheelsCurrentAmps);

            turretWheelsConnected = table.get("turretWheelsConnected", turretWheelsConnected);
            turretWheelsVelocityRadPerSec = table.get("turretWheelsVelocityRadPerSec", turretWheelsVelocityRadPerSec);
            turretWheelsAppliedVolts = table.get("turretWheelsAppliedVolts", turretWheelsAppliedVolts);
            turretWheelsCurrentAmps = table.get("turretWheelsCurrentAmps", turretWheelsCurrentAmps);

            feederConnected = table.get("feederConnected", feederConnected);
            feederVelocityRadPerSec = table.get("feederVelocityRadPerSec", feederVelocityRadPerSec);
            feederAppliedVolts = table.get("feederAppliedVolts", feederAppliedVolts);
            feederCurrentAmps = table.get("feederCurrentAmps", feederCurrentAmps);
        }
    }
    
    public void updateInputs(IndexerIOInputs inputs);
    public void setRollersOpenLoop(double speed);
    public void setShooterWheelsOpenLoop(double speed);
    public void setTurretWheelsOpenLoop(double speed);
    public void setFeederOpenLoop(double speed);
}