package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ShooterIO {
    public static class ShooterIOInputs {
        public boolean flywheelConnected = false;
        public double flywheelVelocityRadPerSec = 0.0;
        public double flywheelAppliedVolts = 0.0;
        public double flywheelCurrentAmps = 0.0;
        public double flywheelTargetVelocityRadPerSec = 0.0; // NEW: for logging target vs actual

        public boolean hoodConnected = false;
        public Rotation2d hoodPosition = new Rotation2d();
        public double hoodVelocityRadPerSec = 0.0;
        public double hoodAppliedVolts = 0.0;
        public double hoodCurrentAmps = 0.0;
    }

    public static class ShooterIOInputsAutoLogged extends ShooterIOInputs implements LoggableInputs {
        @Override
        public void toLog(LogTable table) {
            table.put("flywheelConnected", flywheelConnected);
            table.put("flywheelVelocityRadPerSec", flywheelVelocityRadPerSec);
            table.put("flywheelAppliedVolts", flywheelAppliedVolts);
            table.put("flywheelCurrentAmps", flywheelCurrentAmps);
            table.put("flywheelTargetVelocityRadPerSec", flywheelTargetVelocityRadPerSec); // NEW

            table.put("hoodConnected", hoodConnected);
            table.put("hoodPosition", hoodPosition);
            table.put("hoodVelocityRadPerSec", hoodVelocityRadPerSec);
            table.put("hoodAppliedVolts", hoodAppliedVolts);
            table.put("hoodCurrentAmps", hoodCurrentAmps);
        }

        @Override
        public void fromLog(LogTable table) {
            flywheelConnected = table.get("flywheelConnected", flywheelConnected);
            flywheelVelocityRadPerSec = table.get("flywheelVelocityRadPerSec", flywheelVelocityRadPerSec);
            flywheelAppliedVolts = table.get("flywheelAppliedVolts", flywheelAppliedVolts);
            flywheelCurrentAmps = table.get("flywheelCurrentAmps", flywheelCurrentAmps);
            flywheelTargetVelocityRadPerSec = table.get("flywheelTargetVelocityRadPerSec", flywheelTargetVelocityRadPerSec); // NEW

            hoodConnected = table.get("hoodConnected", hoodConnected);
            hoodPosition = table.get("hoodPosition", hoodPosition);
            hoodVelocityRadPerSec = table.get("hoodVelocityRadPerSec", hoodVelocityRadPerSec);
            hoodAppliedVolts = table.get("hoodAppliedVolts", hoodAppliedVolts);
            hoodCurrentAmps = table.get("hoodCurrentAmps", hoodCurrentAmps);
        }
    }

    public void updateInputs(ShooterIOInputs inputs);

    /** Open-loop: sets raw duty cycle (-1.0 to 1.0). Use only for manual/override control. */
    public void setFlywheelOpenLoop(double speed);

    /**
     * Closed-loop velocity control.
     * The TalonFX will continuously adjust voltage to maintain the target speed
     * regardless of battery state. Uses Slot0 gains from ShooterConstants.flywheelGains.
     *
     * @param velocityRadPerSec Target flywheel speed in radians per second.
     *                          Convert from RPS: velocityRPS * 2 * Math.PI
     */
    public void setFlywheelVelocity(double velocityRadPerSec);

    public void setHoodOpenLoop(double speed);
    public void setHoodPosition(Rotation2d rotation);
}
