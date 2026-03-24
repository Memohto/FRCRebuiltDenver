package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputsAutoLogged;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    public final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public void setFlywheelSpeed(double speed) {
        io.setFlywheelOpenLoop(speed);
    }

    public void startFlywheelForDistance(double distanceMeters) {
        double speed = ShooterConstants.kShooterFlywheelMap.get(distanceMeters);
        Logger.recordOutput("Debug/Shooter/Speed", speed);
        setFlywheelSpeed(speed);
    }

    public void stopFlywheel() {
        io.setFlywheelOpenLoop(0.0);
    }

    public void setHoodOpenLoop(double speed) {
        io.setHoodOpenLoop(speed);
    }

    public void setHoodPosition(Rotation2d rotation) {
        io.setHoodPosition(rotation);
    }

    public void setHoodForDistance(double distanceMeters) {
        double hoodOffsetDeg = ShooterConstants.kShooterHoodMap.get(distanceMeters);
        setHoodPosition(Rotation2d.fromDegrees(hoodOffsetDeg));
    }

    public void setHoodAtInitialPosition() {
        io.setHoodPosition(new Rotation2d());
    }
}