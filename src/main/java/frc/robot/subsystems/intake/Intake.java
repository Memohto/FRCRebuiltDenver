package frc.robot.subsystems.intake;

import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private final IntakeVisualizer visualizer = new IntakeVisualizer();

    public Intake(IntakeIO io) {
        this.io = io;
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        visualizer.updateVisualization(inputs.extensorPosition.getRadians() * IntakeConstants.pinionRadiusMeters);
    }

    public void intake() {
        io.setRollersOpenLoop(IntakeConstants.rollersSpeed);
    }

    public void outtake() {
        io.setRollersOpenLoop(-IntakeConstants.rollersSpeed);
    }

    public void stopRollers() {
        io.setRollersOpenLoop(0.0);
    }

    public void setExtensorPosition(Rotation2d position) {
        io.setExtensorPosition(position);
    }

    public boolean extensorIsAtPosition(Rotation2d target, Rotation2d tolerance) {
        return inputs.extensorPosition.getRadians() > target.getRadians() - tolerance.getRadians() &&
               inputs.extensorPosition.getRadians() < target.getRadians() + tolerance.getRadians();
    }

    public void extend() {
        io.setExtensorOpenLoop(IntakeConstants.extensorSpeed);
    }

    public void retract() {
        io.setExtensorOpenLoop(-IntakeConstants.extensorSpeed);
    }

    public void stopExtensor() {
        io.setExtensorOpenLoop(0.0);
    }

}
