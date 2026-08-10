package frc.robot.subsystems.intake;

import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
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

    public boolean extensorIsAtPosition(Rotation2d target, Rotation2d tolerance) {
        return inputs.extensorPosition.getRadians() > target.getRadians() - tolerance.getRadians() &&
               inputs.extensorPosition.getRadians() < target.getRadians() + tolerance.getRadians();
    }

    public void setExtended() {
        io.setExtensorPosition(IntakeConstants.extendedRotation);//constant
    }

    public void setExtendedReset() {
        io.setExtensorPosition(IntakeConstants.extendedRotationReversed);//constant

    }

    public void extend() {
        io.setExtensorOpenLoop(IntakeConstants.extensorSpeed);
    }

    /** Empuje hacia afuera a velocidad arbitraria. Lo usa el homing por corriente. */
    public void extendAtSpeed(double speed) {
        io.setExtensorOpenLoop(Math.abs(speed));
    }

    public void retract() {
        io.setExtensorOpenLoop(-IntakeConstants.extensorSpeed);
    }

    public void stopExtensor() {
        io.setExtensorOpenLoop(0.0);
    }

    public void setSoftwareLimit(boolean value){
        io.setSoftwareLimit(value);
    }

    // ══════════════════════════════════════════════════════════════════════════
    // Helpers para el Demo Mode
    // ══════════════════════════════════════════════════════════════════════════

    /**
     * Comanda el extensor a una posición arbitraria en radianes.
     *
     * <p>
     * A diferencia de {@code setExtended()} / {@code setExtendedReset()}, que van
     * a dos posiciones fijas, esto permite trayectorias continuas. Lo usa la
     * secuencia de agitación para oscilar la caja.
     */
    public void setExtensorPositionRad(double positionRad) {
        io.setExtensorPosition(Rotation2d.fromRadians(positionRad));
    }

    /** Velocidad del rodillo en lazo abierto. Positivo = hacia adentro. */
    public void setRollersOpenLoop(double speed) {
        io.setRollersOpenLoop(speed);
    }

    /** Posición del extensor en radianes crudos, sin envolver. */
    public double getExtensorPositionRad() {
        return inputs.extensorPositionRad;
    }

    /** Corriente de estator del extensor. Es lo que detecta el tope mecánico. */
    public double getExtensorCurrentAmps() {
        return inputs.extensorCurrentAmps;
    }

    /** Congela el extensor donde está ahora mismo. */
    public void holdExtensorHere() {
        setExtensorPositionRad(getExtensorPositionRad());
    }
}