package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class IntakeVisualizer {
    private final double xModifier = Math.cos(Degrees.of(14).in(Radians));
    private final double zModifier = Math.sin(Degrees.of(14).in(Radians));

    public void updateVisualization(double positionMeters) {
        Pose3d intakePose = new Pose3d(positionMeters * xModifier, 0, positionMeters * zModifier, Rotation3d.kZero);
        Logger.recordOutput("Mechanism/Intake", intakePose);
    }
}