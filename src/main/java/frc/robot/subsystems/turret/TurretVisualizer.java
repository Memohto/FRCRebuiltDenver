package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class TurretVisualizer {
    public void updateVisualization(Rotation3d rotation) {
        Pose3d turretPose = new Pose3d(-0.165, -0.17, -0.21, rotation);
        Logger.recordOutput("Mechanism/Turret", turretPose);
    }
}