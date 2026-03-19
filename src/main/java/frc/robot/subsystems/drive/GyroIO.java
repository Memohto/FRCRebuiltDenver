// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = Rotation2d.kZero;
    public double yawVelocityRadPerSec = 0.0;
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
  }

  class GyroIOInputsAutoLogged extends GyroIOInputs implements LoggableInputs {
    public void toLog(LogTable table) {
        table.put("connected", connected);
        table.put("yawPosition", yawPosition);
        table.put("yawVelocityRadPerSec", yawVelocityRadPerSec);
        table.put("odometryYawTimestamps", odometryYawTimestamps);
        table.put("odometryYawPositions", odometryYawPositions);
    }

    public void fromLog(LogTable table) {
        connected = table.get("connected", connected);
        yawPosition = table.get("yawPosition", yawPosition);
        yawVelocityRadPerSec = table.get("yawVelocityRadPerSec", yawVelocityRadPerSec);
        odometryYawTimestamps = table.get("odometryYawTimestamps", odometryYawTimestamps);
        odometryYawPositions = table.get("odometryYawPositions", odometryYawPositions);
    }
}

  public default void updateInputs(GyroIOInputs inputs) {}
}
