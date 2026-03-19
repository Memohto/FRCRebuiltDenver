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

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;

    public boolean turnConnected = false;
    public boolean turnEncoderConnected = false;
    public Rotation2d turnAbsolutePosition = Rotation2d.kZero;
    public Rotation2d turnPosition = Rotation2d.kZero;
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  class ModuleIOInputsAutoLogged extends ModuleIOInputs implements LoggableInputs {
    public void toLog(LogTable table) {
        table.put("driveConnected", driveConnected);
        table.put("drivePositionRad", drivePositionRad);
        table.put("driveVelocityRadPerSec", driveVelocityRadPerSec);
        table.put("driveAppliedVolts", driveAppliedVolts);
        table.put("driveCurrentAmps", driveCurrentAmps);
        table.put("turnConnected", turnConnected);
        table.put("turnEncoderConnected", turnEncoderConnected);
        table.put("turnAbsolutePosition", turnAbsolutePosition);
        table.put("turnPosition", turnPosition);
        table.put("turnVelocityRadPerSec", turnVelocityRadPerSec);
        table.put("turnAppliedVolts", turnAppliedVolts);
        table.put("turnCurrentAmps", turnCurrentAmps);
        table.put("odometryTimestamps", odometryTimestamps);
        table.put("odometryDrivePositionsRad", odometryDrivePositionsRad);
        table.put("odometryTurnPositions", odometryTurnPositions);
    }

    public void fromLog(LogTable table) {
        driveConnected = table.get("driveConnected", driveConnected);
        drivePositionRad = table.get("drivePositionRad", drivePositionRad);
        driveVelocityRadPerSec = table.get("driveVelocityRadPerSec", driveVelocityRadPerSec);
        driveAppliedVolts = table.get("driveAppliedVolts", driveAppliedVolts);
        driveCurrentAmps = table.get("driveCurrentAmps", driveCurrentAmps);
        turnConnected = table.get("turnConnected", turnConnected);
        turnEncoderConnected = table.get("turnEncoderConnected", turnEncoderConnected);
        turnAbsolutePosition = table.get("turnAbsolutePosition", turnAbsolutePosition);
        turnPosition = table.get("turnPosition", turnPosition);
        turnVelocityRadPerSec = table.get("turnVelocityRadPerSec", turnVelocityRadPerSec);
        turnAppliedVolts = table.get("turnAppliedVolts", turnAppliedVolts);
        turnCurrentAmps = table.get("turnCurrentAmps", turnCurrentAmps);
        odometryTimestamps = table.get("odometryTimestamps", odometryTimestamps);
        odometryDrivePositionsRad = table.get("odometryDrivePositionsRad", odometryDrivePositionsRad);
        odometryTurnPositions = table.get("odometryTurnPositions", odometryTurnPositions);
    }
}

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified open loop value. */
  public default void setDriveOpenLoop(double output) {}

  /** Run the turn motor at the specified open loop value. */
  public default void setTurnOpenLoop(double output) {}

  /** Run the drive motor at the specified velocity. */
  public default void setDriveVelocity(double velocityRadPerSec) {}

  /** Run the turn motor to the specified rotation. */
  public default void setTurnPosition(Rotation2d rotation) {}
}
