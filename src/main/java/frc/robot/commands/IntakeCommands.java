package frc.robot.commands;

import frc.robot.subsystems.intake.Intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakeCommands {
    public static Command joystickIntakeCmd(
        Intake intake,
        BooleanSupplier intakeupplier,
        BooleanSupplier outtakeupplier) {
        return Commands.run(
            () -> {
                boolean in = intakeupplier.getAsBoolean() && !outtakeupplier.getAsBoolean();
                boolean out = outtakeupplier.getAsBoolean() && !intakeupplier.getAsBoolean();
                boolean extend = in || out;

                if (extend ) {
                    intake.setExtended();
                    if(in) {
                        intake.intake();
                    } else if (out) {
                        intake.outtake();
                    }
                } else if (!extend ) {
                    intake.setExtendedReset();
                    intake.stopRollers();
                } else {
                    intake.stopExtensor();
                }
            },
        intake);
    }
}