package frc.robot.commands.demo;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.intake.Intake;

/**
 * Comandos de intake del Demo Mode: agitación y extensión con remate por
 * corriente.
 */
public class DemoIntakeCommands {

    private DemoIntakeCommands() {
    }

    /**
     * Agita la caja para romper puentes de pelotas. La implementación vive en
     * {@link IntakeCommands#agitate(Intake)}; aquí sólo se delega para que los
     * bindings del demo no cambien.
     */
    public static Command agitate(Intake intake) {
        return IntakeCommands.agitate(intake);
    }

    /**
     * Extiende el intake y remata contra el tope mecánico por corriente. Ver
     * {@link IntakeCommands#extendWithStallHoming(Intake)}.
     */
    public static Command extendWithStallHoming(Intake intake) {
        return IntakeCommands.extendWithStallHoming(intake);
    }
}
