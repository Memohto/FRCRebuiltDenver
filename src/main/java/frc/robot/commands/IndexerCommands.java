package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.Indexer;

public class IndexerCommands {
        public static Command joystickIndexerCmd(
        Indexer indexer,
        BooleanSupplier shooterSupplier,
        BooleanSupplier turretSupplier) {
        return Commands.run(
            () -> {
                boolean indexShooter = shooterSupplier.getAsBoolean() && !turretSupplier.getAsBoolean();
                boolean indexTurret = turretSupplier.getAsBoolean() && !shooterSupplier.getAsBoolean();
                boolean indexBoth = shooterSupplier.getAsBoolean() && turretSupplier.getAsBoolean();

                if (indexBoth) {
                    indexer.intake();
                    indexer.indexBoth();
                } else if (indexShooter) {
                    indexer.intake();
                    indexer.indexShooter();
                } else if (indexTurret) {
                    indexer.intake();
                    indexer.indexTurret();
                } else {
                    indexer.stopRollers();
                    indexer.stopIndexer();
                }
            },
        indexer);
    }
}
