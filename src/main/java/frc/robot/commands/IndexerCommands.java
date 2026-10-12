package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.Indexer;

public class IndexerCommands {

        /**
         * Igual que {@link #joystickIndexerCmd}, pero pausa la Y-valve mientras
         * el flywheel del cañón que se está alimentando se recupera del tiro
         * anterior. Ver {@code ShotConstants.feedGate*}.
         */
        public static Command gatedIndexerCmd(
                Indexer indexer,
                BooleanSupplier shooterSupplier,
                BooleanSupplier turretSupplier,
                BooleanSupplier shooterReady,
                BooleanSupplier turretReady) {
            return Commands.run(
                () -> {
                    boolean wantShooter = shooterSupplier.getAsBoolean();
                    boolean wantTurret = turretSupplier.getAsBoolean();

                    boolean ready;
                    if (wantShooter && wantTurret) {
                        ready = shooterReady.getAsBoolean() && turretReady.getAsBoolean();
                    } else if (wantShooter) {
                        ready = shooterReady.getAsBoolean();
                    } else if (wantTurret) {
                        ready = turretReady.getAsBoolean();
                    } else {
                        ready = true;
                    }
                    org.littletonrobotics.junction.Logger.recordOutput(
                        "Match/Feed/Paused", (wantShooter || wantTurret) && !ready);

                    if (!(wantShooter || wantTurret) || !ready) {
                        indexer.stopRollers();
                        indexer.stopIndexer();
                    } else if (wantShooter && wantTurret) {
                        indexer.intake();
                        indexer.indexBoth();
                    } else if (wantShooter) {
                        indexer.intake();
                        indexer.indexShooter();
                    } else {
                        indexer.intake();
                        indexer.indexTurret();
                    }
                },
                indexer);
        }

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
