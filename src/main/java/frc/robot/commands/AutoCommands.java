package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.turret.Turret;

public class AutoCommands {

    public static Command shootingFixed(Shooter shooter) {
        return Commands.run(
            () -> {
                shooter.startFlywheel();
            }
        );
    }

    public static Command shootingTurret(Turret turret) {
        return Commands.run(
            () -> {
                turret.startFlywheel();
            }
        );
    }

    public static Command indexerBoth(Indexer indexer) {

        return Commands.run(
            () -> indexer.indexBoth()
        );
    }

    public static Command shootBothCmd(Indexer indexer, Shooter shooter, Turret turret) {

        return Commands.sequence(
            Commands.parallel(
                    shootingFixed(shooter),
                    shootingTurret(turret)
                ).withTimeout(2.0),

            Commands.parallel(  
                indexerBoth(indexer)
                ).withTimeout(10),

            Commands.run(() -> {
                shooter.stopFlywheel();
                turret.stopFlywheel();
                indexer.stopIndexer();
            })
        );
    }
}