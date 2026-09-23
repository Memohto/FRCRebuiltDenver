package frc.robot.commands.competition;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.CompetitionState;
import frc.robot.util.ShotSolution;

/**
 * Cañón fijo en COMPETENCIA v2.
 *
 * <p>
 * Sólo participa en BOMBER: con la torreta en cero los dos cañones miran hacia
 * atrás y el chasis los orienta al objetivo. En STRIKER la que apunta es la
 * torreta y el fijo estaría tirando a donde nadie pidió, así que se queda
 * apagado — igual que en Denver.
 *
 * <p>
 * Usa la <b>misma</b> {@link ShotSolution} que la torreta, con los mismos
 * argumentos: los dos hoods reciben el mismo número por construcción.
 */
public final class CompShooterCommands {

    private CompShooterCommands() {
    }

    public static Command shooterCmd(
            Shooter shooter,
            Drive drive,
            BooleanSupplier chargeSupplier) {

        return Commands.run(
                () -> {
                    boolean shouldShoot = chargeSupplier.getAsBoolean() && CompetitionState.isBomber();

                    if (!shouldShoot) {
                        shooter.stopFlywheel();
                        shooter.setHoodAtInitialPosition();
                        Logger.recordOutput("Match/FixedShooterActive", false);
                        return;
                    }

                    Pose2d pose = drive.getPose();
                    Translation2d target = CompetitionState.getActiveTarget(pose);
                    ShotSolution shot = ShotSolution.compute(
                            pose, drive.getFieldRelativeVelocity(), target);

                    shooter.setHoodForDistance(shot.distanceMeters);
                    shooter.setFlywheelVelocityForDistance(shot.distanceMeters);
                    Logger.recordOutput("Match/FixedShooterActive", true);
                },
                shooter);
    }
}
