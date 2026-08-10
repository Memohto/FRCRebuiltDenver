package frc.robot.commands.demo;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.DemoConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.DemoState;
import frc.robot.util.FieldTracking;
import frc.robot.util.ShotSolution;

/**
 * Comportamiento del shooter fijo en Demo Mode.
 *
 * <p>
 * El shooter fijo sólo participa en BOMBER. En STRIKER la que apunta es la
 * torreta, y el cañón fijo apuntaría a donde nadie pidió — peligroso en una
 * demo con gente alrededor, así que se mantiene apagado.
 */
public class DemoShooterCommands {

    private static final int CAM = DemoDriveCommands.TURRET_CAMERA;

    private DemoShooterCommands() {
    }

    /**
     * Default command del shooter fijo.
     *
     * <p>
     * En BOMBER, con la torreta congelada en cero, ambos cañones apuntan hacia
     * la trasera del robot y el chasis se encarga de orientarla al HUB. Los dos
     * disparan a la misma distancia: es el modo de volumen.
     */
    public static Command demoShooterCmd(
            Shooter shooter,
            Drive drive,
            BooleanSupplier chargeSupplier,
            Supplier<Pose2d> poseSupplier) {

        return Commands.run(
                () -> {
                    // El cañón fijo sólo se carga orbitando el HUB. En STRIKER
                    // apunta la torreta y el fijo estaría tirando a donde nadie
                    // pidió; durante el follow-me hay una persona enfrente.
                    boolean shouldShoot = chargeSupplier.getAsBoolean()
                            && DemoState.isBomber()
                            && !DemoState.isFollowing();

                    if (!shouldShoot) {
                        shooter.stopFlywheel();
                        shooter.setHoodAtInitialPosition();
                        Logger.recordOutput("Demo/FixedShooterActive", false);
                        return;
                    }

                    if (DemoState.isSmoothDump()) {
                        // Volcado suave: hood plano y potencia fija, igual que
                        // la torreta. Los dos cañones salen parejos.
                        shooter.setHoodPosition(
                                Rotation2d.fromDegrees(DemoConstants.dumpHoodDegrees));
                        shooter.setFlywheelVelocity(
                                Units.rotationsToRadians(DemoConstants.dumpFlywheelRPS()));
                        Logger.recordOutput("Demo/FixedShooterActive", true);
                        return;
                    }

                    // MISMA solución que usa la torreta, con los mismos
                    // argumentos. Los dos hoods reciben el mismo número por
                    // construcción, no por coincidencia.
                    ShotSolution shot = ShotSolution.compute(
                            poseSupplier.get(),
                            drive.getFieldRelativeVelocity(),
                            FieldTracking.getActiveTarget());
                    double distance = shot.distanceMeters;
                    shooter.setHoodForDistance(distance);
                    shooter.setFlywheelVelocityForDistance(distance);
                    Logger.recordOutput("Demo/FixedShooterActive", true);
                },
                shooter);
    }

}
