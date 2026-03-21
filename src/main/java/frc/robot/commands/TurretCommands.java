package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import org.littletonrobotics.junction.Logger;

public class TurretCommands {

    private TurretCommands() {}

    // ══════════════════════════════════════════════════════════════════════════
    //  TRACKING STATE — readable by ShooterCommands / DriveCommands
    // ══════════════════════════════════════════════════════════════════════════

    private static boolean tracking = false;
    private static double  distanceToTargetM = 0.0;

    /** True when trackHub is active (turret aims independently). */
    public static boolean isTracking() { return tracking; }

    /** Current distance to HUB in meters (valid when isTracking or isBomberMode). */
    public static double getDistanceToTarget() { return distanceToTargetM; }

    // ══════════════════════════════════════════════════════════════════════════
    //  BOMBER MODE STATE — chassis aims, both shooters fire
    // ══════════════════════════════════════════════════════════════════════════
    //
    //  When bomber mode is active:
    //    - DriveCommands overrides swerve rotation to aim the robot's BACK
    //      (fixed shooter side) at the HUB
    //    - Turret holds at zero (both shooters face same direction)
    //    - Both hoods auto-adjust based on distance (same kTurretHoodMap)
    //    - When copilot presses flywheel button, BOTH flywheels use
    //      distance-based speed (same kTurretFlywheelMap)
    //

    private static boolean bomberMode = false;

    /** True when bomber mode is active (chassis aims at HUB). */
    public static boolean isBomberMode() { return bomberMode; }

    // ══════════════════════════════════════════════════════════════════════════

    // Cached targets — computed once per enable, null = needs re-init
    private static Translation2d cachedHubTarget = null;
    private static Translation2d cachedDsTarget  = null;

    /** Returns the cached HUB field position, or null if not yet initialized. */
    public static Translation2d getCachedHubTarget() { return cachedHubTarget; }

    public static void resetTargets() {
        cachedHubTarget = null;
        cachedDsTarget  = null;
        tracking = false;
        bomberMode = false;
        distanceToTargetM = 0.0;
    }

    private static void initializeTargets(Pose2d initialPose) {
        if (cachedHubTarget != null) return;

        cachedHubTarget = initialPose.getTranslation().plus(
            new Translation2d(TurretConstants.hubOffsetX, TurretConstants.hubOffsetY)
                .rotateBy(initialPose.getRotation())
        );

        double zeroDirection = initialPose.getRotation().getRadians()
                               + TurretConstants.turretZeroOffsetRad;
        cachedDsTarget = new Translation2d(
            initialPose.getX() + TurretConstants.driverStationDistance * Math.cos(zeroDirection),
            initialPose.getY() + TurretConstants.driverStationDistance * Math.sin(zeroDirection)
        );

        Logger.recordOutput("Turret/Init/RobotX",       initialPose.getX());
        Logger.recordOutput("Turret/Init/RobotY",       initialPose.getY());
        Logger.recordOutput("Turret/Init/HeadingDeg",   initialPose.getRotation().getDegrees());
        Logger.recordOutput("Turret/Init/HubTargetX",   cachedHubTarget.getX());
        Logger.recordOutput("Turret/Init/HubTargetY",   cachedHubTarget.getY());
        Logger.recordOutput("Turret/Init/DSTargetX",    cachedDsTarget.getX());
        Logger.recordOutput("Turret/Init/DSTargetY",    cachedDsTarget.getY());
    }

    // ── Core computations ──────────────────────────────────────────────────────

    private static double computeTurretAngleRad(Pose2d robotPose, Translation2d fieldTarget) {
        double dx = fieldTarget.getX() - robotPose.getX();
        double dy = fieldTarget.getY() - robotPose.getY();
        double worldAngle   = Math.atan2(dy, dx);
        double robotHeading = robotPose.getRotation().getRadians();

        double rawAngle = worldAngle - robotHeading - TurretConstants.turretZeroOffsetRad;

        double shifted = rawAngle - TurretConstants.minRotationRad;
        shifted = shifted - Math.floor(shifted / (2.0 * Math.PI)) * (2.0 * Math.PI);
        double targetAngle = shifted + TurretConstants.minRotationRad;

        targetAngle = MathUtil.clamp(targetAngle,
            TurretConstants.minRotationRad, TurretConstants.maxRotationRad);

        return targetAngle;
    }

    private static double computeDistanceToTarget(Pose2d robotPose, Translation2d fieldTarget) {
        double dx = fieldTarget.getX() - robotPose.getX();
        double dy = fieldTarget.getY() - robotPose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    // ── Public commands ────────────────────────────────────────────────────────

    /**
     * Hold turret at physical zero. Hood at starting position. NOT tracking.
     * Flywheel is NOT touched — copilot controls it via ShooterCommands.
     */
    public static Command holdZero(Turret turret) {
        return Commands.run(
            () -> {
                tracking = false;
                bomberMode = false;
                Logger.recordOutput("Turret/TrackMode", "ZERO");
                turret.rotateToAngle(0.0);
                turret.setHoodInitialPosition();
            },
            turret
        ).withName("Turret:HoldZero");
    }

    /**
     * Track the HUB — rotation + hood auto-adjust. Flywheel controlled by copilot.
     */
    public static Command trackHub(Turret turret, Supplier<Pose2d> poseSupplier) {
        return Commands.run(
            () -> {
                Pose2d pose = poseSupplier.get();
                initializeTargets(pose);

                double angleRad = computeTurretAngleRad(pose, cachedHubTarget);
                turret.rotateToAngle(angleRad);

                double distanceM = computeDistanceToTarget(pose, cachedHubTarget);
                turret.setHoodForDistance(distanceM);

                tracking = true;
                bomberMode = false;
                distanceToTargetM = distanceM;

                Logger.recordOutput("Turret/TrackMode",     "HUB");
                Logger.recordOutput("Turret/TrackAngleDeg", Math.toDegrees(angleRad));
                Logger.recordOutput("Turret/DistanceM",     distanceM);
                Logger.recordOutput("Turret/RobotX",        pose.getX());
                Logger.recordOutput("Turret/RobotY",        pose.getY());
            },
            turret
        ).withName("Turret:TrackHub");
    }

    /** Track the Driver Station. NOT tracking. */
    public static Command trackDriverStation(Turret turret, Supplier<Pose2d> poseSupplier) {
        return Commands.run(
            () -> {
                tracking = false;
                bomberMode = false;
                Pose2d pose = poseSupplier.get();
                initializeTargets(pose);
                double angleRad = computeTurretAngleRad(pose, cachedDsTarget);
                turret.rotateToAngle(angleRad);

                Logger.recordOutput("Turret/TrackMode",     "DRIVER_STATION");
                Logger.recordOutput("Turret/TrackAngleDeg", Math.toDegrees(angleRad));
            },
            turret
        ).withName("Turret:TrackDS");
    }

    // ═══════════════════════════════════════════════════════════════════════════
    //  BOMBER MODE — chassis aims, turret holds zero, both hoods auto-adjust
    // ═══════════════════════════════════════════════════════════════════════════
    //
    //  How it works:
    //    1. This command holds the turret at zero and adjusts the turret hood.
    //    2. DriveCommands reads isBomberMode() to override swerve rotation,
    //       pointing the robot's BACK (fixed shooter) at the HUB.
    //    3. ShooterCommands reads isBomberMode() to auto-adjust the fixed hood
    //       and use distance-based flywheel speed for BOTH shooters.
    //
    //  The result: both shooters face the same direction (backward), both hoods
    //  track the target distance, and the copilot's X button fires both at the
    //  interpolated speed. Double output power.
    //

    /**
     * Bomber mode — turret at zero, turret hood auto-adjusts.
     * DriveCommands handles chassis rotation. ShooterCommands handles fixed hood + flywheels.
     * Toggle with driverJoystick.x().
     */
    public static Command bomberHub(Turret turret, Supplier<Pose2d> poseSupplier) {
        return Commands.run(
            () -> {
                Pose2d pose = poseSupplier.get();
                initializeTargets(pose);

                // Turret holds at zero — chassis does the aiming
                turret.rotateToAngle(0.0);

                // Distance for hood + flywheel interpolation
                double distanceM = computeDistanceToTarget(pose, cachedHubTarget);

                // Turret hood auto-adjusts (fixed hood handled by ShooterCommands)
                turret.setHoodForDistance(distanceM);

                // Publish state for DriveCommands and ShooterCommands
                bomberMode = true;
                tracking = false;
                distanceToTargetM = distanceM;

                Logger.recordOutput("Turret/TrackMode",  "BOMBER");
                Logger.recordOutput("Turret/DistanceM",  distanceM);
                Logger.recordOutput("Turret/RobotX",     pose.getX());
                Logger.recordOutput("Turret/RobotY",     pose.getY());
            },
            turret
        ).withName("Turret:BomberHub");
    }

    // ═══════════════════════════════════════════════════════════════════════════

    /** Auto-aim toward HUB with hood + flywheel, for autonomous. */
    public static Command autoAim(Turret turret, Supplier<Pose2d> poseSupplier) {
        return Commands.run(
            () -> {
                Pose2d pose = poseSupplier.get();
                initializeTargets(pose);

                double angleRad = computeTurretAngleRad(pose, cachedHubTarget);
                turret.rotateToAngle(angleRad);

                double distanceM = computeDistanceToTarget(pose, cachedHubTarget);
                turret.setHoodForDistance(distanceM);
                turret.startFlywheelForDistance(distanceM);

                tracking = true;
                distanceToTargetM = distanceM;

                Logger.recordOutput("Turret/AutoAimAngleDeg", Math.toDegrees(angleRad));
                Logger.recordOutput("Turret/AutoAimDistanceM", distanceM);
            },
            turret
        ).withName("Turret:AutoAim");
    }

    /** Full teleop command (rotation + hood + flywheels + indexer). */
    public static Command joystickTurretCmd(
        Supplier<Pose2d> poseSupplier,
        Turret turret,
        Shooter shooter,
        Indexer indexer,
        BooleanSupplier warmUpSupplier,
        BooleanSupplier aimTurretSupplier,
        BooleanSupplier shootSupplier) {

        return Commands.run(
            () -> {
                double distanceM = 0.0;

                if (aimTurretSupplier.getAsBoolean()) {
                    Pose2d pose = poseSupplier.get();
                    initializeTargets(pose);

                    double angleRad = computeTurretAngleRad(pose, cachedHubTarget);
                    turret.rotateToAngle(angleRad);

                    distanceM = computeDistanceToTarget(pose, cachedHubTarget);
                    turret.setHoodForDistance(distanceM);

                    tracking = true;
                    distanceToTargetM = distanceM;

                    Logger.recordOutput("Debug/TargetTurretAngleDeg", Math.toDegrees(angleRad));
                    Logger.recordOutput("Debug/TargetDistanceM", distanceM);
                } else {
                    tracking = false;
                    turret.rotateToAngle(0.0);
                    turret.setHoodInitialPosition();
                }

                if (warmUpSupplier.getAsBoolean()) {
                    if (tracking && distanceM > 0.0) {
                        turret.startFlywheelForDistance(distanceM);
                    } else {
                        turret.startFlywheel();
                    }
                } else {
                    shooter.stopFlywheel();
                    turret.stopFlywheel();
                }

                if (shootSupplier.getAsBoolean()) {
                    if (turret.isFlywheelAtSpeed() && shooter.isFlywheelAtSpeed()) {
                        indexer.intake(); indexer.indexBoth();
                    } else if (turret.isFlywheelAtSpeed()) {
                        indexer.intake(); indexer.indexTurret();
                    } else if (shooter.isFlywheelAtSpeed()) {
                        indexer.intake(); indexer.indexShooter();
                    } else {
                        indexer.stopIndexer(); indexer.stopRollers();
                    }
                } else {
                    indexer.stopIndexer(); indexer.stopRollers();
                }
            },
            turret, shooter, indexer
        ).withName("Turret:JoystickTurret");
    }
}