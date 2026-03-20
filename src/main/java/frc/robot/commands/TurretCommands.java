package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    //  HOW THE ALGORITHM WORKS
    // ══════════════════════════════════════════════════════════════════════════
    //
    //  On first tracking call after enable, we compute field-absolute target
    //  positions from the robot's current pose:
    //
    //    hubTarget = initialPose + rotateBy(heading) * (hubOffsetX, hubOffsetY)
    //
    //  Every loop thereafter:
    //    dx, dy      = fieldTarget - robot_current_position
    //    worldAngle  = atan2(dy, dx)
    //    turretAngle = worldAngle - robotHeading - turretZeroOffset
    //
    //  The turret angle is then NORMALIZED into [minRotationRad, maxRotationRad]
    //  instead of the old [-π, +π] wrap. This ensures:
    //    1. Angles in the [180°, 243.6°] range are reachable
    //    2. Phoenix MotionMagic always receives a valid in-range setpoint
    //    3. The turret takes the direct path (no unnecessary full rotations)
    // ══════════════════════════════════════════════════════════════════════════

    // Cached targets — computed once per enable, null = needs re-init
    private static Translation2d cachedHubTarget = null;
    private static Translation2d cachedDsTarget  = null;

    /**
     * Call on every robot enable (teleop + auto init) to force target
     * recomputation from the actual robot pose at that moment.
     */
    public static void resetTargets() {
        cachedHubTarget = null;
        cachedDsTarget  = null;
    }

    /**
     * Lazy init — runs exactly once after resetTargets(), then is a no-op.
     * Converts robot-frame offsets to field-frame positions using initial heading.
     */
    private static void initializeTargets(Pose2d initialPose) {
        if (cachedHubTarget != null) return;

        // HUB: offsets are in ROBOT frame (X=forward, Y=left).
        // Rotate by initial heading to convert to field frame.
        cachedHubTarget = initialPose.getTranslation().plus(
            new Translation2d(TurretConstants.hubOffsetX, TurretConstants.hubOffsetY)
                .rotateBy(initialPose.getRotation())
        );

        // DRIVER STATION: along the turret-zero direction at enable time.
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

    // ── Core angle computation ─────────────────────────────────────────────────

    /**
     * Compute the turret setpoint in radians, normalized into the turret's
     * physical range [minRotationRad, maxRotationRad].
     *
     * Returns a raw double — NOT a Rotation2d — so values beyond ±180° are
     * preserved and sent to Phoenix MotionMagic without wrapping.
     */
    private static double computeTurretAngleRad(Pose2d robotPose, Translation2d fieldTarget) {
        double dx = fieldTarget.getX() - robotPose.getX();
        double dy = fieldTarget.getY() - robotPose.getY();
        double worldAngle   = Math.atan2(dy, dx);
        double robotHeading = robotPose.getRotation().getRadians();

        double rawAngle = worldAngle - robotHeading - TurretConstants.turretZeroOffsetRad;

        // ── Normalize into the turret's valid range ────────────────────────
        // Instead of angleModulus (which wraps to [-π, +π] and can't reach
        // angles in [180°, 243.6°]), we shift the angle into the range
        // [minRotationRad, minRotationRad + 2π).  Since the turret's range
        // spans exactly 360°, every field direction maps to exactly one
        // reachable turret position.
        double shifted = rawAngle - TurretConstants.minRotationRad;
        shifted = shifted - Math.floor(shifted / (2.0 * Math.PI)) * (2.0 * Math.PI);
        double targetAngle = shifted + TurretConstants.minRotationRad;

        // Safety clamp (should never trigger if range is exactly 360°)
        targetAngle = MathUtil.clamp(targetAngle,
            TurretConstants.minRotationRad, TurretConstants.maxRotationRad);

        return targetAngle;
    }

    // ── Public commands ────────────────────────────────────────────────────────

    /** Hold turret at physical zero (0 radians). Set as turret default command. */
    public static Command holdZero(Turret turret) {
        return Commands.run(
            () -> {
                Logger.recordOutput("Turret/TrackMode", "ZERO");
                turret.rotateToAngle(0.0);
            },
            turret
        ).withName("Turret:HoldZero");
    }

    /** Track the HUB. Toggle with driverJoystick.start(). */
    public static Command trackHub(Turret turret, Supplier<Pose2d> poseSupplier) {
        return Commands.run(
            () -> {
                Pose2d pose = poseSupplier.get();
                initializeTargets(pose);
                double angleRad = computeTurretAngleRad(pose, cachedHubTarget);
                Logger.recordOutput("Turret/TrackMode",     "HUB");
                Logger.recordOutput("Turret/TrackAngleDeg", Math.toDegrees(angleRad));
                Logger.recordOutput("Turret/RobotX",        pose.getX());
                Logger.recordOutput("Turret/RobotY",        pose.getY());
                turret.rotateToAngle(angleRad);
            },
            turret
        ).withName("Turret:TrackHub");
    }

    /** Track the Driver Station. Toggle with driverJoystick.back(). */
    public static Command trackDriverStation(Turret turret, Supplier<Pose2d> poseSupplier) {
        return Commands.run(
            () -> {
                Pose2d pose = poseSupplier.get();
                initializeTargets(pose);
                double angleRad = computeTurretAngleRad(pose, cachedDsTarget);
                Logger.recordOutput("Turret/TrackMode",     "DRIVER_STATION");
                Logger.recordOutput("Turret/TrackAngleDeg", Math.toDegrees(angleRad));
                turret.rotateToAngle(angleRad);
            },
            turret
        ).withName("Turret:TrackDS");
    }

    /** Auto-aim toward HUB, for use in autonomous routines. */
    public static Command autoAim(Turret turret, Supplier<Pose2d> poseSupplier) {
        return Commands.run(
            () -> {
                Pose2d pose = poseSupplier.get();
                initializeTargets(pose);
                double angleRad = computeTurretAngleRad(pose, cachedHubTarget);
                Logger.recordOutput("Turret/AutoAimAngleDeg", Math.toDegrees(angleRad));
                turret.rotateToAngle(angleRad);
            },
            turret
        ).withName("Turret:AutoAim");
    }

    /** Full teleop command (rotation + flywheels + indexer). */
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
                if (aimTurretSupplier.getAsBoolean()) {
                    Pose2d pose = poseSupplier.get();
                    initializeTargets(pose);
                    double angleRad = computeTurretAngleRad(pose, cachedHubTarget);
                    Logger.recordOutput("Debug/TargetTurretAngleDeg", Math.toDegrees(angleRad));
                    turret.rotateToAngle(angleRad);
                } else {
                    turret.rotateToAngle(0.0);
                }
                if (warmUpSupplier.getAsBoolean()) {
                    turret.startFlywheel();
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
