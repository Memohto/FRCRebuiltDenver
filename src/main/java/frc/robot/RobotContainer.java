// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IndexerCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.TurretCommands;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.constants.RobotConstants.DriveMode;
import frc.robot.constants.RobotConstants.RobotMode;
import frc.robot.constants.RobotConstants.TurretMode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import static frc.robot.constants.VisionConstants.*;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private final Intake intake;
    private final Indexer indexer;
    private final Turret turret;
    private final Shooter shooter;

    private final CommandXboxController driverJoystick = new CommandXboxController(0);
    private final CommandXboxController mechanismsJoystick = new CommandXboxController(1);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (RobotConstants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive =
                    new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));
                new Vision(
                    drive::addVisionMeasurement,
                    new VisionIOLimelight(LimelightFixedCamera, drive::getRotation));
            
                intake = new Intake(new IntakeIOTalonFX(
                    IntakeConstants.rollersCanId,
                    IntakeConstants.extensorCanId
                ));
                indexer = new Indexer(new IndexerIOTalonFX(
                    IndexerConstants.rollersCanId,
                    IndexerConstants.shooterWheelsCanId,
                    IndexerConstants.turretWheelsCanId,
                    IndexerConstants.feederCanId
                ));
                shooter = new Shooter(new ShooterIOTalonFX(
                    ShooterConstants.flywheelCanId,
                    ShooterConstants.hoodCanId
                ));
                turret = new Turret(
                    new TurretIOTalonFX(
                        TurretConstants.flywheelCanId,
                        TurretConstants.hoodCanId,
                        TurretConstants.rotationMotorCanId),
                    new ShooterIOTalonFX(
                        TurretConstants.flywheelCanId,
                        TurretConstants.hoodCanId));
                break;
            
            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive =
                    new Drive(
                        new GyroIO() {},
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));
                new Vision(
                    drive::addVisionMeasurement,
                    new VisionIOPhotonVisionSim(LimelightFixedCamera, robotToLimelightFixed, drive::getPose));
            
                intake = new Intake(new IntakeIOSim());
                indexer = new Indexer(new IndexerIOSim());
                shooter = new Shooter(new ShooterIOSim());
                turret = new Turret(new TurretIOSim(), new ShooterIOSim());
                break;
            
            default:
                // Replayed robot, disable IO implementations
                drive =
                    new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {});
                new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
            
                intake  = new Intake(new IntakeIOSim() {});
                indexer = new Indexer(new IndexerIOSim() {});
                shooter = new Shooter(new ShooterIOSim() {});
                turret  = new Turret(new TurretIOSim() {}, new ShooterIOSim() {});
                break;
        }

        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        NamedCommands.registerCommand("StartFlywheels", Commands.runOnce(
            () -> {
                shooter.startFlywheel();
                turret.startFlywheel();
            }, shooter, turret).withTimeout(2));
        NamedCommands.registerCommand("IndexBoth", Commands.sequence(
            Commands.runOnce(() -> { indexer.intake(); }, indexer),
            Commands.runOnce(() -> { indexer.indexBoth(); }, indexer),
            Commands.waitSeconds(1),
            Commands.runOnce(() -> { indexer.outtake(); }, indexer),
            Commands.waitSeconds(0.25)
        ).repeatedly().withTimeout(22));
        
        // Set up SysId routines
        autoChooser.addOption(
            "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption(
            "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
            "Drive SysId (Quasistatic Forward)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
            "Drive SysId (Quasistatic Reverse)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
            "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
            "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
        * instantiating a {@link GenericHID} or one of its subclasses ({@link
        * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
        * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
        */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive,
                () -> -driverJoystick.getLeftY(),
                () -> -driverJoystick.getLeftX(),
                () -> -driverJoystick.getRightX()));

        // Lock to 0° when A button is held
        driverJoystick
            .a()
            .whileTrue(
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -driverJoystick.getLeftY(),
                    () -> -driverJoystick.getLeftX(),
                    () -> Rotation2d.kZero));

        // Reset gyro to 0° when B button is pressed
        driverJoystick
            .b()
            .onTrue(
                Commands.runOnce(
                        () -> {
                            boolean isFlipped =
                                DriverStation.getAlliance().isPresent()
                                    && DriverStation.getAlliance().get() == Alliance.Red;
                                drive.setPose(
                                    new Pose2d(drive.getPose().getTranslation(), isFlipped ? Rotation2d.k180deg : Rotation2d.kZero));
                        },
                        drive)
                    .ignoringDisable(true));
                    
        driverJoystick.x().onTrue(Commands.runOnce(() -> {
            Robot.mode = RobotMode.BOMBER;
            Drive.mode = DriveMode.ORBIT;
            Turret.mode = TurretMode.NORMAL;
            Logger.recordOutput("RobotMode", "BOMBER");
            Logger.recordOutput("DriveMode", "ORBIT");
            Logger.recordOutput("TurretMode", "NORMAL");
        }));
        driverJoystick.y().onTrue(Commands.runOnce(() -> {
            Robot.mode = RobotMode.BOMBER;
            Drive.mode = DriveMode.FEEDER;
            Turret.mode = TurretMode.NORMAL;
            Logger.recordOutput("RobotMode", "BOMBER");
            Logger.recordOutput("DriveMode", "FEEDER");
            Logger.recordOutput("TurretMode", "NORMAL");
        }));
        driverJoystick.start().onTrue(Commands.runOnce(() -> {
            Robot.mode = RobotMode.STRIKER;
            Drive.mode = DriveMode.NORMAL;
            Turret.mode = TurretMode.HUB_TRACKER;
            Logger.recordOutput("RobotMode", "STRIKER");
            Logger.recordOutput("DriveMode", "NORMAL");
            Logger.recordOutput("TurretMode", "HUB_TRACKER");
        }));
        driverJoystick.back().onTrue(Commands.runOnce(() -> {
            Robot.mode = RobotMode.STRIKER;
            Drive.mode = DriveMode.NORMAL;
            Turret.mode = TurretMode.DS_TRACKER;
            Logger.recordOutput("RobotMode", "STRIKER");
            Logger.recordOutput("DriveMode", "NORMAL");
            Logger.recordOutput("TurretMode", "DS_TRACKER");
        }));

        // driverJoystick.x().toggleOnTrue(TurretCommands.bomberHub(turret, drive::getPose));
        // driverJoystick.start().toggleOnTrue(TurretCommands.trackHub(turret, drive::getPose));
        // driverJoystick.back().toggleOnTrue(TurretCommands.trackDriverStation(turret, drive::getPose));

        turret.setDefaultCommand(TurretCommands.holdZero(turret));

        shooter.setDefaultCommand(
            ShooterCommands.joystickShooterCmd(
                shooter, turret, 
                () -> mechanismsJoystick.x().getAsBoolean(),
                () -> {
                    // Flip everything if red alliance
                    boolean isFlipped = DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red;
                    return drive.getDistanceToTargetMeters(
                        isFlipped ? RobotConstants.redHub : RobotConstants.blueHub
                    );
                }));

        //Mechanism
        mechanismsJoystick.povUp()
            .whileTrue(
                Commands.runEnd(
                    () -> {
                        shooter.debugFlywheel();
                        turret.debugFlywheel();
                    }, 
                    () -> {
                        shooter.stopFlywheel();
                        turret.stopFlywheel();
                    },
                    shooter, turret));
                
        // Outtake ball from indexer (Fallback)
        mechanismsJoystick.rightTrigger(0.5)
            .whileTrue(
                Commands.runEnd(
                    () -> {
                        indexer.outtake();
                    },
                    () -> {
                        indexer.stopIndexer();
                    },
                    indexer));
                
        // Intake
        mechanismsJoystick.leftTrigger(0.5)
            .whileTrue(
                Commands.runEnd(
                    () -> {
                        intake.intake();
                    }, () -> {
                        intake.stopRollers();
                    }, 
                    intake));
                
        // Extend
        mechanismsJoystick.a()
            .whileTrue(
                Commands.runEnd(
                    () -> {
                        intake.extend();
                    }, 
                    () -> {
                        intake.stopExtensor();
                    }, 
                    intake));
                
        // Retract
        mechanismsJoystick.b()
            .whileTrue(
                Commands.runEnd(
                    () -> {
                        intake.retract();
                    }, 
                    () -> {
                        intake.stopExtensor();
                    }, 
                    intake));
                
        // Shoot
        indexer.setDefaultCommand(
            IndexerCommands.joystickIndexerCmd(
                indexer,
                () -> mechanismsJoystick.leftBumper().getAsBoolean(),
                () -> mechanismsJoystick.rightBumper().getAsBoolean()
            )
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
        *
        * @return the command to run in autonomous
        */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    /**
     * Called on every robot enable (teleop + auto init) via Robot.java.
    * Resets turret target cache so targets are recomputed from the
    * robot's current pose at the start of each match / enable cycle.
    */
    public void onEnable() {
        TurretCommands.resetTargets();
    }
}   