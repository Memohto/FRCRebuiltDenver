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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.demo.DemoDriveCommands;
import frc.robot.commands.demo.DemoIntakeCommands;
import frc.robot.commands.demo.DemoShooterCommands;
import frc.robot.commands.demo.DemoTurretCommands;
import frc.robot.constants.DemoConstants;
import frc.robot.util.DemoDashboard;
import frc.robot.util.DemoState;
import frc.robot.commands.DriveCommands;
import java.util.function.BooleanSupplier;
import frc.robot.commands.IndexerCommands;
import frc.robot.commands.IntakeCommands;
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
import frc.robot.Timer;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final Indexer indexer;
  private final Turret turret;
  private final Shooter shooter;
  private final Vision vision;

  // Joysticks
  private final CommandXboxController driverJoystick = new CommandXboxController(0);
  private final CommandXboxController mechanismsJoystick = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // instanciar timer
  @SuppressWarnings("unused")
  private final Timer timer = new Timer(); // basta con instanciarla

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (RobotConstants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight));
        // En demo la cámara vive en la torreta; el nombre NT lo define
        // DemoConstants para que no haya que tocar dos archivos si la renombran.
        vision = new Vision(
            drive::addVisionMeasurement,
            new VisionIOLimelight(
                RobotConstants.isDemoMode ? DemoConstants.turretCameraName : LimelightFixedCamera,
                drive::getRotation));

        intake = new Intake(new IntakeIOTalonFX(
            IntakeConstants.rollersCanId,
            IntakeConstants.extensorCanId));
        indexer = new Indexer(new IndexerIOTalonFX(
            IndexerConstants.rollersCanId,
            IndexerConstants.shooterWheelsCanId,
            IndexerConstants.turretWheelsCanId,
            IndexerConstants.feederCanId));
        shooter = new Shooter(new ShooterIOTalonFX(
            ShooterConstants.flywheelCanId,
            ShooterConstants.hoodCanId));
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
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight));
        // En simulación la transformada de cámara sigue siendo fija: el
        // seguimiento de torreta sólo se puede probar en el robot real.
        vision = new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVisionSim(LimelightFixedCamera, robotToLimelightFixed, drive::getPose));

        intake = new Intake(new IntakeIOSim());
        indexer = new Indexer(new IndexerIOSim());
        shooter = new Shooter(new ShooterIOSim());
        turret = new Turret(new TurretIOSim(), new ShooterIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            });
        // Una sola cámara, igual que REAL y SIM: si aquí se instancian dos, el
        // replay no reproduce la misma estructura de log que el robot real.
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {
        });

        intake = new Intake(new IntakeIOSim() {
        });
        indexer = new Indexer(new IndexerIOSim() {
        });
        shooter = new Shooter(new ShooterIOSim() {
        });
        turret = new Turret(new TurretIOSim() {
        }, new ShooterIOSim() {
        });
        break;
    }

    NamedCommands.registerCommand("AimTurret",
        Commands.run(() -> {
          boolean isRedAlliance = DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
          double distanceM = Drive.getDistanceToTargetMeters(drive.getPose(),
              isRedAlliance ? RobotConstants.redHub : RobotConstants.blueHub);
          double angle = Turret.computeTurretAngleRad(drive.getPose(),
              isRedAlliance ? RobotConstants.redHub : RobotConstants.blueHub);
          turret.setFlywheelVelocityForDistance(distanceM);
          turret.setHoodForDistance(distanceM);
          turret.rotateToAngle(angle);
        }, turret));
    NamedCommands.registerCommand("ResetTurret",
        Commands.runOnce(() -> {
          turret.setHoodAtInitialPosition();
          turret.stopFlywheel();
          turret.rotateToAngle(0.0);
        }, turret));
    NamedCommands.registerCommand("IndexTurret", Commands.sequence(
        Commands.runOnce(() -> {
          indexer.intake();
        }, indexer),
        Commands.runOnce(() -> {
          indexer.indexTurret();
        }, indexer),
        Commands.waitSeconds(1),
        Commands.runOnce(() -> {
          indexer.outtake();
        }, indexer),
        Commands.waitSeconds(0.25)).repeatedly());
    NamedCommands.registerCommand("IndexBoth", Commands.sequence(
        Commands.runOnce(() -> {
          indexer.intake();
        }, indexer),
        Commands.runOnce(() -> {
          indexer.indexBoth();
        }, indexer),
        Commands.waitSeconds(1),
        Commands.runOnce(() -> {
          indexer.outtake();
        }, indexer),
        Commands.waitSeconds(0.25)).repeatedly());
    NamedCommands.registerCommand("ExtendIntake",
        Commands.runOnce(() -> {
          intake.setExtended();
          intake.intake();
        }, intake));
    NamedCommands.registerCommand("RetractIntake",
        Commands.runOnce(() -> intake.setExtendedReset(), intake));
    NamedCommands.registerCommand("StartIntakeRollers",
        Commands.sequence(
            Commands.runOnce(() -> intake.intake(), intake),
            Commands.waitSeconds(0.25),
            Commands.runOnce(() -> intake.stopRollers(), intake),
            Commands.waitSeconds(0.25)).repeatedly());
    NamedCommands.registerCommand("StopIntakeRollers", Commands.runOnce(() -> intake.stopRollers(), intake));

    // ══════════════════════════════════════════════════════════════════════
    // Cableado de la visión en Demo Mode
    // ══════════════════════════════════════════════════════════════════════
    if (RobotConstants.isDemoMode) {
      // La cámara va montada en la torreta, así que la transformada
      // robot→cámara cambia cada ciclo. Sin esto, MegaTag y
      // targetpose_robotspace calculan con una pose de cámara obsoleta.
      vision.setDynamicCameraTransform(
          DemoDriveCommands.TURRET_CAMERA, turret::getRobotToCamera);

      // Odometría de campo a partir de los AprilTags del HUB.
      //
      // El filtro es lo que hace esto seguro fuera de cancha: sólo se aceptan
      // correcciones de pose cuando TODOS los tags visibles son del HUB. Un tag
      // suelto que un alumno traiga en la mano nunca toca la odometría, pero el
      // HUB sí localiza al robot — y con eso el robot puede seguir apuntando
      // varios segundos después de perder el tag de vista, igual que en
      // competencia.
      vision.setPoseEstimationEnabled(() -> DemoConstants.useVisionOdometry);
      vision.setTrustedTagFilter(DemoState::isHubTag);
    }

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

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
    if (RobotConstants.isDemoMode) {
      configureDemoBindings();
    } else {
      configureButtonBindings();
    }
  }

  // ══════════════════════════════════════════════════════════════════════════
  //
  //  D E M O   M O D E
  //
  //  Bindings y default commands para actividades de reclutamiento e
  //  integración. Nada de lo que hay aquí abajo toca el código de competencia:
  //  se activa poniendo RobotConstants.isDemoMode = true y se desactiva
  //  poniéndolo en false.
  //
  //  ── FLUJO DE OPERACIÓN ─────────────────────────────────────────────────
  //
  //    1. SELECCIONAR MODO   (Y)   → define qué hará el gatillo
  //    2. APUNTAR o CARGAR   (X / RT)
  //         X  = la torreta rastrea, los flywheels NO giran
  //         RT = rastrea Y acelera a la solución de tiro (el gatillo)
  //    3. ALIMENTAR          (LB / RB, la Y-valve)
  //         la pelota entra a un cañón que ya está acelerado y apuntando
  //
  //  Sin botón presionado la torreta está INACTIVA y no busca nada.
  //
  //  ── PILOTO (puerto 0) — deliberadamente mínimo ─────────────────────────
  //     Stick izq.  Traslación suavizada, sin flip de alianza
  //     Stick der.  Giro (salvo que el chasis esté asistido)
  //     LT          Modo precisión: baja al 30%
  //     B           Reset del frente. Independiente de cancha, sirve
  //                 habilitado y deshabilitado.
  //
  //  ── OPERADOR (puerto 1) — todo lo demás ────────────────────────────────
  //     X (hold)    APUNTAR — torreta rastrea, sin flywheels
  //     RT (hold)   CARGAR  — rastrea + acelera (el gatillo)
  //     LB / RB     Alimentar cañón fijo / torreta (+ agitación automática)
  //     LT          Rodillos de intake
  //     Y           Alterna STRIKER / BOMBER
  //     Back        Smart feature del modo activo
  //                   STRIKER → fija / libera el tag visible
  //                   BOMBER  → orbitar HUB / follow-me ("perrito")
  //     Start       Rota el pipeline de la Limelight a mano
  //     A / B       Extender / retraer intake
  //     POV ↑       Prueba de flywheels al 25%
  //     POV ↓       Agitación manual de la caja
  //     POV →       Desatascar indexer (reversa)
  //     LS / RS     Jog del extensor sin soft limits (recalibración)
  //
  // ══════════════════════════════════════════════════════════════════════════
  private void configureDemoBindings() {
    // Se crean una sola vez y se reutilizan: crear Triggers dentro de un
    // lambda que corre a 50 Hz genera basura innecesaria cada ciclo.
    // En modo solo, TODO va al control del piloto (puerto 0). El puerto 1
    // queda sin usar.
    final boolean solo = RobotConstants.isSoloDemo;
    final CommandXboxController mech = solo ? driverJoystick : mechanismsJoystick;

    Trigger aimTrigger = mech.x();
    Trigger chargeTrigger = mech.rightTrigger(0.5);
    Trigger feedShooterTrigger = mech.leftBumper();
    Trigger feedTurretTrigger = mech.rightBumper();

    // "El chasis debe asistir" = el operador está apuntando o cargando.
    BooleanSupplier assistSupplier =
        () -> aimTrigger.getAsBoolean() || chargeTrigger.getAsBoolean();

    DemoDashboard.configure(vision, shooter, turret);
    // La gestión térmica por throttle sólo existe en la Limelight 4.
    vision.setThrottleManagementEnabled(DemoConstants.isLimelight4);

    // ── PILOTO ─────────────────────────────────────────────────────────────

    drive.setDefaultCommand(
        DemoDriveCommands.smoothDrive(
            drive,
            vision,
            turret,
            () -> -driverJoystick.getLeftY(),
            () -> -driverJoystick.getLeftX(),
            () -> -driverJoystick.getRightX(),
            () -> !solo && driverJoystick.getLeftTriggerAxis() > 0.5,
            assistSupplier));

    // ── Orientación del manejo ─────────────────────────────────────────────
    //
    // FIELD  = como competencia. Field-relative con la rotación de la odometría
    //          y flip por alianza. La visión mantiene la referencia correcta.
    // DRIVER = el piloto fija su propio frente.
    //
    // En modo solo el toggle se va a Start (y con eso desaparece el ciclado
    // manual de pipelines) y fijar el frente se va a POV izquierda, porque X y
    // B los ocupa el mapeo del operador.
    Trigger orientationToggle = solo ? mech.start() : driverJoystick.x();
    Trigger setFrontButton = solo ? mech.povLeft() : driverJoystick.b();

    orientationToggle.onTrue(
        Commands.runOnce(DemoState::toggleDriveOrientation).ignoringDisable(true));

    // ── Fijar el frente ────────────────────────────────────────────────────
    //
    // El comportamiento depende del modo de orientación, y ésa es la corrección
    // al bug de "presiono B y cuando ve un AprilTag se me mueve el frente":
    //
    //   FIELD  → reset de giroscopio de competencia (reescribe la pose). Tiene
    //            sentido porque en este modo la referencia ES el campo.
    //   DRIVER → guarda un OFFSET sin tocar la odometría. La visión sigue
    //            corrigiendo la pose para el apuntado, pero el frente del
    //            piloto se calcula como (rotación − offset) y no se mueve.
    setFrontButton.onTrue(
        Commands.runOnce(
            () -> {
              if (DemoState.isDriverOriented()) {
                DemoState.captureDriverFront(drive.getRotation());
              } else {
                boolean isRed = DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red;
                drive.setPose(
                    new Pose2d(
                        drive.getPose().getTranslation(),
                        isRed ? Rotation2d.k180deg : Rotation2d.kZero));
              }
            },
            drive)
            .ignoringDisable(true));

    // ── OPERADOR: selección de modo ────────────────────────────────────────
    //
    // Paso 1 del flujo. Seleccionar el modo no mueve nada: sólo le dice al
    // robot qué debe hacer cuando el operador apriete el gatillo.

    mech.y().onTrue(Commands.runOnce(DemoState::toggleMode).ignoringDisable(true));

    mech
        .back()
        .onTrue(
            Commands.runOnce(
                () -> DemoState.toggleSmartFeature(
                    vision.getPrimaryTagId(DemoDriveCommands.TURRET_CAMERA)))
                .ignoringDisable(true));

    // Rotación manual de pipeline. Sólo en modo dos controles: en solo, Start
    // se ocupa del toggle de orientación.
    if (!solo) {
      mech.start()
          .onTrue(
              Commands.runOnce(
                  () -> {
                    int current = vision.getPipelineIndex(DemoDriveCommands.TURRET_CAMERA);
                    int next = (current + 1) % DemoConstants.searchPipelines.length;
                    vision.setPipeline(
                        DemoDriveCommands.TURRET_CAMERA, DemoConstants.searchPipelines[next]);
                  })
                  .ignoringDisable(true));
    }

    // ── Follow-me ("perrito") ──────────────────────────────────────────────
    //
    // Se dispara por estado, no por botón: cuando el operador activa la smart
    // feature de BOMBER, este Trigger toma el drivetrain. Si el piloto toca el
    // stick, el comando se auto-termina y el control vuelve a ser manual; como
    // whileTrue sólo re-programa en flanco de subida, no se vuelve a enganchar
    // solo. El operador tiene que volver a activarlo a propósito.
    // En modo solo la smart feature de BOMBER es el cero odometría, que no
    // necesita comando propio: es el mismo orbit contra un punto capturado.
    new Trigger(DemoState::isFollowing)
        .whileTrue(
            DemoDriveCommands.followTarget(
                drive,
                vision,
                turret,
                () -> -mech.getLeftY(),
                () -> -mech.getLeftX(),
                () -> -mech.getRightX()));

    // ── Apuntado y carga ───────────────────────────────────────────────────
    //
    // Paso 2 del flujo. Ambos comandos leen los dos gatillos: X apunta sin
    // acelerar, RT apunta y además carga.

    turret.setDefaultCommand(
        DemoTurretCommands.demoTurretCmd(
            turret,
            vision,
            drive,
            aimTrigger::getAsBoolean,
            chargeTrigger::getAsBoolean,
            drive::getPose));

    shooter.setDefaultCommand(
        DemoShooterCommands.demoShooterCmd(
            shooter, drive, chargeTrigger::getAsBoolean, drive::getPose));

    // ── Paso 3: alimentar con la Y-valve ───────────────────────────────────
    indexer.setDefaultCommand(
        IndexerCommands.joystickIndexerCmd(
            indexer,
            feedShooterTrigger::getAsBoolean,
            feedTurretTrigger::getAsBoolean));

    // ── Agitación ──────────────────────────────────────────────────────────

    if (DemoConstants.autoAgitateWhileFeeding) {
      // Mientras se alimenta, la caja se sacude sola. Resuelve los puentes de
      // pelotas sin que el operador tenga que acordarse de nada.
      feedShooterTrigger.or(feedTurretTrigger).whileTrue(DemoIntakeCommands.agitate(intake));
    }

    mech.povDown().whileTrue(DemoIntakeCommands.agitate(intake));

    // ── Intake ─────────────────────────────────────────────────────────────

    mech
        .leftTrigger(0.5)
        .whileTrue(Commands.startEnd(intake::intake, intake::stopRollers, intake));

    // Desatascar el indexer. Se movió al D-pad porque RT ahora es el gatillo
    // de carga.
    mech
        .povRight()
        .whileTrue(Commands.runEnd(indexer::outtake, indexer::stopIndexer, indexer));

    // Extender con remate por corriente: va a posición y luego empuja despacio
    // hasta sentir el tope mecánico. Así el intake sale COMPLETO aunque la
    // calibración de posición se haya corrido.
    mech.a().whileTrue(DemoIntakeCommands.extendWithStallHoming(intake));

    mech
        .b()
        .whileTrue(Commands.startEnd(intake::setExtendedReset, intake::stopExtensor, intake));

    // Jog sin soft limits — sólo para recalibrar contra el tope mecánico.
    // Van en los clicks de stick a propósito: son incómodos de presionar por
    // accidente, que es justo lo que quieres de una función que puede dañar el
    // rack si se usa sin cuidado.
    mech
        .rightStick()
        .whileTrue(
            Commands.runEnd(
                () -> {
                  intake.setSoftwareLimit(false);
                  intake.extend();
                },
                () -> {
                  intake.setSoftwareLimit(true);
                  intake.stopExtensor();
                },
                intake));

    mech
        .leftStick()
        .whileTrue(
            Commands.runEnd(
                () -> {
                  intake.setSoftwareLimit(false);
                  intake.retract();
                },
                () -> {
                  intake.setSoftwareLimit(true);
                  intake.stopExtensor();
                },
                intake));

    // ── Prueba de pit ──────────────────────────────────────────────────────

    mech
        .povUp()
        .whileTrue(
            Commands.runEnd(
                () -> {
                  shooter.setFlywheelOpenLoop(0.25);
                  turret.setFlywheelOpenLoop(0.25);
                },
                () -> {
                  shooter.stopFlywheel();
                  turret.stopFlywheel();
                },
                shooter, turret));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // State machine
    driverJoystick.x().onTrue(Commands.runOnce(() -> {
      Drive.mode = Drive.mode != DriveMode.ORBIT ? DriveMode.ORBIT : DriveMode.NORMAL;
      Robot.mode = RobotMode.BOMBER;
      Turret.mode = TurretMode.NORMAL;
      Logger.recordOutput("RobotMode", Robot.mode);
      Logger.recordOutput("DriveMode", Drive.mode);
      Logger.recordOutput("TurretMode", Turret.mode);
    }));
    driverJoystick.y().onTrue(Commands.runOnce(() -> {
      Drive.mode = Drive.mode != DriveMode.FEEDER ? DriveMode.FEEDER : DriveMode.NORMAL;
      Robot.mode = RobotMode.BOMBER;
      Turret.mode = TurretMode.NORMAL;
      Logger.recordOutput("RobotMode", Robot.mode);
      Logger.recordOutput("DriveMode", Drive.mode);
      Logger.recordOutput("TurretMode", Turret.mode);
    }));
    driverJoystick.start().onTrue(Commands.runOnce(() -> {
      Robot.mode = Turret.mode != TurretMode.HUB_TRACKER ? RobotMode.STRIKER : RobotMode.BOMBER;
      Turret.mode = Turret.mode != TurretMode.HUB_TRACKER ? TurretMode.HUB_TRACKER : TurretMode.NORMAL;
      Drive.mode = DriveMode.NORMAL;
      Logger.recordOutput("RobotMode", Robot.mode);
      Logger.recordOutput("DriveMode", Drive.mode);
      Logger.recordOutput("TurretMode", Turret.mode);
    }));
    driverJoystick.back().onTrue(Commands.runOnce(() -> {
      Robot.mode = Turret.mode != TurretMode.DS_TRACKER ? RobotMode.STRIKER : RobotMode.BOMBER;
      Turret.mode = Turret.mode != TurretMode.DS_TRACKER ? TurretMode.DS_TRACKER : TurretMode.NORMAL;
      Drive.mode = DriveMode.NORMAL;
      Logger.recordOutput("RobotMode", Robot.mode);
      Logger.recordOutput("DriveMode", Drive.mode);
      Logger.recordOutput("TurretMode", Turret.mode);
    }));

    // Default drive command
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverJoystick.getLeftY(),
            () -> -driverJoystick.getLeftX(),
            () -> -driverJoystick.getRightX()));

    // Reset gyro to 0° when B button is pressed
    driverJoystick
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  boolean isFlipped = DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
                  drive.setPose(
                      new Pose2d(drive.getPose().getTranslation(),
                          isFlipped ? Rotation2d.k180deg : Rotation2d.kZero));
                },
                drive)
                .ignoringDisable(true));

    turret.setDefaultCommand(
        TurretCommands.joystickTurretCmd(
            turret,
            () -> mechanismsJoystick.x().getAsBoolean(),
            drive::getPose));

    shooter.setDefaultCommand(
        ShooterCommands.joystickShooterCmd(
            shooter, turret,
            () -> mechanismsJoystick.x().getAsBoolean(),
            drive::getPose));

    indexer.setDefaultCommand(
        IndexerCommands.joystickIndexerCmd(
            indexer,
            () -> mechanismsJoystick.leftBumper().getAsBoolean(),
            () -> mechanismsJoystick.rightBumper().getAsBoolean()));

    // Start flywheels at low speed for tests
    mechanismsJoystick.povUp()
        .whileTrue(
            Commands.runEnd(
                () -> {
                  shooter.setFlywheelSpeed(0.25);
                  turret.setFlywheelSpeed(0.25);
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
            Commands.startEnd(
                () -> {
                  intake.intake();
                },
                () -> {
                  intake.stopRollers();
                }, 
                intake));

    // Extend
    mechanismsJoystick.a()
        .whileTrue(
            Commands.startEnd(
                () -> {
                  intake.setExtended();
                },
                () -> {
                  intake.stopExtensor();
                },
                intake));

    // Retract
    mechanismsJoystick.b()
        .whileTrue(
            Commands.startEnd(
                () -> {
                  intake.setExtendedReset();
                },
                () -> {
                  intake.stopExtensor();
                },
                intake));

    // Extend
    mechanismsJoystick.start()
        .whileTrue(
            Commands.runEnd(
                () -> {
                  intake.setSoftwareLimit(false);
                  intake.extend();
                },
                () -> {
                  intake.setSoftwareLimit(true);
                  intake.stopExtensor();
                },
                intake));

    // Retract
    mechanismsJoystick.back()
        .whileTrue(
            Commands.runEnd(
                () -> {
                  intake.setSoftwareLimit(false);
                  intake.retract();
                },
                () -> {
                  intake.setSoftwareLimit(true);
                  intake.stopExtensor();
                },
                intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}