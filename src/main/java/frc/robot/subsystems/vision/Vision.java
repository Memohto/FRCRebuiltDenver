// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DemoConstants;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputsAutoLogged;

import static frc.robot.constants.VisionConstants.*;

import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  /**
   * Compuerta para las actualizaciones de pose.
   *
   * <p>
   * En demos fuera de cancha llevamos AprilTags sueltos. Si se los damos al
   * pose estimator, el código los interpreta como tags del campo oficial y
   * teletransporta la odometría a donde estaría ese tag en un campo imaginario.
   * Con esto en false, la visión sigue sirviendo para apuntar (que es
   * robot-relativo) pero no toca la odometría.
   */
  private BooleanSupplier poseEstimationEnabled = () -> true;

  /** Proveedor de la transformada robot→cámara para cámaras móviles. */
  private Supplier<Transform3d> dynamicCameraTransform = null;
  private int dynamicCameraIndex = 0;

  /** Gestión térmica automática de la Limelight 4. */
  private boolean throttleManagement = false;

  /**
   * Qué tags tienen permiso de corregir la odometría.
   *
   * <p>
   * Sólo se acepta una observación si TODOS los tags que la produjeron pasan el
   * filtro. En demo se restringe a los tags del HUB: están en el layout oficial
   * y en su posición real, así que localizan bien. Un tag suelto en la mano de
   * un alumno teletransportaría la odometría al punto del campo donde ese ID
   * debería estar.
   */
  private java.util.function.IntPredicate trustedTagFilter = id -> true;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  // ══════════════════════════════════════════════════════════════════════════
  // API de seguimiento para el Demo Mode
  //
  // Todo lo de aquí abajo es ROBOT-RELATIVO: no necesita odometría, no necesita
  // que el tag esté en el layout oficial y no necesita saber en qué cancha
  // estamos. Es lo que hace que el robot pueda cazar un AprilTag en el pasillo
  // de la prepa exactamente igual que en un evento.
  // ══════════════════════════════════════════════════════════════════════════

  /** Define cuándo se le permite a la visión corregir la odometría. */
  public void setPoseEstimationEnabled(BooleanSupplier enabled) {
    this.poseEstimationEnabled = enabled;
  }

  /** Restringe qué tags pueden corregir la odometría. */
  public void setTrustedTagFilter(java.util.function.IntPredicate filter) {
    this.trustedTagFilter = filter;
  }

  /** ¿La cámara está viendo un AprilTag ahora mismo? */
  public boolean hasTarget(int cameraIndex) {
    return inputs[cameraIndex].connected && inputs[cameraIndex].hasTarget;
  }

  /** ¿La cámara está respondiendo en NetworkTables? */
  public boolean isConnected(int cameraIndex) {
    return inputs[cameraIndex].connected;
  }

  /** ID del tag primario, o -1 si no hay ninguno. */
  public int getPrimaryTagId(int cameraIndex) {
    return inputs[cameraIndex].primaryTagId;
  }

  /** Ángulo horizontal al target, en radianes. Positivo = target a la derecha. */
  public double getTargetXRad(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx().getRadians();
  }

  /** Pose del tag primario relativa al centro del robot. */
  public Pose3d getTargetPoseRobotSpace(int cameraIndex) {
    return inputs[cameraIndex].targetPoseRobotSpace;
  }

  /**
   * Distancia horizontal al tag primario, en metros. Devuelve 0 si no hay
   * target o si la Limelight aún no resolvió la pose 3D.
   */
  public double getTargetDistanceMeters(int cameraIndex) {
    if (!hasTarget(cameraIndex)) return 0.0;
    Pose3d p = inputs[cameraIndex].targetPoseRobotSpace;
    return Math.hypot(p.getX(), p.getY());
  }

  /** Cambia el pipeline de la cámara indicada. */
  public void setPipeline(int cameraIndex, int pipeline) {
    io[cameraIndex].setPipeline(pipeline);
  }

  /** Pipeline actualmente activo en la cámara indicada. */
  public int getPipelineIndex(int cameraIndex) {
    return inputs[cameraIndex].pipelineIndex;
  }

  /**
   * Latencia total de la imagen en segundos.
   *
   * <p>
   * Es el dato que permite compensar el retardo al apuntar un mecanismo que se
   * mueve. Sin esto, el lazo de seguimiento oscila.
   */
  public double getLatencySeconds(int cameraIndex) {
    return inputs[cameraIndex].latencySeconds;
  }

  /** tx medido desde el centro óptico en vez de la crosshair calibrada. */
  public double getTargetXNoCrosshairRad(int cameraIndex) {
    return inputs[cameraIndex].txNoCrosshairRad;
  }

  /**
   * Marca de tiempo de la última muestra de la cámara.
   *
   * <p>
   * Compárala contra la del ciclo anterior para saber si llegó un frame nuevo o
   * estás releyendo el mismo. Con una cámara a 22 FPS y un robot a 50 Hz, dos de
   * cada tres lecturas son repeticiones.
   */
  public double getSampleTimestamp(int cameraIndex) {
    return inputs[cameraIndex].sampleTimestamp;
  }

  /**
   * IDs de todos los tags visibles ahora mismo.
   *
   * <p>
   * Funciona con tags sueltos, a diferencia de la lista que sale de MegaTag.
   */
  public int[] getVisibleTagIds(int cameraIndex) {
    return inputs[cameraIndex].visibleTagIds;
  }

  /**
   * Fija qué tag debe usar la cámara para tx/ty. -1 = automático.
   *
   * <p>
   * Mantiene el enganche sobre un target concreto aunque aparezca otro tag más
   * grande o más cercano en la imagen.
   */
  public void setPriorityTagId(int cameraIndex, int tagId) {
    io[cameraIndex].setPriorityTagId(tagId);
  }

  /** Activa la gestión térmica automática de la Limelight 4. */
  public void setThrottleManagementEnabled(boolean enabled) {
    this.throttleManagement = enabled;
  }

  /**
   * Publica la transformada robot→cámara. Con la Limelight en la torreta hay
   * que llamarlo cada ciclo con el ángulo actual del mecanismo.
   */
  public void setRobotToCamera(int cameraIndex, Transform3d robotToCamera) {
    io[cameraIndex].setRobotToCamera(robotToCamera);
  }

  /**
   * Registra una cámara cuya transformada robot→cámara NO es constante.
   *
   * <p>
   * Es el caso de la Limelight montada en la torreta: cada vez que el mecanismo
   * gira, la cámara cambia de posición y orientación respecto al robot. Si no se
   * lo decimos a la Limelight, tanto MegaTag como {@code targetpose_robotspace}
   * devuelven resultados calculados con una transformada obsoleta.
   */
  public void setDynamicCameraTransform(int cameraIndex, Supplier<Transform3d> supplier) {
    this.dynamicCameraIndex = cameraIndex;
    this.dynamicCameraTransform = supplier;
  }

  @Override
  public void periodic() {
    // Se publica ANTES de leer, para que el frame que estamos por leer se haya
    // calculado con la pose de cámara más reciente posible.
    if (dynamicCameraTransform != null && dynamicCameraIndex < io.length) {
      io[dynamicCameraIndex].setRobotToCamera(dynamicCameraTransform.get());
    }

    // Gestión térmica de la LL4: consume hasta 12 W y se calienta. En una demo
    // el robot pasa mucho más tiempo deshabilitado en un pasillo que en cancha,
    // así que bajar el framerate mientras no se usa importa de verdad.
    if (throttleManagement) {
      int throttle =
          DriverStation.isEnabled()
              ? DemoConstants.limelight4ThrottleEnabled
              : DemoConstants.limelight4ThrottleDisabled;
      for (VisionIO camera : io) {
        camera.setThrottle(throttle);
      }
    }

    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation. Se bloquea si algún tag de la solución no
        // está en la lista de confiables.
        boolean allTagsTrusted = inputs[cameraIndex].tagIds.length > 0;
        for (int tagId : inputs[cameraIndex].tagIds) {
          if (!trustedTagFilter.test(tagId)) {
            allTagsTrusted = false;
            break;
          }
        }
        if (poseEstimationEnabled.getAsBoolean() && allTagsTrusted) {
          consumer.accept(
              observation.pose().toPose2d(),
              observation.timestamp(),
              VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
          // Marca de tiempo para saber hasta cuándo le podemos seguir creyendo
          // a la odometría sin ver nada.
          frc.robot.util.FieldTracking.notePoseUpdate();
        }
      }

      // Log camera metadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[0]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
