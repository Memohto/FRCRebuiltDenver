// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.DemoConstants;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {
  private final Supplier<Rotation2d> rotationSupplier;
  private final DoubleArrayPublisher orientationPublisher;
  private final DoubleArrayPublisher cameraPosePublisher;
  private final DoublePublisher pipelinePublisher;
  private final DoublePublisher priorityIdPublisher;
  private final DoublePublisher throttlePublisher;
  private final DoublePublisher imuModePublisher;

  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber captureLatencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleSubscriber txncSubscriber;
  private final DoubleSubscriber tvSubscriber;
  private final DoubleSubscriber taSubscriber;
  private final DoubleSubscriber tidSubscriber;
  private final DoubleSubscriber getpipeSubscriber;
  private final DoubleSubscriber heartbeatSubscriber;
  private final DoubleArraySubscriber targetPoseSubscriber;
  private final DoubleArraySubscriber rawFiducialsSubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;

  private int lastPipeline = -1;
  private int lastPriorityId = Integer.MIN_VALUE;
  private int lastThrottle = Integer.MIN_VALUE;

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   */
  public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
    var table = NetworkTableInstance.getDefault().getTable(name);
    this.rotationSupplier = rotationSupplier;
    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    cameraPosePublisher = table.getDoubleArrayTopic("camerapose_robotspace_set").publish();
    pipelinePublisher = table.getDoubleTopic("pipeline").publish();
    priorityIdPublisher = table.getDoubleTopic("priorityid").publish();
    throttlePublisher = table.getDoubleTopic("throttle_set").publish();
    imuModePublisher = table.getDoubleTopic("imumode_set").publish();

    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    captureLatencySubscriber = table.getDoubleTopic("cl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    txncSubscriber = table.getDoubleTopic("txnc").subscribe(0.0);
    tvSubscriber = table.getDoubleTopic("tv").subscribe(0.0);
    taSubscriber = table.getDoubleTopic("ta").subscribe(0.0);
    tidSubscriber = table.getDoubleTopic("tid").subscribe(-1.0);
    getpipeSubscriber = table.getDoubleTopic("getpipe").subscribe(0.0);
    heartbeatSubscriber = table.getDoubleTopic("hb").subscribe(0.0);
    targetPoseSubscriber =
        table.getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[] {});
    rawFiducialsSubscriber =
        table.getDoubleArrayTopic("rawfiducials").subscribe(new double[] {});
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});

    // ── Limelight 4: modo de IMU ──────────────────────────────────────────
    //
    // Sólo aplica a la LL4; en una LL2/2+ esta clave no existe y escribirla no
    // hace nada, pero deja basura en la tabla que confunde al depurar.
    //
    // Si algún día vuelven a la LL4: se fuerza a modo externo (0) porque la IMU
    // interna está atornillada a la cámara, y la cámara a la torreta. Cuando la
    // torreta gira 90°, la IMU cree que el ROBOT giró 90°, y MegaTag2 recibiría
    // un yaw que no corresponde al chasis.
    if (DemoConstants.isLimelight4) {
      imuModePublisher.accept(DemoConstants.limelight4ImuMode);
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update connection status based on whether an update has been seen in the last
    // 250ms
    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    // Update target observation
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    // ── Datos de seguimiento simple (no dependen de la cancha) ─────────────
    boolean targetVisible = tvSubscriber.get() > 0.5;
    inputs.primaryTagId = targetVisible ? (int) tidSubscriber.get() : -1;
    inputs.targetAreaPercent = taSubscriber.get();
    inputs.pipelineIndex = (int) getpipeSubscriber.get();
    inputs.heartbeat = heartbeatSubscriber.get();
    inputs.txNoCrosshairRad = Units.degreesToRadians(txncSubscriber.get());

    // Marca de tiempo NT de la última publicación de tx. Permite distinguir un
    // dato nuevo de la repetición del anterior — importante con la LL2/2+, que
    // a 960x720 va a ~22 FPS mientras el código corre a 50 Hz.
    inputs.sampleTimestamp = txSubscriber.getAtomic().timestamp * 1.0e-6;

    // Lista completa de tags visibles, desde rawfiducials. Cada tag ocupa 7
    // valores: [id, txnc, tync, ta, distToCamera, distToRobot, ambiguity].
    //
    // A diferencia de inputs.tagIds (que sale de MegaTag y necesita el layout
    // oficial del campo), esto funciona con cualquier tag suelto. Es lo que
    // permite saber si están visibles la tag central Y la izquierda del HUB al
    // mismo tiempo, para quedarse con la central.
    double[] rawFiducials = rawFiducialsSubscriber.get();
    int fiducialCount = rawFiducials.length / 7;
    int[] visibleIds = new int[fiducialCount];
    for (int f = 0; f < fiducialCount; f++) {
      visibleIds[f] = (int) rawFiducials[f * 7];
    }
    inputs.visibleTagIds = visibleIds;

    // Latencia total = procesamiento del pipeline + captura del sensor.
    // Es lo que hace posible compensar el retardo de la imagen al apuntar un
    // mecanismo en movimiento.
    inputs.latencySeconds =
        (latencySubscriber.get() + captureLatencySubscriber.get()) / 1000.0;

    double[] targetPose = targetPoseSubscriber.get();
    if (targetVisible && targetPose.length >= 6) {
      inputs.targetPoseRobotSpace =
          new Pose3d(
              targetPose[0],
              targetPose[1],
              targetPose[2],
              new Rotation3d(
                  Units.degreesToRadians(targetPose[3]),
                  Units.degreesToRadians(targetPose[4]),
                  Units.degreesToRadians(targetPose[5])));
    } else {
      inputs.targetPoseRobotSpace = new Pose3d();
    }

    // Update orientation for MegaTag 2
    orientationPublisher.accept(
        new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
    NetworkTableInstance.getDefault()
        .flush(); // Increases network traffic but recommended by Limelight

    // Read new pose observations from NetworkTables
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var rawSample : megatag1Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }
      poseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

              // 3D pose estimate
              parsePose(rawSample.value),

              // Ambiguity, using only the first tag because ambiguity isn't applicable for
              // multitag
              rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,

              // Tag count
              (int) rawSample.value[7],

              // Average tag distance
              rawSample.value[9],

              // Observation type
              PoseObservationType.MEGATAG_1));
    }
    for (var rawSample : megatag2Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }
      poseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

              // 3D pose estimate
              parsePose(rawSample.value),

              // Ambiguity, zeroed because the pose is already disambiguated
              0.0,

              // Tag count
              (int) rawSample.value[7],

              // Average tag distance
              rawSample.value[9],

              // Observation type
              PoseObservationType.MEGATAG_2));
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }

    // hasTarget ahora refleja tv, no la presencia de una solución de pose.
    // Con la cámara en la torreta muchas veces vemos un tag suelto que sirve
    // para apuntar pero que no produce una pose de campo válida.
    inputs.hasTarget = targetVisible;
  }

  @Override
  public void setPipeline(int index) {
    if (index != lastPipeline) {
      pipelinePublisher.accept(index);
      lastPipeline = index;
    }
  }

  /**
   * Publica la pose de cámara. Se llama cada ciclo, ANTES de leer.
   *
   * <p>
   * La documentación de Limelight es explícita para mecanismos móviles: hay que
   * actualizar la pose cada iteración y llamarla <b>antes</b> de
   * {@code robot_orientation_set}, para aprovechar el {@code flush()} interno
   * de esa actualización y que ambos valores lleguen juntos a la cámara.
   *
   * <p>
   * <b>Además, en la web UI de la Limelight la pose de cámara debe quedar en
   * ceros.</b> Si dejas valores ahí, se suman a los que publicamos y todo queda
   * corrido.
   */
  @Override
  public void setRobotToCamera(Transform3d robotToCamera) {
    double sideSign = DemoConstants.limelightInvertSideAxis ? -1.0 : 1.0;
    cameraPosePublisher.accept(
        new double[] {
          robotToCamera.getX(),
          robotToCamera.getY() * sideSign,
          robotToCamera.getZ(),
          Units.radiansToDegrees(robotToCamera.getRotation().getX()),
          Units.radiansToDegrees(robotToCamera.getRotation().getY()),
          Units.radiansToDegrees(robotToCamera.getRotation().getZ()) * sideSign
        });
  }

  @Override
  public void setPriorityTagId(int tagId) {
    if (tagId != lastPriorityId) {
      priorityIdPublisher.accept(tagId);
      lastPriorityId = tagId;
    }
  }

  /** Gestión térmica. Sólo existe en la Limelight 4. */
  @Override
  public void setThrottle(int throttle) {
    if (DemoConstants.isLimelight4 && throttle != lastThrottle) {
      throttlePublisher.accept(throttle);
      lastThrottle = throttle;
    }
  }

  /** Modo de IMU. Sólo existe en la Limelight 4. */
  @Override
  public void setImuMode(int mode) {
    if (DemoConstants.isLimelight4) {
      imuModePublisher.accept(mode);
    }
  }

  /** Parses the 3D pose from a Limelight botpose array. */
  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }
}
