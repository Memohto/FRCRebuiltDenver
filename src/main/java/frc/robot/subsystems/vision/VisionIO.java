// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface VisionIO {
  public static class VisionIOInputs {
    public boolean connected = false;
    public TargetObservation latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
    public boolean hasTarget = false;

    // ── Añadido para el Demo Mode (cámara en torreta) ────────────────────

    /** ID del AprilTag primario que la cámara está viendo, o -1 si ninguno. */
    public int primaryTagId = -1;

    /**
     * Pose del tag primario relativa al CENTRO DEL ROBOT.
     *
     * <p>
     * La Limelight calcula esto usando la pose de cámara que le publicamos cada
     * ciclo, así que ya viene compensada por el ángulo de la torreta. Es el dato
     * que usa el follow-me: contiene distancia, rumbo y hacia dónde mira el tag,
     * todo en el marco del robot y sin depender de la odometría.
     */
    public Pose3d targetPoseRobotSpace = new Pose3d();

    /** Área del target en % de la imagen. Sirve como proxy de cercanía. */
    public double targetAreaPercent = 0.0;

    /** Pipeline activo en la Limelight. */
    public int pipelineIndex = 0;

    /**
     * Latencia total del pipeline, en segundos (captura + procesamiento).
     *
     * <p>
     * Sin este número no se puede compensar el retardo de la imagen, y sin esa
     * compensación cualquier lazo de seguimiento sobre un mecanismo que se
     * mueve va a oscilar.
     */
    public double latencySeconds = 0.0;

    /** Heartbeat de la cámara. Incrementa cada frame; sirve para detectar cuelgues. */
    public double heartbeat = 0.0;

    /** tx medido desde el centro óptico y no desde la crosshair calibrada. */
    public double txNoCrosshairRad = 0.0;

    /**
     * Marca de tiempo de la última muestra de la cámara, en segundos.
     *
     * <p>
     * Sirve para saber si el dato que estamos leyendo es <b>nuevo</b> o es el
     * mismo de hace un ciclo. Importa mucho con cámaras lentas: una Limelight
     * 2/2+ a 960x720 corre a ~22 FPS, así que el código (que corre a 50 Hz) lee
     * la misma medición dos o tres veces seguidas. Si el rastreador la vuelve a
     * procesar cada vez, la compensación de latencia se aplica repetidamente
     * sobre un ángulo de torreta que sí cambió, y el estimado se va derivando.
     */
    public double sampleTimestamp = 0.0;

    /**
     * IDs de TODOS los tags que la cámara está viendo ahora mismo.
     *
     * <p>
     * Distinto de {@code tagIds}, que sale de la solución de MegaTag y por lo
     * tanto sólo existe si los tags están en el layout oficial del campo. Éste
     * sale de {@code rawfiducials} y funciona con cualquier tag suelto.
     *
     * <p>
     * Es lo que permite decidir entre la tag central y la izquierda del HUB
     * cuando ambas están a la vista.
     */
    public int[] visibleTagIds = new int[0];
  }

  class VisionIOInputsAutoLogged extends VisionIOInputs implements LoggableInputs {
    @Override
    public void toLog(LogTable table) {
      table.put("connected", connected);
      table.put("latestTargetObservation", latestTargetObservation);
      table.put("poseObservations", poseObservations);
      table.put("tagIds", tagIds);
      table.put("hasTarget", hasTarget);
      table.put("primaryTagId", primaryTagId);
      table.put("targetPoseRobotSpace", targetPoseRobotSpace);
      table.put("targetAreaPercent", targetAreaPercent);
      table.put("pipelineIndex", pipelineIndex);
      table.put("latencySeconds", latencySeconds);
      table.put("heartbeat", heartbeat);
      table.put("txNoCrosshairRad", txNoCrosshairRad);
      table.put("sampleTimestamp", sampleTimestamp);
      table.put("visibleTagIds", visibleTagIds);
    }

    @Override
    public void fromLog(LogTable table) {
      connected = table.get("connected", connected);
      latestTargetObservation = table.get("latestTargetObservation", latestTargetObservation);
      poseObservations = table.get("poseObservations", poseObservations);
      tagIds = table.get("tagIds", tagIds);
      hasTarget = table.get("hasTarget", hasTarget);
      primaryTagId = table.get("primaryTagId", primaryTagId);
      targetPoseRobotSpace = table.get("targetPoseRobotSpace", targetPoseRobotSpace);
      targetAreaPercent = table.get("targetAreaPercent", targetAreaPercent);
      pipelineIndex = table.get("pipelineIndex", pipelineIndex);
      latencySeconds = table.get("latencySeconds", latencySeconds);
      heartbeat = table.get("heartbeat", heartbeat);
      txNoCrosshairRad = table.get("txNoCrosshairRad", txNoCrosshairRad);
      sampleTimestamp = table.get("sampleTimestamp", sampleTimestamp);
      visibleTagIds = table.get("visibleTagIds", visibleTagIds);
    }
  }

  /** Represents the angle to a simple target, not used for pose estimation. */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  public static enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  /**
   * Cambia el pipeline de visión.
   *
   * <p>
   * Se usa para adaptarse a iluminación cambiante: configura varios pipelines
   * con exposiciones distintas en la web UI de la Limelight y el código los
   * rota mientras busca sin encontrar nada.
   */
  public default void setPipeline(int index) {}

  /**
   * Publica la pose de la cámara relativa al robot.
   *
   * <p>
   * Indispensable con la cámara montada en la torreta: la transformada deja de
   * ser constante y hay que actualizarla cada ciclo con el ángulo actual, o
   * MegaTag y {@code targetpose_robotspace} devuelven basura.
   */
  public default void setRobotToCamera(Transform3d robotToCamera) {}

  /**
   * Fija qué AprilTag debe usar la cámara para calcular tx/ty.
   *
   * <p>
   * Sin esto, la Limelight siempre reporta el tag más grande de la imagen. Con
   * esto, el enganche sobre un target concreto se mantiene aunque aparezca otro
   * tag más cercano. Pasa -1 para volver al comportamiento automático.
   */
  public default void setPriorityTagId(int tagId) {}

  /**
   * Throttle de procesamiento (Limelight 4).
   *
   * <p>
   * Valores altos reducen el framerate y con eso el consumo y la temperatura.
   * 0 = máximo rendimiento.
   */
  public default void setThrottle(int throttle) {}

  /**
   * Modo de IMU (Limelight 4).
   *
   * <p>
   * <b>Con la cámara en una torreta esto tiene que ser 0.</b> La IMU interna
   * gira con el mecanismo, así que en cualquier otro modo le estaría reportando
   * a MegaTag2 el yaw de la torreta como si fuera el del robot.
   */
  public default void setImuMode(int mode) {}
}
