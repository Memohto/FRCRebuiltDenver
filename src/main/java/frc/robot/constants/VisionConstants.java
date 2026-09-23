// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String LimelightFixedCamera = "limelight-fixed";
  public static String camera1Name = "camera_1";

  // ══════════════════════════════════════════════════════════════════════════
  // Limelight 4 fija (competencia)
  // ══════════════════════════════════════════════════════════════════════════

  /**
   * Transformada robot → Limelight fija, en el marco del robot.
   *
   * <p>
   * Convención WPILib: +X hacia adelante, +Y hacia la izquierda, +Z hacia
   * arriba; rotación (roll, pitch, yaw) en radianes, pitch NEGATIVO = mirando
   * hacia arriba.
   *
   * <p>
   * <b>La cámara va montada en el cañón fijo, que dispara hacia ATRÁS del
   * robot</b>, así que su yaw es π (mira por donde salen las pelotas). En
   * competencia el código publica esta transformada a la Limelight al arrancar
   * ({@code camerapose_robotspace_set}), así que <b>en la web UI la pose de
   * cámara debe quedar en ceros</b>: si dejas valores ahí se suman a éstos y todo
   * queda corrido.
   *
   * <p>
   * <b>TODO — medir con cinta</b> desde el centro del robot (centro del
   * cuadrado que forman las cuatro ruedas) hasta el lente, y el ángulo de
   * inclinación con un nivel digital. Los valores actuales son una estimación
   * de la posición anterior de la LL2 con el yaw corregido. Cómo verificarlo:
   * con el robot quieto viendo un tag del HUB, {@code Vision/Camera0/RobotPoses}
   * en AdvantageScope debe caer encima de {@code Odometry/Robot}; si la pose de
   * visión aparece corrida hacia un lado fijo, la traslación está mal; si se
   * mueve al girar el robot, el yaw está mal.
   */
  public static Transform3d robotToLimelightFixed =
      new Transform3d(-0.25, 0.0, 0.25, new Rotation3d(0.0, Math.toRadians(-20.0), Math.PI));

  /**
   * Publicar {@link #robotToLimelightFixed} desde el código al arrancar.
   *
   * <p>
   * {@code true} = la constante de arriba manda y la web UI debe estar en ceros.
   * {@code false} = se respeta lo que esté configurado en la web UI (la forma
   * de siempre en Denver).
   */
  public static boolean publishFixedCameraTransform = true;

  /**
   * Invierte el signo del eje Y al publicar la pose de cámara a la Limelight.
   * Algunas versiones de firmware usan "side positive = derecha". Si la pose
   * estimada se corre hacia el lado contrario, pon esto en {@code true}.
   */
  public static boolean limelightInvertSideAxis = false;

  /**
   * ¿Es una Limelight 4?
   *
   * <p>
   * {@code true} = se escriben las claves que sólo existen en la LL4
   * ({@code imumode_set}, {@code throttle_set}) y se gestiona su IMU interna.
   * En una LL2/2+/3 ponlo en false: escribirlas no rompe nada pero deja basura
   * en la tabla.
   */
  public static boolean isLimelight4 = true;

  /**
   * Modo de IMU de la LL4 <b>mientras el robot está deshabilitado</b>.
   *
   * <p>
   * Valores según la API de Limelight: 0 = IMU externa; 1 = IMU externa y
   * siembra la interna; 2 = IMU interna; 3 = interna con convergencia asistida
   * por MegaTag1; 4 = interna con convergencia asistida por la IMU externa.
   *
   * <p>
   * Deshabilitado se usa <b>1</b>: la LL toma el yaw del Pigeon que publicamos
   * en {@code robot_orientation_set} y con él siembra su IMU interna, para que
   * al habilitar ya esté alineada con la odometría.
   */
  public static int limelight4ImuModeDisabled = 1;

  /**
   * Modo de IMU de la LL4 <b>mientras el robot está habilitado</b>.
   *
   * <p>
   * <b>4</b> = IMU interna con el Pigeon como asistencia lenta. La interna tiene
   * menos latencia que el yaw que le mandamos por NetworkTables y hace que
   * MegaTag2 aguante mejor los giros rápidos; el Pigeon evita que derive.
   * Si ves que la pose de visión se va girando sola, cambia a <b>0</b> (sólo
   * Pigeon, el comportamiento de la LL2).
   */
  public static int limelight4ImuModeEnabled = 4;

  /**
   * Modo de IMU único (alias de compatibilidad para el demo). El demo usa la
   * cámara en la torreta, donde la IMU interna gira con el mecanismo y por eso
   * ahí se fuerza a externa (0).
   */
  public static int limelight4ImuMode = 0;

  /**
   * Frames a saltar entre procesados mientras el robot está deshabilitado.
   * Gestión térmica: la LL4 consume hasta 12 W y se calienta en el pit.
   */
  public static int limelight4ThrottleDisabled = 150;

  /** Frames a saltar habilitado. 0 = procesar todos. */
  public static int limelight4ThrottleEnabled = 0;

  // ══════════════════════════════════════════════════════════════════════════
  // Siembra del rumbo por visión
  // ══════════════════════════════════════════════════════════════════════════

  /**
   * Mientras el robot está DESHABILITADO y ve un tag con buena ambigüedad, la
   * pose completa (posición y rumbo) se reescribe desde MegaTag1.
   *
   * <p>
   * MegaTag2 no corrige rotación: confía en el yaw del Pigeon. En cancha eso
   * funciona porque B fija el frente mirando al lado contrario; en el taller,
   * con el robot en cualquier orientación respecto a un tag en la pared, el yaw
   * no tiene relación con el marco del tag y la torreta apunta a un HUB
   * imaginario. Con esto, al habilitar el rumbo ya es coherente con el campo
   * que define el tag. En cancha también ayuda: el piloto ya no depende de
   * presionar B perfectamente alineado.
   */
  public static boolean seedHeadingFromVisionWhileDisabled = true;

  /** Cada cuánto se permite re-sembrar, en segundos. */
  public static double headingSeedPeriodSeconds = 0.5;

  // ══════════════════════════════════════════════════════════════════════════
  // Otras cámaras
  // ══════════════════════════════════════════════════════════════════════════

  // Robot to camera transforms for PhotonVision cameras (a coprocessor camera,
  // e.g. the HBVCAM on an Orange Pi running PhotonVision, would go here).
  public static Transform3d robotToCamera1 =
      new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

  // ══════════════════════════════════════════════════════════════════════════
  // Filtrado de observaciones de pose
  // ══════════════════════════════════════════════════════════════════════════

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
