// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Reads every camera, throws away bad AprilTag measurements, and sends the good ones to the drive's
 * pose estimator.
 *
 * <p>This class does not know or care what kind of camera it is reading. Each camera is a {@link
 * VisionIO}, so swapping a Limelight for another device only changes {@code RobotContainer}.
 */
public class Vision extends SubsystemBase {
  private final VisionConsumer m_consumer;
  private final VisionIO[] m_io;
  private final VisionIOInputsAutoLogged[] m_inputs;
  private final Alert[] m_disconnectedAlerts;

  /**
   * Creates the vision subsystem.
   *
   * @param consumer where to send accepted measurements, normally {@code
   *     drive::addVisionMeasurement}
   * @param io one IO per camera; their order must match {@link
   *     VisionConstants#kCameraStdDevFactors}
   */
  public Vision(VisionConsumer consumer, VisionIO... io) {
    m_consumer = consumer;
    m_io = io;

    m_inputs = new VisionIOInputsAutoLogged[io.length];
    m_disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < io.length; i++) {
      m_inputs[i] = new VisionIOInputsAutoLogged();
      m_disconnectedAlerts[i] =
          new Alert("Vision camera " + i + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the horizontal angle to the best target seen by a camera, for simple aiming. Positive
   * means the target is to the right.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return m_inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < m_io.length; i++) {
      m_io[i].updateInputs(m_inputs[i]);
      Logger.processInputs("Vision/Camera" + i, m_inputs[i]);
    }

    List<Pose3d> allTagPoses = new ArrayList<>();
    List<Pose3d> allRobotPosesAccepted = new ArrayList<>();
    List<Pose3d> allRobotPosesRejected = new ArrayList<>();

    for (int cameraIndex = 0; cameraIndex < m_io.length; cameraIndex++) {
      VisionIOInputsAutoLogged inputs = m_inputs[cameraIndex];
      m_disconnectedAlerts[cameraIndex].set(!inputs.connected);

      // Positions of the tags this camera saw, for display in AdvantageScope.
      List<Pose3d> tagPoses = new ArrayList<>();
      for (int tagId : inputs.tagIds) {
        kAprilTagLayout.getTagPose(tagId).ifPresent(tagPoses::add);
      }

      List<Pose3d> robotPosesAccepted = new ArrayList<>();
      List<Pose3d> robotPosesRejected = new ArrayList<>();
      for (PoseObservation observation : inputs.poseObservations) {
        if (!isObservationValid(observation)) {
          robotPosesRejected.add(observation.pose());
          continue;
        }
        robotPosesAccepted.add(observation.pose());

        m_consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            calculateStdDevs(observation, cameraIndex));
      }

      String prefix = "Vision/Camera" + cameraIndex;
      Logger.recordOutput(prefix + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          prefix + "/RobotPosesAccepted", robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          prefix + "/RobotPosesRejected", robotPosesRejected.toArray(new Pose3d[0]));
      allTagPoses.addAll(tagPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }

  /**
   * Decides whether a pose measurement is believable. Static so it can be unit tested without any
   * hardware.
   */
  static boolean isObservationValid(PoseObservation observation) {
    // No tags means no measurement.
    if (observation.tagCount() == 0) {
      return false;
    }
    // A single tag can look the same from two different places; skip it if that is likely.
    if (observation.tagCount() == 1 && observation.ambiguity() > kMaxAmbiguity) {
      return false;
    }
    // The robot does not fly or dig.
    if (Math.abs(observation.pose().getZ()) > kMaxZErrorMeters) {
      return false;
    }
    // The robot must be on the field.
    return observation.pose().getX() >= 0.0
        && observation.pose().getX() <= kAprilTagLayout.getFieldLength()
        && observation.pose().getY() >= 0.0
        && observation.pose().getY() <= kAprilTagLayout.getFieldWidth();
  }

  /**
   * Works out how much to trust a measurement: less for far-away tags, more when several tags are
   * visible.
   */
  static Matrix<N3, N1> calculateStdDevs(PoseObservation observation, int cameraIndex) {
    double stdDevFactor =
        Math.pow(observation.averageTagDistance(), 2.0) / Math.max(observation.tagCount(), 1);
    double linearStdDev = kLinearStdDevBaselineMeters * stdDevFactor;
    double angularStdDev = kAngularStdDevBaselineRadians * stdDevFactor;
    if (observation.type() == PoseObservationType.MEGATAG_2) {
      linearStdDev *= kLinearStdDevMegatag2Factor;
      angularStdDev *= kAngularStdDevMegatag2Factor;
    }
    if (cameraIndex < kCameraStdDevFactors.length) {
      linearStdDev *= kCameraStdDevFactors[cameraIndex];
      angularStdDev *= kCameraStdDevFactors[cameraIndex];
    }
    return VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
  }

  /** Something that accepts vision measurements, normally the drive's pose estimator. */
  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
