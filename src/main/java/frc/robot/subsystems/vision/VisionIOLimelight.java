// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.kLimelightImuModeDisabled;
import static frc.robot.subsystems.vision.VisionConstants.kLimelightImuModeEnabled;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/**
 * A Limelight camera, read directly over NetworkTables.
 *
 * <p>This talks to NetworkTables itself instead of using LimelightHelpers, so it reads every frame
 * that arrived since the last loop (not just the newest one) and has no extra dependency.
 *
 * <p>Key names and array layouts come from the Limelight NetworkTables API docs:
 * https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
 */
public class VisionIOLimelight implements VisionIO {
  /** The Limelight counts as disconnected if its heartbeat has not changed for this long. */
  private static final double kDisconnectedTimeoutMs = 250.0;

  // botpose arrays: [x, y, z, roll, pitch, yaw, latency ms, tag count, tag span, avg distance,
  // avg area, then 7 values for each tag: id, txnc, tync, ta, distToCamera, distToRobot,
  // ambiguity]
  private static final int kLatencyIndex = 6;
  private static final int kTagCountIndex = 7;
  private static final int kAvgDistanceIndex = 9;
  private static final int kFirstTagIndex = 11;
  private static final int kValuesPerTag = 7;
  private static final int kAmbiguityOffset = 6;

  private final Supplier<Rotation2d> m_rotationSupplier;
  private final DoubleArrayPublisher m_orientationPublisher;
  private final IntegerPublisher m_imuModePublisher;

  private final DoubleSubscriber m_heartbeatSubscriber;
  private final DoubleSubscriber m_txSubscriber;
  private final DoubleSubscriber m_tySubscriber;
  private final DoubleArraySubscriber m_megatag1Subscriber;
  private final DoubleArraySubscriber m_megatag2Subscriber;

  /**
   * Creates a new Limelight IO.
   *
   * @param name the Limelight's name (its NetworkTables table), e.g. "limelight"
   * @param rotationSupplier the robot's current heading, which MegaTag 2 needs
   */
  public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
    m_rotationSupplier = rotationSupplier;
    m_orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    m_imuModePublisher = table.getIntegerTopic("imumode_set").publish();
    m_heartbeatSubscriber = table.getDoubleTopic("hb").subscribe(0.0);
    m_txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    m_tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    m_megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    m_megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected =
        ((RobotController.getFPGATime() - m_heartbeatSubscriber.getLastChange()) / 1000.0)
            < kDisconnectedTimeoutMs;

    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(m_txSubscriber.get()),
            Rotation2d.fromDegrees(m_tySubscriber.get()));

    // Send the robot heading to the Limelight for MegaTag 2, and pick the Limelight 4 IMU mode.
    m_orientationPublisher.accept(
        new double[] {m_rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
    m_imuModePublisher.set(
        DriverStation.isDisabled() ? kLimelightImuModeDisabled : kLimelightImuModeEnabled);
    // Send now instead of waiting for the next automatic update, so the heading is not stale.
    NetworkTableInstance.getDefault().flush();

    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new ArrayList<>();
    for (TimestampedDoubleArray sample : m_megatag1Subscriber.readQueue()) {
      addObservation(sample, PoseObservationType.MEGATAG_1, tagIds, poseObservations);
    }
    for (TimestampedDoubleArray sample : m_megatag2Subscriber.readQueue()) {
      addObservation(sample, PoseObservationType.MEGATAG_2, tagIds, poseObservations);
    }

    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
    inputs.tagIds = tagIds.stream().mapToInt(Integer::intValue).toArray();
  }

  /** Turns one botpose array from NetworkTables into a {@link PoseObservation}. */
  private static void addObservation(
      TimestampedDoubleArray sample,
      PoseObservationType type,
      Set<Integer> tagIds,
      List<PoseObservation> poseObservations) {
    double[] values = sample.value;
    if (values.length < kFirstTagIndex) {
      return;
    }

    for (int i = kFirstTagIndex; i + kValuesPerTag <= values.length; i += kValuesPerTag) {
      tagIds.add((int) values[i]);
    }

    poseObservations.add(
        new PoseObservation(
            // The NT timestamp is when the data arrived; subtract the latency to get when the
            // image was actually taken.
            sample.timestamp * 1.0e-6 - values[kLatencyIndex] * 1.0e-3,
            parsePose(values),
            // Ambiguity of the first tag. Only used to filter single-tag measurements.
            values.length >= kFirstTagIndex + kValuesPerTag
                ? values[kFirstTagIndex + kAmbiguityOffset]
                : 0.0,
            (int) values[kTagCountIndex],
            values[kAvgDistanceIndex],
            type));
  }

  /** Reads x, y, z (meters) and roll, pitch, yaw (degrees) from the start of a botpose array. */
  private static Pose3d parsePose(double[] values) {
    return new Pose3d(
        values[0],
        values[1],
        values[2],
        new Rotation3d(
            Units.degreesToRadians(values[3]),
            Units.degreesToRadians(values[4]),
            Units.degreesToRadians(values[5])));
  }
}
