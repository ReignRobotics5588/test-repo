// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * The hardware layer ("IO") for one camera.
 *
 * <p>This is what makes the vision device easy to swap. The {@link Vision} subsystem only ever sees
 * the generic data below, so it does not care which camera produced it. Implementations:
 *
 * <ul>
 *   <li>{@link VisionIOLimelight}: a Limelight (written for Limelight 4) over NetworkTables.
 *   <li>The empty default methods: no camera, used in simulation and replay.
 * </ul>
 *
 * <p>To use a different device (PhotonVision, a different coprocessor, ...), write a new class that
 * implements this interface and fills in {@link VisionIOInputs}, then select it in {@code
 * RobotContainer}. Nothing else changes.
 */
public interface VisionIO {
  /** Everything one camera reports. AdvantageKit logs all of these fields. */
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;

    /** Angle to the best target, for simple "point at the target" aiming. */
    public TargetObservation latestTargetObservation =
        new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);

    /** Robot poses measured from AprilTags since the last loop. */
    public PoseObservation[] poseObservations = new PoseObservation[0];

    /** IDs of every AprilTag seen since the last loop. */
    public int[] tagIds = new int[0];
  }

  /**
   * Where the best target is in the image.
   *
   * @param tx horizontal angle from the center of the image; positive = target is to the right
   * @param ty vertical angle from the center of the image; positive = target is above
   */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /**
   * One robot pose measured from AprilTags.
   *
   * @param timestamp when the image was taken, in seconds (same clock as Timer.getFPGATimestamp)
   * @param pose where the robot is on the field
   * @param ambiguity 0 to 1; how likely the pose is to be "flipped" (only matters for one tag)
   * @param tagCount how many tags were used
   * @param averageTagDistance average distance from the camera to the tags, in meters
   * @param type which algorithm produced the pose
   */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  /** Which algorithm produced a pose. Different algorithms get trusted differently. */
  public static enum PoseObservationType {
    /** Limelight MegaTag 1: solves full pose from the tags alone. */
    MEGATAG_1,
    /** Limelight MegaTag 2: uses the robot's gyro heading; very stable, but no heading output. */
    MEGATAG_2,
    /** PhotonVision multi-tag / single-tag pose. */
    PHOTONVISION
  }

  /** Reads the latest data from the camera into {@code inputs}. */
  public default void updateInputs(VisionIOInputs inputs) {}
}
