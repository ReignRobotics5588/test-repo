// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

/** Settings for the cameras and for how much to trust what they report. */
public final class VisionConstants {
  private VisionConstants() {}

  /** Where every AprilTag is on this year's field. */
  public static final AprilTagFieldLayout kAprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  /**
   * Camera names. For a Limelight this is its hostname / NetworkTables table name, set in the
   * Limelight web interface. Index 0 is camera 0 in the logs.
   */
  public static final String kCamera0Name = "limelight";

  // ---------------------------------------------------------------------------------------------
  // Limelight 4 built-in IMU. See "imumode_set" in the Limelight NetworkTables docs.
  //   0 = use the robot gyro only (works on every Limelight)
  //   1 = use the robot gyro, and keep resetting ("seeding") the Limelight's IMU to match it
  //   2 = use the Limelight's IMU only
  //   3 = Limelight IMU, corrected by MegaTag 1
  //   4 = Limelight IMU, corrected by the robot gyro
  // Seeding while disabled and then using the fast internal IMU while enabled is the setup
  // Limelight recommends for MegaTag 2 on a Limelight 4. Set both to 0 for an older Limelight.
  // ---------------------------------------------------------------------------------------------
  public static final int kLimelightImuModeDisabled = 1;
  public static final int kLimelightImuModeEnabled = 4;

  // ---------------------------------------------------------------------------------------------
  // Filtering: pose measurements that fail these checks are thrown away.
  // ---------------------------------------------------------------------------------------------

  /** Single-tag poses with more ambiguity than this are rejected. */
  public static final double kMaxAmbiguity = 0.3;

  /** The robot is on the floor, so a pose higher or lower than this (meters) is wrong. */
  public static final double kMaxZErrorMeters = 0.75;

  // ---------------------------------------------------------------------------------------------
  // Trust: standard deviations for a measurement 1 meter away from 1 tag. They grow with distance
  // squared and shrink with more tags. Smaller numbers = trust vision more.
  // ---------------------------------------------------------------------------------------------
  public static final double kLinearStdDevBaselineMeters = 0.02;
  public static final double kAngularStdDevBaselineRadians = 0.06;

  /**
   * Extra multiplier for each camera, in the same order as the cameras. Raise it for a camera that
   * is less accurate (lower resolution, worse mounting, ...).
   */
  public static final double[] kCameraStdDevFactors = new double[] {1.0};

  // MegaTag 2 is more stable than MegaTag 1, but gets its heading from the gyro, so it must never
  // be used to correct the heading.
  public static final double kLinearStdDevMegatag2Factor = 0.5;
  public static final double kAngularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;
}
