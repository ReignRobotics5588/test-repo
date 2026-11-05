// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * Every number that describes the swerve drivetrain: size, CAN IDs, gear ratios and PID gains.
 *
 * <p>The defaults describe REV MAXSwerve modules (NEO or Vortex drive motor, NEO 550 turn motor,
 * REV Through Bore absolute encoder plugged into the turn SPARK MAX). If you use different modules,
 * update the gear ratios, wheel size and encoder settings below.
 *
 * <p>Units: all distances are in meters, all angles in radians, and all times in seconds unless the
 * name says otherwise.
 */
public final class DriveConstants {
  private DriveConstants() {}

  // ---------------------------------------------------------------------------------------------
  // Robot size and speed
  // ---------------------------------------------------------------------------------------------

  /** Fastest the robot can drive, in meters per second. Measure this on the real robot. */
  public static final double kMaxSpeedMetersPerSec = 4.8;

  /** Distance between the left and right wheels (center to center). */
  public static final double kTrackWidthMeters = Units.inchesToMeters(26.5);

  /** Distance between the front and back wheels (center to center). */
  public static final double kWheelBaseMeters = Units.inchesToMeters(26.5);

  /** Distance from the center of the robot to the center of any wheel. */
  public static final double kDriveBaseRadiusMeters =
      Math.hypot(kTrackWidthMeters / 2.0, kWheelBaseMeters / 2.0);

  /**
   * Where each module is, relative to the center of the robot. +X is forward, +Y is left. The order
   * is always front left, front right, back left, back right, everywhere in this code.
   */
  public static final Translation2d[] kModuleTranslations =
      new Translation2d[] {
        new Translation2d(kWheelBaseMeters / 2.0, kTrackWidthMeters / 2.0),
        new Translation2d(kWheelBaseMeters / 2.0, -kTrackWidthMeters / 2.0),
        new Translation2d(-kWheelBaseMeters / 2.0, kTrackWidthMeters / 2.0),
        new Translation2d(-kWheelBaseMeters / 2.0, -kTrackWidthMeters / 2.0)
      };

  /** Where the robot thinks it is when the code starts, before vision or auto sets the pose. */
  public static final Pose2d kStartingPose = new Pose2d(2.0, 3.8, Rotation2d.kZero);

  // ---------------------------------------------------------------------------------------------
  // Absolute encoder offsets
  //
  // The angle each module's absolute encoder reads when that wheel points straight forward. For
  // MAXSwerve these come from REV's module calibration tool. Otherwise: point every wheel forward
  // (bevel gears all facing the same side), read "Drive/Module<N>/TurnPosition" in AdvantageScope
  // with these set to zero, and copy the values here.
  // ---------------------------------------------------------------------------------------------
  public static final Rotation2d kFrontLeftZeroRotation = new Rotation2d(-Math.PI / 2.0);
  public static final Rotation2d kFrontRightZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d kBackLeftZeroRotation = new Rotation2d(Math.PI);
  public static final Rotation2d kBackRightZeroRotation = new Rotation2d(Math.PI / 2.0);

  // ---------------------------------------------------------------------------------------------
  // CAN IDs. Keep this list in sync with the robot's wiring diagram / CAN ID spreadsheet.
  // ---------------------------------------------------------------------------------------------
  public static final int kPigeonCanId = 9;

  public static final int kFrontLeftDriveCanId = 1;
  public static final int kBackLeftDriveCanId = 3;
  public static final int kFrontRightDriveCanId = 5;
  public static final int kBackRightDriveCanId = 7;

  public static final int kFrontLeftTurnCanId = 2;
  public static final int kBackLeftTurnCanId = 4;
  public static final int kFrontRightTurnCanId = 6;
  public static final int kBackRightTurnCanId = 8;

  /**
   * Name of the CAN bus the Pigeon is on. An empty string means the controller's built-in bus.
   * Systemcore has several CAN buses, so this is where you will pick one after migrating.
   */
  public static final String kCanBusName = "";

  /**
   * How often odometry samples the encoders and gyro, in Hz. Sampling faster than the 50 Hz main
   * loop makes the pose estimate more accurate during fast turns.
   */
  public static final double kOdometryFrequencyHz = 100.0;

  // ---------------------------------------------------------------------------------------------
  // Drive motor
  // ---------------------------------------------------------------------------------------------

  /** True for a SPARK Flex + NEO Vortex drive motor, false for a SPARK MAX + NEO. */
  public static final boolean kDriveUsesSparkFlex = false;

  public static final int kDriveMotorCurrentLimitAmps = 50;
  public static final double kWheelRadiusMeters = Units.inchesToMeters(1.5);

  /** MAXSwerve with a 14 tooth pinion. Motor rotations per wheel rotation. */
  public static final double kDriveMotorReduction = (45.0 * 22.0) / (14.0 * 15.0);

  public static final DCMotor kDriveGearbox =
      kDriveUsesSparkFlex ? DCMotor.getNeoVortex(1) : DCMotor.getNEO(1);

  /** Converts motor rotations to wheel radians. */
  public static final double kDriveEncoderPositionFactor = 2 * Math.PI / kDriveMotorReduction;

  /** Converts motor RPM to wheel radians per second. */
  public static final double kDriveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / kDriveMotorReduction;

  // Drive velocity PID + feedforward, used on the real robot. Find kS and kV with the
  // "Drive Simple FF Characterization" auto, then tune kP.
  public static final double kDriveKp = 0.0;
  public static final double kDriveKd = 0.0;
  public static final double kDriveKs = 0.0;
  public static final double kDriveKv = 0.1;

  // Drive gains used only in simulation.
  public static final double kDriveSimP = 0.05;
  public static final double kDriveSimD = 0.0;
  public static final double kDriveSimKs = 0.0;
  public static final double kDriveSimKv = 0.0789;

  // ---------------------------------------------------------------------------------------------
  // Turn motor
  // ---------------------------------------------------------------------------------------------
  public static final boolean kTurnInverted = false;
  public static final int kTurnMotorCurrentLimitAmps = 20;

  /** MAXSwerve turn reduction. Motor rotations per module rotation. */
  public static final double kTurnMotorReduction = 9424.0 / 203.0;

  public static final DCMotor kTurnGearbox = DCMotor.getNeo550(1);

  // The absolute encoder is on the module output shaft, so it measures the wheel angle directly.
  public static final boolean kTurnEncoderInverted = true;
  public static final double kTurnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double kTurnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Turn position PID, used on the real robot.
  public static final double kTurnKp = 2.0;
  public static final double kTurnKd = 0.0;

  // Turn gains used only in simulation.
  public static final double kTurnSimP = 8.0;
  public static final double kTurnSimD = 0.0;

  // The SPARK's position PID wraps around at these limits (one full circle).
  public static final double kTurnPIDMinInput = 0;
  public static final double kTurnPIDMaxInput = 2 * Math.PI;

  // ---------------------------------------------------------------------------------------------
  // Autonomous path following (BLine). Tune these on the real robot.
  // ---------------------------------------------------------------------------------------------
  public static final double kPathTranslationKp = 5.0;
  public static final double kPathRotationKp = 3.0;
  public static final double kPathCrossTrackKp = 2.0;
}
