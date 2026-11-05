// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Robot-wide constants that do not belong to a single subsystem.
 *
 * <p>Subsystem-specific numbers (CAN IDs, gear ratios, PID gains, ...) live next to the subsystem
 * that uses them, for example {@link frc.robot.subsystems.drive.DriveConstants} and {@link
 * frc.robot.subsystems.vision.VisionConstants}. Keeping them close to their code makes it easy to
 * find the right number and to copy a subsystem into another project.
 */
public final class Constants {
  private Constants() {}

  /**
   * Which mode the code runs in when NOT on a real robot. Change this to {@link Mode#REPLAY} to
   * re-run the code against a log file recorded on the robot.
   */
  public static final Mode kSimMode = Mode.SIM;

  /** The mode the code is running in right now. */
  public static final Mode kCurrentMode = RobotBase.isReal() ? Mode.REAL : kSimMode;

  /** How often the main robot loop runs, in seconds. */
  public static final double kLoopPeriodSeconds = 0.02;

  /**
   * The three ways this code can run. Each subsystem picks its hardware ("IO") implementation based
   * on this in {@link RobotContainer}.
   */
  public enum Mode {
    /** Running on a real robot, talking to real motors and sensors. */
    REAL,

    /** Running on a computer with a physics simulation pretending to be the robot. */
    SIM,

    /** Replaying a log file: inputs come from the log instead of from hardware. */
    REPLAY
  }
}
