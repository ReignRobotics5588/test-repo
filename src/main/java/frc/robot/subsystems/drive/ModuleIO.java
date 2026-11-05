// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * The hardware layer ("IO") for one swerve module.
 *
 * <p>The {@link Module} class holds the logic, and talks to the hardware only through this
 * interface. Each kind of hardware gets its own implementation:
 *
 * <ul>
 *   <li>{@link ModuleIOSpark}: real SPARK MAX / SPARK Flex motor controllers.
 *   <li>{@link ModuleIOSim}: a physics simulation for testing without a robot.
 *   <li>The empty default methods: used for log replay, where inputs come from the log.
 * </ul>
 *
 * <p>Switching to different motor controllers means writing one new class that implements this
 * interface. No other code has to change.
 */
public interface ModuleIO {
  /** Everything the module reads from its sensors. AdvantageKit logs all of these fields. */
  @AutoLog
  public static class ModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;

    public boolean turnConnected = false;
    public Rotation2d turnPosition = Rotation2d.kZero;
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;

    // High-frequency samples collected by the odometry thread since the last loop.
    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Reads the latest sensor values into {@code inputs}. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Runs the drive motor at the given voltage, without closed-loop control. */
  public default void setDriveOpenLoop(double volts) {}

  /** Runs the turn motor at the given voltage, without closed-loop control. */
  public default void setTurnOpenLoop(double volts) {}

  /** Runs the drive motor at the given wheel speed, in radians per second. */
  public default void setDriveVelocity(double velocityRadPerSec) {}

  /** Turns the module to the given angle. */
  public default void setTurnPosition(Rotation2d rotation) {}
}
