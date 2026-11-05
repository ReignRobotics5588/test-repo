// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * The hardware layer ("IO") for the gyro.
 *
 * <p>Implementations:
 *
 * <ul>
 *   <li>{@link GyroIOPigeon2}: a CTRE Pigeon 2.0 on CAN.
 *   <li>The empty default methods: used in simulation and replay. With no gyro connected, {@link
 *       Drive} works out the heading from the wheel movements instead.
 * </ul>
 *
 * <p>To use the Systemcore's built-in IMU after moving to WPILib 2027, add a {@code
 * GyroIOSystemcore} class that implements this interface and select it in {@code RobotContainer}.
 */
public interface GyroIO {
  /** Everything the gyro reads. AdvantageKit logs all of these fields. */
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;

    /** Robot heading. Counter-clockwise (turning left) is positive. */
    public Rotation2d yawPosition = Rotation2d.kZero;

    public double yawVelocityRadPerSec = 0.0;

    // High-frequency samples collected by the odometry thread since the last loop.
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
  }

  /** Reads the latest sensor values into {@code inputs}. */
  public default void updateInputs(GyroIOInputs inputs) {}
}
