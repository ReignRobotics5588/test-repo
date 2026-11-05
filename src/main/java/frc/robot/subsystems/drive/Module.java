// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.kWheelRadiusMeters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

/**
 * The logic for one swerve module. It turns a desired wheel speed and angle into motor commands,
 * and turns sensor readings into wheel distance and angle. The hardware itself is behind {@link
 * ModuleIO}.
 */
public class Module {
  private static final String[] kModuleNames = {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};

  private final ModuleIO m_io;
  private final ModuleIOInputsAutoLogged m_inputs = new ModuleIOInputsAutoLogged();
  private final int m_index;

  private final Alert m_driveDisconnectedAlert;
  private final Alert m_turnDisconnectedAlert;
  private SwerveModulePosition[] m_odometryPositions = new SwerveModulePosition[] {};

  public Module(ModuleIO io, int index) {
    m_io = io;
    m_index = index;
    m_driveDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on " + kModuleNames[index] + " module.", AlertType.kError);
    m_turnDisconnectedAlert =
        new Alert(
            "Disconnected turn motor on " + kModuleNames[index] + " module.", AlertType.kError);
  }

  /**
   * Reads the sensors and logs them. {@link Drive} calls this while holding the odometry lock, so
   * it is not a normal subsystem periodic method.
   */
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Drive/Module" + m_index, m_inputs);

    // Turn the high-frequency samples into module positions for the pose estimator.
    int sampleCount = m_inputs.odometryTimestamps.length;
    m_odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = m_inputs.odometryDrivePositionsRad[i] * kWheelRadiusMeters;
      Rotation2d angle = m_inputs.odometryTurnPositions[i];
      m_odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    m_driveDisconnectedAlert.set(!m_inputs.driveConnected);
    m_turnDisconnectedAlert.set(!m_inputs.turnConnected);
  }

  /** Drives the module at the given speed and angle. */
  public void runSetpoint(SwerveModuleState state) {
    // Never turn more than 90 degrees: reverse the wheel instead.
    state.optimize(getAngle());
    // Slow the wheel down while it is still turning toward the right angle, so the robot does not
    // lurch sideways.
    state.cosineScale(getAngle());

    m_io.setDriveVelocity(state.speedMetersPerSecond / kWheelRadiusMeters);
    m_io.setTurnPosition(state.angle);
  }

  /** Points the wheel forward and runs the drive motor at a fixed voltage, for characterization. */
  public void runCharacterization(double volts) {
    m_io.setDriveOpenLoop(volts);
    m_io.setTurnPosition(Rotation2d.kZero);
  }

  /** Turns off both motors. */
  public void stop() {
    m_io.setDriveOpenLoop(0.0);
    m_io.setTurnOpenLoop(0.0);
  }

  /** Returns which way the wheel is pointing. */
  public Rotation2d getAngle() {
    return m_inputs.turnPosition;
  }

  /** Returns how far the wheel has rolled, in meters. */
  public double getPositionMeters() {
    return m_inputs.drivePositionRad * kWheelRadiusMeters;
  }

  /** Returns how fast the wheel is rolling, in meters per second. */
  public double getVelocityMetersPerSec() {
    return m_inputs.driveVelocityRadPerSec * kWheelRadiusMeters;
  }

  /** Returns the module distance and angle. */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module speed and angle. */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions sampled by the odometry thread during the last loop. */
  public SwerveModulePosition[] getOdometryPositions() {
    return m_odometryPositions;
  }

  /** Returns the times (in seconds) of the odometry samples from the last loop. */
  public double[] getOdometryTimestamps() {
    return m_inputs.odometryTimestamps;
  }

  /** Returns the wheel position in radians, for wheel radius characterization. */
  public double getWheelRadiusCharacterizationPosition() {
    return m_inputs.drivePositionRad;
  }

  /** Returns the wheel speed in radians per second, for feedforward characterization. */
  public double getFFCharacterizationVelocity() {
    return m_inputs.driveVelocityRadPerSec;
  }
}
