// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The swerve drivetrain: four {@link Module}s, a gyro, and a pose estimator that tracks where the
 * robot is on the field.
 *
 * <p>Field coordinates (WPILib "blue alliance origin"):
 *
 * <ul>
 *   <li>(0, 0) is the corner of the field on the BLUE alliance wall, at the right side as seen by
 *       the blue drivers.
 *   <li>X (meters) increases going away from the blue wall, toward the red alliance wall.
 *   <li>Y (meters) increases going to the left, as seen by the blue drivers.
 *   <li>Angle: 0 = facing the red alliance wall. Positive angles turn LEFT (counter-clockwise).
 * </ul>
 */
public class Drive extends SubsystemBase {
  /** Stops the odometry thread from changing samples while the main loop is reading them. */
  static final Lock kOdometryLock = new ReentrantLock();

  private final GyroIO m_gyroIO;
  private final GyroIOInputsAutoLogged m_gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] m_modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine m_sysId;
  private final Alert m_gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using wheel odometry for heading.", AlertType.kError);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(kModuleTranslations);
  private Rotation2d m_rawGyroRotation = Rotation2d.kZero;
  private final SwerveModulePosition[] m_lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  /**
   * Combines wheel odometry with vision measurements. The standard deviations say how much to trust
   * the wheels (x meters, y meters, angle radians). Smaller = trust more.
   */
  private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          m_kinematics,
          m_rawGyroRotation,
          m_lastModulePositions,
          kStartingPose,
          VecBuilder.fill(0.1, 0.1, 0.1),
          VecBuilder.fill(0.9, 0.9, 0.9));

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    m_gyroIO = gyroIO;
    m_modules[0] = new Module(flModuleIO, 0);
    m_modules[1] = new Module(frModuleIO, 1);
    m_modules[2] = new Module(blModuleIO, 2);
    m_modules[3] = new Module(brModuleIO, 3);

    // All signals are registered now, so start sampling them.
    SparkOdometryThread.getInstance().start();

    // System identification routine, used to measure the drive motor feedforward. Run it from the
    // auto chooser; results are in the log under "Drive/SysIdState".
    m_sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    kOdometryLock.lock(); // Keep the odometry thread from changing the samples while we read them
    try {
      m_gyroIO.updateInputs(m_gyroInputs);
      Logger.processInputs("Drive/Gyro", m_gyroInputs);
      for (Module module : m_modules) {
        module.periodic();
      }
    } finally {
      kOdometryLock.unlock();
    }

    // Motors must never move while the robot is disabled.
    if (DriverStation.isDisabled()) {
      for (Module module : m_modules) {
        module.stop();
      }
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    updateOdometry();

    m_gyroDisconnectedAlert.set(!m_gyroInputs.connected && Constants.kCurrentMode != Mode.SIM);
  }

  /** Feeds every odometry sample from the last loop into the pose estimator. */
  private void updateOdometry() {
    double[] sampleTimestamps = m_modules[0].getOdometryTimestamps();
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // How far each wheel moved since the previous sample.
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = m_modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - m_lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        m_lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      if (m_gyroInputs.connected && i < m_gyroInputs.odometryYawPositions.length) {
        // Use the real gyro.
        m_rawGyroRotation = m_gyroInputs.odometryYawPositions[i];
      } else {
        // No gyro (simulation, or it got unplugged): work out the turn from the wheel movements.
        Twist2d twist = m_kinematics.toTwist2d(moduleDeltas);
        m_rawGyroRotation = m_rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      m_poseEstimator.updateWithTime(sampleTimestamps[i], m_rawGyroRotation, modulePositions);
    }
  }

  /**
   * Drives the robot at the given speeds.
   *
   * @param speeds Robot-relative speeds: +X forward, +Y left, +omega counter-clockwise.
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Correct for the robot curving slightly when it drives and turns at the same time.
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.kLoopPeriodSeconds);
    SwerveModuleState[] setpointStates = m_kinematics.toSwerveModuleStates(discreteSpeeds);
    // If any wheel is asked to go faster than it can, slow all wheels down by the same amount.
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, kMaxSpeedMetersPerSec);

    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    for (int i = 0; i < 4; i++) {
      m_modules[i].runSetpoint(setpointStates[i]);
    }

    // runSetpoint() changes the states in place, so this logs the optimized versions.
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Points all wheels forward and drives them at the same voltage, for characterization. */
  public void runCharacterization(double volts) {
    for (Module module : m_modules) {
      module.runCharacterization(volts);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the wheels into an "X" shape, so the robot is hard to push. The
   * wheels go back to normal the next time the robot is asked to move.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = kModuleTranslations[i].getAngle();
    }
    m_kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic (slow ramp) SysId test in the given direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(m_sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic (fast step) SysId test in the given direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(m_sysId.dynamic(direction));
  }

  /** Returns the measured speed and angle of every module. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = m_modules[i].getState();
    }
    return states;
  }

  /** Returns the measured distance and angle of every module. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = m_modules[i].getPosition();
    }
    return positions;
  }

  /** Returns the measured robot-relative speeds. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns each wheel's position in radians, for wheel radius characterization. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = m_modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average wheel speed in radians per second, for feedforward characterization. */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += m_modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns where the robot is on the field. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /** Returns which way the robot is facing on the field. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Tells the robot where it is on the field, for example at the start of an auto. */
  public void setPose(Pose2d pose) {
    m_poseEstimator.resetPosition(m_rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Adds a robot pose measured by a camera.
   *
   * @param visionRobotPoseMeters where the camera thinks the robot is
   * @param timestampSeconds when the image was taken (not when it arrived)
   * @param visionMeasurementStdDevs how much to trust it: x meters, y meters, angle radians
   */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    m_poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the fastest the robot can drive, in meters per second. */
  public double getMaxLinearSpeedMetersPerSec() {
    return kMaxSpeedMetersPerSec;
  }

  /** Returns the fastest the robot can spin, in radians per second. */
  public double getMaxAngularSpeedRadPerSec() {
    return kMaxSpeedMetersPerSec / kDriveBaseRadiusMeters;
  }
}
