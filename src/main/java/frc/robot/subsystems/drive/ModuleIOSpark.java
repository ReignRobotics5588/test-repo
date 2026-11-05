// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.SparkUtil;
import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * One swerve module driven by REV SPARK motor controllers: a SPARK MAX (NEO) or SPARK Flex (Vortex)
 * for the drive motor, and a SPARK MAX (NEO 550) for the turn motor with an absolute encoder
 * plugged into its data port.
 *
 * <p>Closed-loop control runs on the SPARKs themselves (1 kHz), not in this code.
 */
public class ModuleIOSpark implements ModuleIO {
  private final Rotation2d m_zeroRotation;

  private final SparkBase m_driveSpark;
  private final SparkBase m_turnSpark;
  private final RelativeEncoder m_driveEncoder;
  private final AbsoluteEncoder m_turnEncoder;
  private final SparkClosedLoopController m_driveController;
  private final SparkClosedLoopController m_turnController;

  private final Queue<Double> m_timestampQueue;
  private final Queue<Double> m_drivePositionQueue;
  private final Queue<Double> m_turnPositionQueue;

  // Ignore a single dropped CAN frame instead of flashing a "disconnected" alert.
  private final Debouncer m_driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer m_turnConnectedDebounce = new Debouncer(0.5);

  /**
   * Creates the IO for one module.
   *
   * @param module 0 = front left, 1 = front right, 2 = back left, 3 = back right
   */
  public ModuleIOSpark(int module) {
    m_zeroRotation =
        switch (module) {
          case 0 -> kFrontLeftZeroRotation;
          case 1 -> kFrontRightZeroRotation;
          case 2 -> kBackLeftZeroRotation;
          case 3 -> kBackRightZeroRotation;
          default -> throw new IllegalArgumentException("Invalid module index: " + module);
        };
    int driveCanId =
        switch (module) {
          case 0 -> kFrontLeftDriveCanId;
          case 1 -> kFrontRightDriveCanId;
          case 2 -> kBackLeftDriveCanId;
          default -> kBackRightDriveCanId;
        };
    int turnCanId =
        switch (module) {
          case 0 -> kFrontLeftTurnCanId;
          case 1 -> kFrontRightTurnCanId;
          case 2 -> kBackLeftTurnCanId;
          default -> kBackRightTurnCanId;
        };

    m_driveSpark =
        kDriveUsesSparkFlex
            ? new SparkFlex(driveCanId, MotorType.kBrushless)
            : new SparkMax(driveCanId, MotorType.kBrushless);
    m_turnSpark = new SparkMax(turnCanId, MotorType.kBrushless);
    m_driveEncoder = m_driveSpark.getEncoder();
    m_turnEncoder = m_turnSpark.getAbsoluteEncoder();
    m_driveController = m_driveSpark.getClosedLoopController();
    m_turnController = m_turnSpark.getClosedLoopController();

    // ---- Drive motor ----
    SparkBaseConfig driveConfig =
        kDriveUsesSparkFlex ? new SparkFlexConfig() : new SparkMaxConfig();
    driveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(kDriveMotorCurrentLimitAmps)
        .voltageCompensation(12.0);
    driveConfig
        .encoder
        .positionConversionFactor(kDriveEncoderPositionFactor)
        .velocityConversionFactor(kDriveEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    driveConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kDriveKp, 0.0, kDriveKd);
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / kOdometryFrequencyHz))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        m_driveSpark,
        5,
        () ->
            m_driveSpark.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(m_driveSpark, 5, () -> m_driveEncoder.setPosition(0.0));

    // ---- Turn motor ----
    SparkMaxConfig turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(kTurnInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(kTurnMotorCurrentLimitAmps)
        .voltageCompensation(12.0);
    turnConfig
        .absoluteEncoder
        .inverted(kTurnEncoderInverted)
        .positionConversionFactor(kTurnEncoderPositionFactor)
        .velocityConversionFactor(kTurnEncoderVelocityFactor)
        .averageDepth(2);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(kTurnPIDMinInput, kTurnPIDMaxInput)
        .pid(kTurnKp, 0.0, kTurnKd);
    turnConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / kOdometryFrequencyHz))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        m_turnSpark,
        5,
        () ->
            m_turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // ---- High-frequency odometry ----
    m_timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    m_drivePositionQueue =
        SparkOdometryThread.getInstance().registerSignal(m_driveSpark, m_driveEncoder::getPosition);
    m_turnPositionQueue =
        SparkOdometryThread.getInstance().registerSignal(m_turnSpark, m_turnEncoder::getPosition);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Drive motor
    SparkUtil.s_sparkStickyFault = false;
    ifOk(m_driveSpark, m_driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
    ifOk(
        m_driveSpark,
        m_driveEncoder::getVelocity,
        (value) -> inputs.driveVelocityRadPerSec = value);
    ifOk(
        m_driveSpark,
        new DoubleSupplier[] {m_driveSpark::getAppliedOutput, m_driveSpark::getBusVoltage},
        (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
    ifOk(m_driveSpark, m_driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
    inputs.driveConnected = m_driveConnectedDebounce.calculate(!SparkUtil.s_sparkStickyFault);

    // Turn motor
    SparkUtil.s_sparkStickyFault = false;
    ifOk(
        m_turnSpark,
        m_turnEncoder::getPosition,
        (value) -> inputs.turnPosition = new Rotation2d(value).minus(m_zeroRotation));
    ifOk(m_turnSpark, m_turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
    ifOk(
        m_turnSpark,
        new DoubleSupplier[] {m_turnSpark::getAppliedOutput, m_turnSpark::getBusVoltage},
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(m_turnSpark, m_turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = m_turnConnectedDebounce.calculate(!SparkUtil.s_sparkStickyFault);

    // Odometry samples collected since the last loop
    inputs.odometryTimestamps =
        m_timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        m_drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions =
        m_turnPositionQueue.stream()
            .map((Double value) -> new Rotation2d(value).minus(m_zeroRotation))
            .toArray(Rotation2d[]::new);
    m_timestampQueue.clear();
    m_drivePositionQueue.clear();
    m_turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double volts) {
    m_driveSpark.setVoltage(volts);
  }

  @Override
  public void setTurnOpenLoop(double volts) {
    m_turnSpark.setVoltage(volts);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double ffVolts = kDriveKs * Math.signum(velocityRadPerSec) + kDriveKv * velocityRadPerSec;
    m_driveController.setSetpoint(
        velocityRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVolts);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double setpoint =
        MathUtil.inputModulus(
            rotation.plus(m_zeroRotation).getRadians(), kTurnPIDMinInput, kTurnPIDMaxInput);
    m_turnController.setSetpoint(setpoint, ControlType.kPosition);
  }
}
