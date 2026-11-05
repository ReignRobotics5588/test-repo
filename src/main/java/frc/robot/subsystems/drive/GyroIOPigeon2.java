// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.kCanBusName;
import static frc.robot.subsystems.drive.DriveConstants.kOdometryFrequencyHz;
import static frc.robot.subsystems.drive.DriveConstants.kPigeonCanId;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Queue;

/** A CTRE Pigeon 2.0 gyro on CAN. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 m_pigeon = new Pigeon2(kPigeonCanId, new CANBus(kCanBusName));
  private final StatusSignal<Angle> m_yaw = m_pigeon.getYaw();
  private final StatusSignal<AngularVelocity> m_yawVelocity = m_pigeon.getAngularVelocityZWorld();

  // A separate copy of the yaw signal for the odometry thread, so the two threads never share one
  // StatusSignal object.
  private final StatusSignal<Angle> m_yawForOdometry = m_yaw.clone();
  private final Queue<Double> m_yawTimestampQueue;
  private final Queue<Double> m_yawPositionQueue;

  public GyroIOPigeon2() {
    m_pigeon.getConfigurator().apply(new Pigeon2Configuration());
    m_pigeon.getConfigurator().setYaw(0.0);
    m_yaw.setUpdateFrequency(kOdometryFrequencyHz);
    m_yawVelocity.setUpdateFrequency(50.0);
    // Turn off every signal we did not ask for, to save CAN bandwidth.
    m_pigeon.optimizeBusUtilization();

    m_yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    m_yawPositionQueue =
        SparkOdometryThread.getInstance()
            .registerSignal(() -> m_yawForOdometry.refresh().getValueAsDouble());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(m_yaw, m_yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(m_yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(m_yawVelocity.getValueAsDouble());

    inputs.odometryYawTimestamps =
        m_yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        m_yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    m_yawTimestampQueue.clear();
    m_yawPositionQueue.clear();
  }
}
