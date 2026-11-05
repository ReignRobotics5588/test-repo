// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/**
 * Pretends to be one swerve module when the code runs on a computer instead of a robot.
 *
 * <p>Every loop it reads what the code asked for, uses a physics model of each motor to work out
 * how the wheel would move, and reports that back as if real sensors had measured it. The rest of
 * the code ({@link Module}, {@link Drive}, odometry, autos) cannot tell the difference.
 *
 * <p>The PID loops here run in code at 50 Hz because there is no SPARK to run them. The gains are
 * in {@link DriveConstants} with "Sim" in their names.
 */
public class ModuleIOSim implements ModuleIO {
  private final DCMotorSim m_driveSim;
  private final DCMotorSim m_turnSim;

  private boolean m_driveClosedLoop = false;
  private boolean m_turnClosedLoop = false;
  private final PIDController m_driveController = new PIDController(kDriveSimP, 0, kDriveSimD);
  private final PIDController m_turnController = new PIDController(kTurnSimP, 0, kTurnSimD);
  private double m_driveFFVolts = 0.0;
  private double m_driveAppliedVolts = 0.0;
  private double m_turnAppliedVolts = 0.0;

  public ModuleIOSim() {
    // The numbers 0.025 and 0.004 are the moments of inertia (kg m^2) of the drive and turn
    // mechanisms. They only affect how quickly the simulated wheel speeds up.
    m_driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(kDriveGearbox, 0.025, kDriveMotorReduction),
            kDriveGearbox);
    m_turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(kTurnGearbox, 0.004, kTurnMotorReduction),
            kTurnGearbox);

    // The shortest way to an angle may cross from -180 to +180 degrees.
    m_turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run the closed-loop controllers, if they are in use.
    if (m_driveClosedLoop) {
      m_driveAppliedVolts =
          m_driveFFVolts + m_driveController.calculate(m_driveSim.getAngularVelocityRadPerSec());
    } else {
      m_driveController.reset();
    }
    if (m_turnClosedLoop) {
      m_turnAppliedVolts = m_turnController.calculate(m_turnSim.getAngularPositionRad());
    } else {
      m_turnController.reset();
    }

    // Step the physics forward one loop.
    m_driveSim.setInputVoltage(MathUtil.clamp(m_driveAppliedVolts, -12.0, 12.0));
    m_turnSim.setInputVoltage(MathUtil.clamp(m_turnAppliedVolts, -12.0, 12.0));
    m_driveSim.update(Constants.kLoopPeriodSeconds);
    m_turnSim.update(Constants.kLoopPeriodSeconds);

    // Report the "measurements".
    inputs.driveConnected = true;
    inputs.drivePositionRad = m_driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = m_driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = m_driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(m_driveSim.getCurrentDrawAmps());

    inputs.turnConnected = true;
    inputs.turnPosition = new Rotation2d(m_turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = m_turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = m_turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(m_turnSim.getCurrentDrawAmps());

    // The simulation only produces one odometry sample per loop.
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveOpenLoop(double volts) {
    m_driveClosedLoop = false;
    m_driveAppliedVolts = volts;
  }

  @Override
  public void setTurnOpenLoop(double volts) {
    m_turnClosedLoop = false;
    m_turnAppliedVolts = volts;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    m_driveClosedLoop = true;
    m_driveFFVolts = kDriveSimKs * Math.signum(velocityRadPerSec) + kDriveSimKv * velocityRadPerSec;
    m_driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    m_turnClosedLoop = true;
    m_turnController.setSetpoint(rotation.getRadians());
  }
}
