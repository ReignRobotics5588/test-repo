// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The top-level robot class. It sets up logging and then hands everything else to {@link
 * RobotContainer} and the command scheduler.
 *
 * <p>Keep this class small. Robot behavior belongs in subsystems and commands, and wiring (which
 * button runs which command) belongs in {@link RobotContainer}.
 *
 * <p>This extends AdvantageKit's {@link LoggedRobot} instead of TimedRobot, so every sensor input
 * and output is logged automatically. Open the logs in AdvantageScope to see exactly what the robot
 * did in a match, or replay them through the code to debug changes without the robot.
 */
public class Robot extends LoggedRobot {
  private final RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

  /** Called once at the beginning of the robot program. */
  public Robot() {
    // Metadata is saved at the top of every log file.
    Logger.recordMetadata("ProjectName", "example-one");

    switch (Constants.kCurrentMode) {
      case REAL:
        // Save a log file (to a USB stick if one is plugged in) and stream live data to the
        // dashboard over NetworkTables.
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Stream live data so AdvantageScope can show the simulated robot.
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Run as fast as possible, read inputs from a log file, and save the outputs to a new log
        // file with "_sim" added to the name.
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start logging. No more data receivers or replay sources can be added after this.
    Logger.start();

    // Create subsystems, commands and button bindings.
    m_robotContainer = new RobotContainer();
  }

  /** Called every 20 ms in all modes, after the mode-specific periodic method. */
  @Override
  public void robotPeriodic() {
    // Run the scheduler at high priority so garbage collection and other threads are less likely
    // to delay the main loop.
    Threads.setCurrentThreadPriority(true, 99);

    // Polls buttons, schedules new commands, runs running commands, and calls every subsystem's
    // periodic() method. This one line is what makes command-based code work.
    CommandScheduler.getInstance().run();

    Threads.setCurrentThreadPriority(false, 10);
  }

  /** Called once each time the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** Called every 20 ms while disabled. */
  @Override
  public void disabledPeriodic() {}

  /** Called once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** Called every 20 ms during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** Called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    // Stop the autonomous command when teleop starts. Remove this line if you want the auto
    // command to keep running until it is interrupted by another command.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** Called every 20 ms during teleoperated mode. */
  @Override
  public void teleopPeriodic() {}

  /** Called once each time the robot enters test mode. */
  @Override
  public void testInit() {
    // Cancel everything so test mode starts from a clean state.
    CommandScheduler.getInstance().cancelAll();
  }

  /** Called every 20 ms during test mode. */
  @Override
  public void testPeriodic() {}

  /** Called once when the program starts, in simulation only (never on the real robot). */
  @Override
  public void simulationInit() {}

  /** Called every 20 ms in simulation only. */
  @Override
  public void simulationPeriodic() {}
}
