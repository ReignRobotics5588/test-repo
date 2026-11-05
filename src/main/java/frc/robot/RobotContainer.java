// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.kCamera0Name;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Creates the subsystems, connects controller buttons to commands, and builds the auto chooser.
 *
 * <p>This is the ONLY place that decides which hardware is used. Each subsystem gets an "IO" object
 * that is either real hardware, a simulation, or an empty placeholder for log replay. Swapping a
 * device (for example a different camera) means changing one line here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive m_drive;
  private final Vision m_vision;

  // Controllers
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> m_autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");

  public RobotContainer() {
    switch (Constants.kCurrentMode) {
      case REAL:
        // Real robot: real hardware.
        m_drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        m_vision =
            new Vision(
                m_drive::addVisionMeasurement,
                new VisionIOLimelight(kCamera0Name, m_drive::getRotation));
        break;

      case SIM:
        // Simulation: physics models, no gyro (heading comes from the wheels), no camera.
        m_drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        m_vision = new Vision(m_drive::addVisionMeasurement, new VisionIO() {});
        break;

      default:
        // Replay: every input comes from the log file, so the IO does nothing.
        m_drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        m_vision = new Vision(m_drive::addVisionMeasurement, new VisionIO() {});
        break;
    }

    configureAutos();
    configureButtonBindings();
  }

  /** Connects controller buttons to commands. */
  private void configureButtonBindings() {
    // Default: field-relative driving. The Xbox Y axis is negative when pushed forward, and the X
    // axis is positive to the right, so both are flipped to match the field (+X forward, +Y left).
    m_drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            m_drive,
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX()));

    // Hold A: drive while facing the target the camera sees.
    m_driverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                m_drive,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () -> m_drive.getRotation().minus(m_vision.getTargetX(0))));

    // Hold X: lock the wheels in an X so the robot is hard to push.
    m_driverController.x().whileTrue(Commands.run(m_drive::stopWithX, m_drive));

    // Press B: reset the heading so the way the robot faces now is "away from the driver".
    m_driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        m_drive.setPose(
                            new Pose2d(m_drive.getPose().getTranslation(), Rotation2d.kZero)),
                    m_drive)
                .ignoringDisable(true));
  }

  /** Sets up BLine path following and fills the auto chooser. */
  private void configureAutos() {
    // Send BLine's internal data to the AdvantageKit log.
    FollowPath.setPoseLoggingConsumer(
        pair -> Logger.recordOutput(pair.getFirst(), pair.getSecond()));
    FollowPath.setTranslationListLoggingConsumer(
        pair -> Logger.recordOutput(pair.getFirst(), pair.getSecond()));
    FollowPath.setBooleanLoggingConsumer(
        pair -> Logger.recordOutput(pair.getFirst(), pair.getSecond()));
    FollowPath.setDoubleLoggingConsumer(
        pair -> Logger.recordOutput(pair.getFirst(), pair.getSecond()));

    // One builder, reused for every path. Paths are drawn for the blue alliance and flipped
    // automatically on red. The pose is reset to the path's start when the path begins.
    FollowPath.Builder pathBuilder =
        new FollowPath.Builder(
                m_drive,
                m_drive::getPose,
                m_drive::getChassisSpeeds,
                m_drive::runVelocity,
                new PIDController(DriveConstants.kPathTranslationKp, 0.0, 0.0),
                new PIDController(DriveConstants.kPathRotationKp, 0.0, 0.0),
                new PIDController(DriveConstants.kPathCrossTrackKp, 0.0, 0.0))
            .withDefaultShouldFlip()
            .withPoseReset(m_drive::setPose);

    // Path files live in src/main/deploy/autos/paths. Edit them with the BLine web editor:
    // https://bline-web.pages.dev/
    m_autoChooser.addDefaultOption("None", Commands.none());
    m_autoChooser.addOption("Example Path", pathBuilder.build(new Path("ExamplePath")));
    m_autoChooser.addOption("Auto Test 1", pathBuilder.build(new Path("phase-1-canvas-draft")));
    m_autoChooser.addOption("Robot go spin", pathBuilder.build(new Path("spinny-robot")));

    // Characterization and tuning routines. Never pick these in a match!
    m_autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(m_drive));
    m_autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(m_drive));
    m_autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /** Returns the autonomous command picked on the dashboard. */
  public Command getAutonomousCommand() {
    return m_autoChooser.get();
  }
}
