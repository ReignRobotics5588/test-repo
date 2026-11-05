// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Drives the simulated swerve and checks that odometry follows. */
class DriveTest {
  @BeforeAll
  static void setUp() {
    HAL.initialize(500, 0);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  private static Drive createSimDrive() {
    return new Drive(
        new GyroIO() {},
        new ModuleIOSim(),
        new ModuleIOSim(),
        new ModuleIOSim(),
        new ModuleIOSim());
  }

  /** Runs one 20 ms loop: read sensors, then command the drive. */
  private static void step(Drive drive, ChassisSpeeds speeds, int loops) {
    for (int i = 0; i < loops; i++) {
      drive.periodic();
      drive.runVelocity(speeds);
    }
  }

  @Test
  void drivingForwardMovesTheRobotForward() {
    Drive drive = createSimDrive();
    Pose2d start = drive.getPose();

    step(drive, new ChassisSpeeds(1.0, 0.0, 0.0), 100);

    Pose2d end = drive.getPose();
    assertTrue(end.getX() - start.getX() > 0.5, "robot should have driven forward");
    assertEquals(start.getY(), end.getY(), 0.05, "robot should not drift sideways");
  }

  @Test
  void turningLeftIncreasesTheHeading() {
    Drive drive = createSimDrive();
    double startHeading = drive.getRotation().getRadians();

    step(drive, new ChassisSpeeds(0.0, 0.0, 1.0), 50);

    assertTrue(drive.getRotation().getRadians() - startHeading > 0.3);
  }
}
