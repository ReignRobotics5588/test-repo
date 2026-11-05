// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class DriveCommandsTest {
  private static final double kEpsilon = 1e-9;

  @Test
  void smallStickMovementsAreIgnored() {
    Translation2d result = DriveCommands.getLinearVelocityFromJoysticks(0.05, -0.05);
    assertEquals(0.0, result.getNorm(), kEpsilon);
  }

  @Test
  void fullStickGivesFullSpeedInTheSameDirection() {
    Translation2d result = DriveCommands.getLinearVelocityFromJoysticks(0.0, 1.0);
    assertEquals(0.0, result.getX(), kEpsilon);
    assertEquals(1.0, result.getY(), kEpsilon);
  }

  @Test
  void diagonalStickNeverGoesFasterThanFullSpeed() {
    Translation2d result = DriveCommands.getLinearVelocityFromJoysticks(1.0, 1.0);
    assertEquals(1.0, result.getNorm(), kEpsilon);
  }
}
