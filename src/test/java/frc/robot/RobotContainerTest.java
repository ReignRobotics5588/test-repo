// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Smoke test: builds every subsystem, command, button binding and auto path in simulation mode and
 * runs the scheduler for a moment. Catches crashes at robot startup (a missing path file, a typo in
 * a CAN config, ...) before they happen on the field.
 */
class RobotContainerTest {
  @BeforeAll
  static void setUp() {
    // Start the simulated hardware layer.
    HAL.initialize(500, 0);
  }

  @Test
  void robotStartsAndRunsWithoutCrashing() {
    new RobotContainer();

    for (int i = 0; i < 50; i++) {
      CommandScheduler.getInstance().run();
    }
  }
}
