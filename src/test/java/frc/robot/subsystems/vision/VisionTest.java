// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import org.junit.jupiter.api.Test;

class VisionTest {
  private static PoseObservation observation(
      double x, double y, double z, double ambiguity, int tagCount, PoseObservationType type) {
    return new PoseObservation(
        0.0, new Pose3d(x, y, z, Rotation3d.kZero), ambiguity, tagCount, 2.0, type);
  }

  @Test
  void acceptsAGoodMultiTagPose() {
    assertTrue(
        Vision.isObservationValid(
            observation(3.0, 3.0, 0.0, 0.9, 2, PoseObservationType.MEGATAG_1)));
  }

  @Test
  void rejectsPosesWithNoTags() {
    assertFalse(
        Vision.isObservationValid(
            observation(3.0, 3.0, 0.0, 0.0, 0, PoseObservationType.MEGATAG_2)));
  }

  @Test
  void rejectsAmbiguousSingleTagPoses() {
    assertFalse(
        Vision.isObservationValid(
            observation(3.0, 3.0, 0.0, 0.9, 1, PoseObservationType.MEGATAG_1)));
  }

  @Test
  void rejectsPosesOffTheFieldOrInTheAir() {
    assertFalse(
        Vision.isObservationValid(
            observation(-1.0, 3.0, 0.0, 0.0, 2, PoseObservationType.MEGATAG_2)));
    assertFalse(
        Vision.isObservationValid(
            observation(3.0, 3.0, 2.0, 0.0, 2, PoseObservationType.MEGATAG_2)));
  }

  @Test
  void megatag2NeverCorrectsHeading() {
    Matrix<N3, N1> stdDevs =
        Vision.calculateStdDevs(
            observation(3.0, 3.0, 0.0, 0.0, 2, PoseObservationType.MEGATAG_2), 0);
    assertEquals(Double.POSITIVE_INFINITY, stdDevs.get(2, 0));
  }

  @Test
  void moreTagsMeansMoreTrust() {
    double oneTag =
        Vision.calculateStdDevs(
                observation(3.0, 3.0, 0.0, 0.0, 1, PoseObservationType.MEGATAG_1), 0)
            .get(0, 0);
    double threeTags =
        Vision.calculateStdDevs(
                observation(3.0, 3.0, 0.0, 0.0, 3, PoseObservationType.MEGATAG_1), 0)
            .get(0, 0);
    assertTrue(threeTags < oneTag);
  }
}
