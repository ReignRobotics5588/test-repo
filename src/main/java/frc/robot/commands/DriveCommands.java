// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** Commands for driving the swerve drive, and for measuring (characterizing) it. */
public final class DriveCommands {
  /** Joystick values smaller than this are treated as zero, so a drifting stick does nothing. */
  private static final double kDeadband = 0.1;

  // Heading controller for joystickDriveAtAngle().
  private static final double kAngleKp = 5.0;
  private static final double kAngleKd = 0.4;
  private static final double kAngleMaxVelocityRadPerSec = 8.0;
  private static final double kAngleMaxAccelerationRadPerSec2 = 20.0;

  // Characterization settings.
  private static final double kFFStartDelaySeconds = 2.0;
  private static final double kFFRampRateVoltsPerSec = 0.1;
  private static final double kWheelRadiusMaxVelocityRadPerSec = 0.25;
  private static final double kWheelRadiusRampRateRadPerSec2 = 0.05;

  private DriveCommands() {}

  /**
   * Turns joystick X/Y into a field direction and speed (0 to 1). Squaring the magnitude gives
   * finer control at low speeds while still reaching full speed at full stick.
   */
  static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), kDeadband);
    if (linearMagnitude == 0.0) {
      return Translation2d.kZero;
    }
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));
    linearMagnitude = Math.min(linearMagnitude * linearMagnitude, 1.0);
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * Field-relative driving: pushing the stick away from you always drives the robot away from your
   * alliance wall, no matter which way the robot is facing.
   *
   * @param xSupplier forward/back, -1 to 1 (forward is positive)
   * @param ySupplier left/right, -1 to 1 (left is positive)
   * @param omegaSupplier rotation, -1 to 1 (counter-clockwise is positive)
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), kDeadband);
          omega = Math.copySign(omega * omega, omega);

          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getDriverRelativeRotation(drive)));
        },
        drive);
  }

  /**
   * Field-relative driving, while the robot automatically turns to face an angle. Useful for aiming
   * at a target while the driver keeps control of where the robot goes.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            kAngleKp,
            0.0,
            kAngleKd,
            new TrapezoidProfile.Constraints(
                kAngleMaxVelocityRadPerSec, kAngleMaxAccelerationRadPerSec2));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getDriverRelativeRotation(drive)));
            },
            drive)
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * The robot heading as seen by the driver. Red drivers stand on the other side of the field, so
   * "away from me" is the opposite direction for them.
   */
  private static Rotation2d getDriverRelativeRotation(Drive drive) {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    return isRed ? drive.getRotation().plus(Rotation2d.kPi) : drive.getRotation();
  }

  /**
   * Measures the drive motor feedforward (kS and kV) by slowly ramping up the voltage. Run it with
   * the robot on the floor and plenty of space in front of it. The results are printed when the
   * command ends; copy them into {@link DriveConstants}.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Point the wheels forward and let them settle.
        Commands.run(() -> drive.runCharacterization(0.0), drive).withTimeout(kFFStartDelaySeconds),
        Commands.runOnce(timer::restart),

        // Ramp up the voltage and record the speed.
        Commands.run(
                () -> {
                  double voltage = timer.get() * kFFRampRateVoltsPerSec;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, fit a line to voltage vs. speed: kS is where it crosses zero speed,
            // and kV is the slope.
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /**
   * Measures the real wheel radius by spinning the robot in place and comparing how far the wheels
   * rolled with how far the gyro says the robot turned. Run it on carpet, since wheels sink in a
   * little. The result is printed when the command ends; copy it into {@link DriveConstants}.
   */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(kWheelRadiusRampRateRadPerSec2);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Spin the robot.
        Commands.sequence(
            Commands.runOnce(() -> limiter.reset(0.0)),
            Commands.run(
                () -> {
                  double speed = limiter.calculate(kWheelRadiusMaxVelocityRadPerSec);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measure.
        Commands.sequence(
            // Wait for the wheels to point the right way.
            Commands.waitSeconds(1.0),
            Commands.runOnce(
                () -> {
                  state.m_positions = drive.getWheelRadiusCharacterizationPositions();
                  state.m_lastAngle = drive.getRotation();
                  state.m_gyroDeltaRad = 0.0;
                }),
            Commands.run(
                    () -> {
                      Rotation2d rotation = drive.getRotation();
                      state.m_gyroDeltaRad +=
                          Math.abs(rotation.minus(state.m_lastAngle).getRadians());
                      state.m_lastAngle = rotation;
                    })
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.m_positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.m_gyroDeltaRad * DriveConstants.kDriveBaseRadiusMeters)
                              / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.m_gyroDeltaRad) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(wheelRadius * 39.37)
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] m_positions = new double[4];
    Rotation2d m_lastAngle = Rotation2d.kZero;
    double m_gyroDeltaRad = 0.0;
  }
}
