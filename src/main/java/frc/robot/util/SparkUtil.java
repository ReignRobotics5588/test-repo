// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** Helpers for reading and configuring REV SPARK motor controllers safely. */
public final class SparkUtil {
  private SparkUtil() {}

  /**
   * Set to true whenever a SPARK read fails. IO classes reset it to false before reading, then use
   * it to decide whether the motor controller is still connected.
   */
  public static boolean s_sparkStickyFault = false;

  /** Reads a value and passes it to {@code consumer}, but only if the SPARK reported no error. */
  public static void ifOk(SparkBase spark, DoubleSupplier supplier, DoubleConsumer consumer) {
    double value = supplier.getAsDouble();
    if (spark.getLastError() == REVLibError.kOk) {
      consumer.accept(value);
    } else {
      s_sparkStickyFault = true;
    }
  }

  /** Reads several values and passes them to {@code consumer}, but only if all were OK. */
  public static void ifOk(
      SparkBase spark, DoubleSupplier[] suppliers, Consumer<double[]> consumer) {
    double[] values = new double[suppliers.length];
    for (int i = 0; i < suppliers.length; i++) {
      values[i] = suppliers[i].getAsDouble();
      if (spark.getLastError() != REVLibError.kOk) {
        s_sparkStickyFault = true;
        return;
      }
    }
    consumer.accept(values);
  }

  /**
   * Runs a configuration command until it succeeds, up to {@code maxAttempts} times. CAN messages
   * occasionally get lost at startup, so a single failed attempt should not leave a motor
   * misconfigured.
   */
  public static void tryUntilOk(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
    for (int i = 0; i < maxAttempts; i++) {
      if (command.get() == REVLibError.kOk) {
        break;
      } else {
        s_sparkStickyFault = true;
      }
    }
  }
}
