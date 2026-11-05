// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;

/**
 * Samples the drive encoders, turn encoders and gyro on a background thread, faster than the main
 * 50 Hz loop (see {@link DriveConstants#kOdometryFrequencyHz}).
 *
 * <p>Each sample goes into a queue. Every main loop, the IO classes empty their queues into their
 * inputs, and {@link Drive} feeds every sample into the pose estimator. More samples means less
 * error when the robot is turning and driving quickly at the same time.
 *
 * <p>All values in one sample are only saved if every SPARK reported them without an error, so the
 * queues always stay the same length.
 */
public class SparkOdometryThread {
  private final List<SparkBase> m_sparks = new ArrayList<>();
  private final List<DoubleSupplier> m_sparkSignals = new ArrayList<>();
  private final List<DoubleSupplier> m_genericSignals = new ArrayList<>();
  private final List<Queue<Double>> m_sparkQueues = new ArrayList<>();
  private final List<Queue<Double>> m_genericQueues = new ArrayList<>();
  private final List<Queue<Double>> m_timestampQueues = new ArrayList<>();

  private static SparkOdometryThread s_instance = null;
  private final Notifier m_notifier = new Notifier(this::run);

  /** Returns the single shared odometry thread, creating it the first time. */
  public static SparkOdometryThread getInstance() {
    if (s_instance == null) {
      s_instance = new SparkOdometryThread();
    }
    return s_instance;
  }

  private SparkOdometryThread() {
    m_notifier.setName("OdometryThread");
  }

  /** Starts sampling. Call once, after every signal has been registered. */
  public void start() {
    if (!m_timestampQueues.isEmpty()) {
      m_notifier.startPeriodic(1.0 / DriveConstants.kOdometryFrequencyHz);
    }
  }

  /** Registers a value read from a SPARK. Samples are skipped if that SPARK reports an error. */
  public Queue<Double> registerSignal(SparkBase spark, DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.kOdometryLock.lock();
    try {
      m_sparks.add(spark);
      m_sparkSignals.add(signal);
      m_sparkQueues.add(queue);
    } finally {
      Drive.kOdometryLock.unlock();
    }
    return queue;
  }

  /** Registers a value from any other device, such as a gyro. */
  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.kOdometryLock.lock();
    try {
      m_genericSignals.add(signal);
      m_genericQueues.add(queue);
    } finally {
      Drive.kOdometryLock.unlock();
    }
    return queue;
  }

  /** Returns a queue that receives the time (in seconds) of every saved sample. */
  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.kOdometryLock.lock();
    try {
      m_timestampQueues.add(queue);
    } finally {
      Drive.kOdometryLock.unlock();
    }
    return queue;
  }

  private void run() {
    Drive.kOdometryLock.lock();
    try {
      double timestamp = RobotController.getFPGATime() / 1e6;

      // Read every SPARK value first, and only keep the sample if all of them are valid.
      double[] sparkValues = new double[m_sparkSignals.size()];
      boolean isValid = true;
      for (int i = 0; i < m_sparkSignals.size(); i++) {
        sparkValues[i] = m_sparkSignals.get(i).getAsDouble();
        if (m_sparks.get(i).getLastError() != REVLibError.kOk) {
          isValid = false;
        }
      }

      if (isValid) {
        for (int i = 0; i < m_sparkSignals.size(); i++) {
          m_sparkQueues.get(i).offer(sparkValues[i]);
        }
        for (int i = 0; i < m_genericSignals.size(); i++) {
          m_genericQueues.get(i).offer(m_genericSignals.get(i).getAsDouble());
        }
        for (Queue<Double> timestampQueue : m_timestampQueues) {
          timestampQueue.offer(timestamp);
        }
      }
    } finally {
      Drive.kOdometryLock.unlock();
    }
  }
}
