package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.Notifier;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for devices like the SparkFlex that require polling rather than a
 * blocking thread. A Notifier thread is used to gather samples with consistent timing.
 */
public class SparkFlexOdometryThread {
  private List<DoubleSupplier> signals = new ArrayList<>();
  private List<Queue<Double>> queues = new ArrayList<>();

  private final Notifier notifier;
  private static SparkFlexOdometryThread instance = null;

  public static SparkFlexOdometryThread getInstance() {
    if (instance == null) {
      instance = new SparkFlexOdometryThread();
    }
    return instance;
  }

  private SparkFlexOdometryThread() {
    notifier = new Notifier(this::periodic);
    notifier.setName("SparkFlexOdometryThread");
    notifier.startPeriodic(1.0 / Module.ODOMETRY_FREQUENCY);
  }

  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(100);
    Drive.odometryLock.lock();
    try {
      signals.add(signal);
      queues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  private void periodic() {
    Drive.odometryLock.lock();
    try {
      for (int i = 0; i < signals.size(); i++) {
        queues.get(i).offer(signals.get(i).getAsDouble());
      }
    } finally {
      Drive.odometryLock.unlock();
    }
  }
}
