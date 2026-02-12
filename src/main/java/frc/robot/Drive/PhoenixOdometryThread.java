// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.Drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Drive.Generated.TunerConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for Phoenix 6 devices on both the RIO and CANivore buses. When using
 * a CANivore, the thread uses the "waitForAll" blocking method to enable more consistent sampling.
 * This also allows Phoenix Pro users to benefit from lower latency between devices using CANivore
 * time synchronization.
 */
public class PhoenixOdometryThread extends Thread {
  private final Lock signalsLock =
      new ReentrantLock(); // Prevents conflicts when registering signals
  private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
  private final List<DoubleSupplier> genericSignals = new ArrayList<>();
  private final List<Queue<Double>> phoenixQueues = new ArrayList<>();
  private final List<Queue<Double>> genericQueues = new ArrayList<>();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();

  private static final boolean IS_CAN_FD = TunerConstants.kCANBus.isNetworkFD();
  private static PhoenixOdometryThread instance = null;

  public static PhoenixOdometryThread getInstance() {
    if (instance == null) {
      instance = new PhoenixOdometryThread();
    }
    return instance;
  }

  private PhoenixOdometryThread() {
    setName("PhoenixOdometryThread");
    setDaemon(true);
  }

  @Override
  public void start() {
    if (!timestampQueues.isEmpty()) {
      super.start();
    }
  }

  /** Registers a Phoenix signal to be read from the thread. */
  public Queue<Double> registerSignal(StatusSignal<Angle> signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    signalsLock.lock();
    Drive.odometryLock.lock();
    try {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixSignals.length + 1];
      System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.length);
      newSignals[phoenixSignals.length] = signal;
      phoenixSignals = newSignals;
      phoenixQueues.add(queue);
    } finally {
      signalsLock.unlock();
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  /** Registers a generic signal to be read from the thread. */
  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    signalsLock.lock();
    Drive.odometryLock.lock();
    try {
      genericSignals.add(signal);
      genericQueues.add(queue);
    } finally {
      signalsLock.unlock();
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  @Override
  public void run() {
    final double updatePeriodSeconds = 1.0 / Drive.ODOMETRY_FREQUENCY;
    final long updatePeriodNanoseconds = (long) (updatePeriodSeconds * 1_000_000_000L);

    while (!Thread.currentThread().isInterrupted()) {

      // Snapshot para no dormir/bloquear con signalsLock tomado
      final BaseStatusSignal[] phoenixSignalsSnapshot;
      final List<DoubleSupplier> genericSignalsSnapshot;

      signalsLock.lock();
      try {
        phoenixSignalsSnapshot = phoenixSignals.clone();
        genericSignalsSnapshot = new ArrayList<>(genericSignals);
      } finally {
        signalsLock.unlock();
      }

      try {
        if (IS_CAN_FD && phoenixSignalsSnapshot.length > 0) {
          BaseStatusSignal.waitForAll(2.0 / Drive.ODOMETRY_FREQUENCY, phoenixSignalsSnapshot);
        } else {
          // Evitar dormir con locks tomados (ya no tenemos signalsLock aquí)
          java.util.concurrent.TimeUnit.NANOSECONDS.sleep(updatePeriodNanoseconds);

          if (phoenixSignalsSnapshot.length > 0) {
            BaseStatusSignal.refreshAll(phoenixSignalsSnapshot);
          }
        }
      } catch (InterruptedException interruptedException) {
        Thread.currentThread().interrupt();

        // DS console (útil en match)
        DriverStation.reportWarning(
            "Odometry thread interrupted; stopping thread.", false);

        // AdvantageKit log (útil en postmortem)
        Logger.recordOutput("Odometry/ThreadInterrupted", true);
        Logger.recordOutput("Odometry/ThreadInterruptedMessage", interruptedException.getMessage());

        return;
      } catch (Exception unexpectedException) {
        // Esto NO es para spamear; es para capturar fallos reales de runtime
        DriverStation.reportError(
            "Unexpected exception in odometry thread: " + unexpectedException.getMessage(),
            unexpectedException.getStackTrace());

        Logger.recordOutput("Odometry/ThreadException", true);
        Logger.recordOutput("Odometry/ThreadExceptionMessage", unexpectedException.getMessage());

        // Decide si quieres matar el thread o seguir:
        return;
      }

      Drive.odometryLock.lock();
      try {
        double timestampSeconds = RobotController.getFPGATime() / 1e6;

        double totalLatencySeconds = 0.0;
        for (var signal : phoenixSignalsSnapshot) {
          totalLatencySeconds += signal.getTimestamp().getLatency();
        }
        if (phoenixSignalsSnapshot.length > 0) {
          timestampSeconds -= totalLatencySeconds / phoenixSignalsSnapshot.length;
        }

        for (int signalIndex = 0; signalIndex < phoenixSignalsSnapshot.length; signalIndex++) {
          phoenixQueues.get(signalIndex).offer(phoenixSignalsSnapshot[signalIndex].getValueAsDouble());
        }

        for (int signalIndex = 0; signalIndex < genericSignalsSnapshot.size(); signalIndex++) {
          genericQueues.get(signalIndex).offer(genericSignalsSnapshot.get(signalIndex).getAsDouble());
        }

        for (int timestampIndex = 0; timestampIndex < timestampQueues.size(); timestampIndex++) {
          timestampQueues.get(timestampIndex).offer(timestampSeconds);
        }

        // Opcional: medir jitter/overrun para verlo en AdvantageScope
        // Logger.recordOutput("Odometry/LastTimestampSeconds", timestampSeconds);

      } finally {
        Drive.odometryLock.unlock();
      }
    }
  }

}