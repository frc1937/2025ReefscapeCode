package frc.lib.generic;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;

import static frc.robot.GlobalConstants.*;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for devices like the SparkMax that require polling rather than a
 * blocking thread. A Notifier thread is used to gather samples with consistent timing.
 */
public class OdometryThread extends Thread {
    private final List<Queue<Double>> queues = new ArrayList<>();
    private final Queue<Double> timestamps = new ArrayBlockingQueue<>(100);

    private DoubleSupplier[] nonCtreSignals = new DoubleSupplier[0];
    private BaseStatusSignal[] ctreSignals = new BaseStatusSignal[0];

    private static OdometryThread INSTANCE = null;

    private final ThreadInputsAutoLogged threadInputs = new ThreadInputsAutoLogged();

    public static OdometryThread getInstance() {
        if (INSTANCE == null)
            INSTANCE = new OdometryThread();

        return INSTANCE;
    }

    private OdometryThread() {
        if (CURRENT_MODE == Mode.REPLAY) return;

        Notifier notifier = new Notifier(this::periodic);
        notifier.setName("OdometryThread");
        Timer.delay(5);
        notifier.startPeriodic(1.0 / ODOMETRY_FREQUENCY_HERTZ);
    }

    public Queue<Double> registerCTRESignal(BaseStatusSignal signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(100);
        FASTER_THREAD_LOCK.lock();

        try {
            insertCTRESignalToSignalArray(signal);
            queues.add(queue);
        } finally {
            FASTER_THREAD_LOCK.unlock();
        }

        return queue;
    }

    public Queue<Double> registerSignal(DoubleSupplier signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(100);
        FASTER_THREAD_LOCK.lock();

        try {
            insertSignalToSignalArray(signal);
            queues.add(queue);
        } finally {
            FASTER_THREAD_LOCK.unlock();
        }

        return queue;
    }

    private void periodic() {
        BaseStatusSignal.refreshAll(ctreSignals);
        final double updateTimestamp = (RobotController.getFPGATime() / 1.0e6 - ctreSignals[0].getTimestamp().getLatency());

        FASTER_THREAD_LOCK.lock();

        try {
            final int nonCtreSignalsSize = nonCtreSignals.length;

            for (int i = 0; i < nonCtreSignalsSize; i++) {
                queues.get(i).offer(nonCtreSignals[i].getAsDouble());
            }

            for (int i = 0; i < ctreSignals.length; i++) {
                queues.get(nonCtreSignalsSize + i).offer(ctreSignals[i].getValueAsDouble());
            }

            timestamps.offer(updateTimestamp);
        } finally {
            FASTER_THREAD_LOCK.unlock();
        }
    }

    private void insertCTRESignalToSignalArray(BaseStatusSignal statusSignal) {
        final BaseStatusSignal[] newSignals = new BaseStatusSignal[ctreSignals.length + 1];

        System.arraycopy(ctreSignals, 0, newSignals, 0, ctreSignals.length);
        newSignals[ctreSignals.length] = statusSignal;

        ctreSignals = newSignals;
    }

    private void insertSignalToSignalArray(DoubleSupplier statusSignal) {
        final DoubleSupplier[] newSignals = new DoubleSupplier[nonCtreSignals.length + 1];

        System.arraycopy(nonCtreSignals, 0, newSignals, 0, nonCtreSignals.length);
        newSignals[nonCtreSignals.length] = statusSignal;

        nonCtreSignals = newSignals;
    }

    public void updateLatestTimestamps() {
        if (CURRENT_MODE != Mode.REPLAY) {
            threadInputs.timestamps = timestamps.stream().mapToDouble(Double::doubleValue).toArray();
            timestamps.clear();
        }
        Logger.processInputs("OdometryThread", threadInputs);
    }

    public double[] getLatestTimestamps() {
        return threadInputs.timestamps;
    }

    @AutoLog
    public static class ThreadInputs {
        public double[] timestamps = {0.0};
    }
}