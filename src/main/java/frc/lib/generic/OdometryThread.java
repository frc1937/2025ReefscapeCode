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

import static frc.lib.util.QueueUtilities.queueToArrayAndClearQueue;
import static frc.robot.GlobalConstants.*;
import static frc.robot.subsystems.swerve.SwerveConstants.yawOffset;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for devices like the SparkMax that require polling rather than a
 * blocking thread. A Notifier thread is used to gather samples with consistent timing.
 */
public class OdometryThread extends Thread {
    private final List<Queue<Double>> queues = new ArrayList<>();
    private final Queue<Double> timestamps = new ArrayBlockingQueue<>(100);

    private DoubleSupplier[] nonCtreThreadedSignals = new DoubleSupplier[0];
    private BaseStatusSignal[] ctreThreadedSignals = new BaseStatusSignal[0];

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
        if (ctreThreadedSignals.length >= 1)
            BaseStatusSignal.refreshAll(ctreThreadedSignals);

        final double updateTimestamp = (RobotController.getFPGATime() / 1.0e6);

        FASTER_THREAD_LOCK.lock();

        try {
            final int nonCtreSignalsSize = nonCtreThreadedSignals.length;

            for (int i = 0; i < nonCtreSignalsSize; i++) {
                queues.get(i).offer(nonCtreThreadedSignals[i].getAsDouble());
            }

            for (int i = 0; i < ctreThreadedSignals.length; i++) {
                if (ctreThreadedSignals[i].getName() == "Yaw") {
                    queues.get(nonCtreSignalsSize + i).offer((ctreThreadedSignals[i].getValueAsDouble() / 360));
                } else
                    queues.get(nonCtreSignalsSize + i).offer(ctreThreadedSignals[i].getValueAsDouble());
            }

            timestamps.offer(updateTimestamp);
        } finally {
            FASTER_THREAD_LOCK.unlock();
        }
    }

    private void insertCTRESignalToSignalArray(BaseStatusSignal statusSignal) {
        final BaseStatusSignal[] newSignals = new BaseStatusSignal[ctreThreadedSignals.length + 1];

        System.arraycopy(ctreThreadedSignals, 0, newSignals, 0, ctreThreadedSignals.length);
        newSignals[ctreThreadedSignals.length] = statusSignal;

        ctreThreadedSignals = newSignals;
    }

    private void insertSignalToSignalArray(DoubleSupplier statusSignal) {
        final DoubleSupplier[] newSignals = new DoubleSupplier[nonCtreThreadedSignals.length + 1];

        System.arraycopy(nonCtreThreadedSignals, 0, newSignals, 0, nonCtreThreadedSignals.length);
        newSignals[nonCtreThreadedSignals.length] = statusSignal;

        nonCtreThreadedSignals = newSignals;
    }

    public void updateLatestTimestamps() {
        if (CURRENT_MODE != Mode.REPLAY) {
            threadInputs.timestamps = queueToArrayAndClearQueue(timestamps);
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