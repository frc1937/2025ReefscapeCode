package frc.lib.generic;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;

import static frc.lib.util.QueueUtilities.queueToDoubleArray;
import static frc.robot.GlobalConstants.*;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for devices like the SparkMax that require polling rather than a
 * blocking thread. A Notifier thread is used to gather samples with consistent timing.
 */
public class OdometryThread {
    private final List<Queue<Double>> queues = new ArrayList<>();
    private final Queue<Double> timestamps = new ArrayBlockingQueue<>(100);

    private BaseStatusSignal[] ctreThreadedSignals = new BaseStatusSignal[0];

    private final ThreadInputsAutoLogged threadInputs = new ThreadInputsAutoLogged();

    private static OdometryThread INSTANCE = null;

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

    private void periodic() {
        if (BaseStatusSignal.refreshAll(ctreThreadedSignals) != StatusCode.OK)
            return;

        final double currentTimestamp = RobotController.getFPGATime() / 1e6;

        FASTER_THREAD_LOCK.lock();

        try {
            for (int i = 0; i < ctreThreadedSignals.length; i++) {
                if (ctreThreadedSignals[i].getName() == "Yaw") {
                    queues.get(i).offer((ctreThreadedSignals[i].getValueAsDouble() / 360));
                } else
                    queues.get(i).offer(ctreThreadedSignals[i].getValueAsDouble());
            }

            timestamps.offer(currentTimestamp);
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

    public void updateLatestTimestamps() {
        if (CURRENT_MODE != Mode.REPLAY) {
            threadInputs.timestamps = queueToDoubleArray(timestamps);
        }

        Logger.processInputs("OdometryThread", threadInputs);
    }

//    private double calculateLatency() {
//        double totalLatency = 0.0;
//
//        for (BaseStatusSignal signal : ctreThreadedSignals)
//            totalLatency += signal.getTimestamp().getLatency();
//
//        return totalLatency / ctreThreadedSignals.length;
//    }

    public double[] getLatestTimestamps() {
        return threadInputs.timestamps;
    }

    @AutoLog
    public static class ThreadInputs {
        public double[] timestamps;
    }
}