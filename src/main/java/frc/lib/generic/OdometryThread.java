package frc.lib.generic;

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

import static frc.robot.GlobalConstants.CURRENT_MODE;
import static frc.robot.GlobalConstants.FASTER_THREAD_LOCK;
import static frc.robot.GlobalConstants.Mode;
import static frc.robot.GlobalConstants.ODOMETRY_FREQUENCY_HERTZ;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for devices like the SparkMax that require polling rather than a
 * blocking thread. A Notifier thread is used to gather samples with consistent timing.
 */
public class OdometryThread extends Thread {
    private final List<DoubleSupplier> signals = new ArrayList<>();
    private final List<Queue<Double>> queues = new ArrayList<>();
    private final Queue<Double> timestamps = new ArrayBlockingQueue<>(100);

    private static OdometryThread INSTANCE = null;

    private final ThreadInputsAutoLogged threadInputs = new ThreadInputsAutoLogged();

    public static OdometryThread getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new OdometryThread();
        }
        return INSTANCE;
    }

    private OdometryThread() {
        if (CURRENT_MODE == Mode.REPLAY) return;

        Notifier notifier = new Notifier(this::periodic);
        notifier.setName("OdometryThread");
        Timer.delay(5);
        notifier.startPeriodic(1.0 / ODOMETRY_FREQUENCY_HERTZ);
    }

    public Queue<Double> registerSignal(DoubleSupplier signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(100);
        FASTER_THREAD_LOCK.lock();

        try {
            signals.add(signal);
            queues.add(queue);
        } finally {
            FASTER_THREAD_LOCK.unlock();
        }

        return queue;
    }

    private void periodic() {
        FASTER_THREAD_LOCK.lock();

        timestamps.offer(RobotController.getFPGATime() / 1.0e6);

        try {
            for (int i = 0; i < signals.size(); i++) {
                queues.get(i).offer(signals.get(i).getAsDouble());
            }
        } finally {
            FASTER_THREAD_LOCK.unlock();
        }
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