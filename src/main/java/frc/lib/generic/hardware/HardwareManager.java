package frc.lib.generic.hardware;

import frc.lib.generic.OdometryThread;
import frc.lib.generic.advantagekit.LoggableHardware;
import frc.lib.generic.hardware.motor.MotorFactory;
import frc.robot.GlobalConstants;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static frc.robot.GlobalConstants.*;

//Credit to team 418 for this
public enum HardwareManager {
    INSTANCE;

    private static final boolean IS_PRACTICE = true;
    private static final long MIN_FREE_SPACE = IS_PRACTICE ? 100000000 /*100 MB*/: 1000000000 /*1 GB*/;

    HardwareManager() {
    }

    private static final List<LoggableHardware> hardware = new ArrayList<>();
    private static final List<Runnable> periodicRunnable = new ArrayList<>();

    /**
     * Initialize and start logging
     * <p>
     * Call this at the beginning of <code>robotInit()</code>.
     * <p>
     * To enable replay, set the variable <code>CURRENT_MODE=REPLAY</code>
     *
     * @param robot Robot object
     */
    public static void initialize(LoggedRobot robot) {
        String logPath = CURRENT_MODE == Mode.REAL ?"/home/lvuser/logs" : "logs";

        final File logsDirectory = new File(logPath);

        if (!logsDirectory.exists())
            logsDirectory.mkdir();

        cleanOldFiles(logsDirectory);

        if (CURRENT_MODE == GlobalConstants.Mode.REAL || CURRENT_MODE == GlobalConstants.Mode.SIMULATION) {
            Logger.addDataReceiver(new NT4Publisher());

            if (SHOULD_WRITE_LOGS) {
                Logger.addDataReceiver(new WPILOGWriter(logPath));
            }

        } else {
            robot.setUseTiming(true);
            logPath = LogFileUtil.findReplayLog();

            final String logWriterPath = LogFileUtil.addPathSuffix(logPath, "_replay");

            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(logWriterPath));
        }

        Logger.start();
        Logger.disableConsoleCapture();
    }


    /**
     * Add hardware device to hardware logging manager
     * <p>
     * Should not be necessary to call this manually, all devices register themselves when instantiated
     *
     * @param devices Devices to add
     */
    public static void addHardware(LoggableHardware... devices) {
        hardware.addAll(Arrays.asList(devices));
    }

    /**
     * Add custom periodicRunnable to the hardware logging manager to be called every loop
     *
     * @param periodicRunnable Desired periodicRunnable
     */
    public static void addCallback(Runnable periodicRunnable) {
        HardwareManager.periodicRunnable.add(periodicRunnable);
    }

    /**
     * Update all hardware devices
     * <p>
     * Call this periodically, preferably in the beginning of <code>robotPeriodic()</code> every loop
     */
    public static void update() {
        FASTER_THREAD_LOCK.lock();

        OdometryThread.getInstance().updateLatestTimestamps();

        for (LoggableHardware loggableHardware : hardware) {
            loggableHardware.periodic();
        }

        FASTER_THREAD_LOCK.unlock();

        periodicRunnable.forEach(Runnable::run);
    }

    public static void updateSimulation() {
        MotorFactory.updateAllSimulations();
    }

    private static void cleanOldFiles(File logsDirectory) {
        if (CURRENT_MODE != Mode.REAL || !SHOULD_WRITE_LOGS || logsDirectory.getFreeSpace() >= MIN_FREE_SPACE)
            return;

        System.out.println("[!] ERROR: out of space!");
        File[] files = logsDirectory.listFiles();

        if (files == null) {
            System.out.println("[!] ERROR: Cannot delete, Files are NULL!");
            return;
        }

        // Sorting the files by name will ensure that the oldest files are deleted first
        files = Arrays.stream(files).sorted().toArray(File[]::new);

        long bytesToDelete = MIN_FREE_SPACE - logsDirectory.getFreeSpace();

        for (File file : files) {
            if (!file.getName().endsWith(".wpilog")) continue;

            try {
                bytesToDelete -= Files.size(file.toPath());
            } catch (IOException e) {
                System.out.println("[!] Failed to get size of file " + file.getName());
                continue;
            }

            if (file.delete()) {
                System.out.println("[!] Deleted " + file.getName() + " to free up space");
            } else {
                System.out.println("[!] Failed to delete " + file.getName());
            }

            if (bytesToDelete <= 0)
                return;
        }
    }
}
