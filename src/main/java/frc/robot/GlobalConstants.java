package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class GlobalConstants {
    public static final double ROBOT_PERIODIC_LOOP_TIME = 0.02;

    public static final Lock FASTER_THREAD_LOCK = new ReentrantLock();
    public static final double ODOMETRY_FREQUENCY_HERTZ = 200.0;

    public static final double GRAVITY = 9.80665;

    public static final double FIELD_LENGTH_METRES = 17.55;

    public static final double VOLTAGE_COMPENSATION_SATURATION = 12;

    public static final boolean IS_REPLAY = (false);

    public static final boolean SHOULD_WRITE_LOGS = true; //for when the RoboRio doesn't have enough space...
    public static final boolean SHOULD_DISPLAY_MECHANISMS = false; //For when we want to save resources.

    public static final Mode CURRENT_MODE;

    public enum Mode {
        REAL,
        SIMULATION,
        REPLAY
    }

    static {
        if (RobotBase.isReal()) {
            CURRENT_MODE = Mode.REAL;
        } else if (RobotBase.isSimulation() && !IS_REPLAY) {
            CURRENT_MODE = Mode.SIMULATION;
        } else {
            CURRENT_MODE = Mode.REPLAY;
        }
    }

    public static final boolean IS_SIMULATION = CURRENT_MODE == Mode.SIMULATION;
}
