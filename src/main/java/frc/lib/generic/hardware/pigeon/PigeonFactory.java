package frc.lib.generic.hardware.pigeon;

import frc.lib.generic.hardware.pigeon.hardware.GenericIMU;
import frc.lib.generic.hardware.pigeon.hardware.GenericPigeon2;
import frc.lib.generic.hardware.pigeon.hardware.SimulatedIMU;
import frc.robot.GlobalConstants;

import static frc.robot.GlobalConstants.CURRENT_MODE;

public class PigeonFactory {
    public static Pigeon createIMU(String name, int deviceId) {
        if (CURRENT_MODE == GlobalConstants.Mode.REPLAY)
            return new Pigeon(name);

        if (CURRENT_MODE == GlobalConstants.Mode.SIMULATION)
            return new SimulatedIMU(name);

        return new GenericIMU(name, deviceId);
    }

    public static Pigeon createPigeon2(String name, int deviceId) {
        if (CURRENT_MODE == GlobalConstants.Mode.REPLAY)
            return new Pigeon(name);

        if (CURRENT_MODE == GlobalConstants.Mode.SIMULATION)
            return new SimulatedIMU(name);

        return new GenericPigeon2(name, deviceId);
    }
}
