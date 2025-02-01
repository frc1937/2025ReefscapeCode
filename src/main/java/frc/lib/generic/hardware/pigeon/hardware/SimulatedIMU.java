package frc.lib.generic.hardware.pigeon.hardware;

import edu.wpi.first.math.util.Units;
import frc.lib.generic.hardware.pigeon.Pigeon;
import frc.lib.generic.hardware.pigeon.PigeonInputs;
import frc.lib.generic.hardware.pigeon.PigeonSignal;

import static frc.lib.generic.hardware.pigeon.PigeonInputs.PIGEON_INPUTS_LENGTH;
import static frc.robot.GlobalConstants.ROBOT_PERIODIC_LOOP_TIME;
import static frc.robot.RobotContainer.SWERVE;

public class SimulatedIMU extends Pigeon {
    private double simulatedYawRotations = 0;
    private final boolean[] signalsToLog = new boolean[PIGEON_INPUTS_LENGTH];

    public SimulatedIMU(String name) {
        super(name);
    }

    public void update(double omegaRadiansPerSecond, double timeSeconds) {
        simulatedYawRotations += Units.radiansToRotations(omegaRadiansPerSecond * timeSeconds);
    }

    @Override
    public double getYawRotations() {
        return simulatedYawRotations;
    }

    @Override
    public void setGyroYaw(double yawRotations) {
        simulatedYawRotations = yawRotations;
    }

    @Override
    public void setupSignalUpdates(PigeonSignal signal, boolean useFasterThread) {
        if (useFasterThread) {
            signalsToLog[signal.getId() + PIGEON_INPUTS_LENGTH / 2] = true;
        }

        signalsToLog[signal.getId()] = true;
    }

    @Override
    protected void refreshInputs(PigeonInputs inputs) {
        inputs.setSignalsToLog(signalsToLog);

        update(SWERVE.getRobotRelativeVelocity().omegaRadiansPerSecond, ROBOT_PERIODIC_LOOP_TIME);

        inputs.gyroYawRotations = getYawRotations();
        inputs.threadGyroYawRotations = new double[]{inputs.gyroYawRotations};
    }
}
