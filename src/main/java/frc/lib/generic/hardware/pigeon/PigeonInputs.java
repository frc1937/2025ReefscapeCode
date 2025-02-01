package frc.lib.generic.hardware.pigeon;

import frc.lib.generic.advantagekit.ChoosableLoggedInputs;
import org.littletonrobotics.junction.LogTable;

public class PigeonInputs implements ChoosableLoggedInputs {
    public static final int PIGEON_INPUTS_LENGTH = 6;

    public double gyroYawRotations = 0;
    public double gyroRollRotations = 0;
    public double gyroPitchRotations = 0;

    public double[] threadGyroYawRotations = new double[0];
    public double[] threadGyroPitchRotations = new double[0];
    public double[] threadGyroRollRotations = new double[0];

    private boolean[] signalsToLog;

    @Override
    public void setSignalsToLog(boolean[] signalsToLog) {
        this.signalsToLog = signalsToLog;
    }

    @Override
    public void toLog(LogTable table) {
        if (signalsToLog == null) return;

        if (signalsToLog[0]) table.put("GyroYawRotations", gyroYawRotations);
        if (signalsToLog[1]) table.put("GyroRollRotations", gyroRollRotations);
        if (signalsToLog[2]) table.put("GyroPitchRotations", gyroPitchRotations);

        if (signalsToLog[3]) table.put("ThreadGyroYawRotations", threadGyroYawRotations);
        if (signalsToLog[4]) table.put("ThreadGyroPitchRotations", threadGyroPitchRotations);
        if (signalsToLog[5]) table.put("ThreadGyroRollRotations", threadGyroRollRotations);
    }

    @Override
    public void fromLog(LogTable table) {
        gyroYawRotations = table.get("GyroYawRotations", gyroYawRotations);
        gyroRollRotations = table.get("GyroRollRotations", gyroRollRotations);
        gyroPitchRotations = table.get("GyroPitchRotations", gyroPitchRotations);

        threadGyroYawRotations = table.get("ThreadGyroYawRotations", threadGyroYawRotations);
        threadGyroPitchRotations = table.get("ThreadGyroPitchRotations", threadGyroPitchRotations);
        threadGyroRollRotations = table.get("ThreadGyroRollRotations", threadGyroRollRotations);
    }
}
