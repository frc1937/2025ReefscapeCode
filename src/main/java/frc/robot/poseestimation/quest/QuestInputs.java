package frc.robot.poseestimation.quest;

import gg.questnav.questnav.PoseFrame;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class QuestInputs implements LoggableInputs, Cloneable {
    public boolean connected = false;
    public boolean tracking = false;

    public double batteryPercent = -1.0;

    public PoseFrame[] robotPoses = new PoseFrame[0];

    @Override
    public void toLog(LogTable table) {
        table.put("Connected", connected);
        table.put("Tracking", tracking);
        table.put("BatteryPercent", batteryPercent);
        table.put("Poses", robotPoses);
    }

    @Override
    public void fromLog(LogTable table) {
        connected = table.get("Connected", connected);
        tracking = table.get("Tracking", tracking);
        batteryPercent = table.get("BatteryPercent", batteryPercent);
        robotPoses = table.get("Poses", robotPoses);
    }

    public QuestInputs clone() {
        QuestInputs copy = new QuestInputs();
        copy.connected = this.connected;
        copy.tracking = this.tracking;
        copy.batteryPercent = this.batteryPercent;
        copy.robotPoses = this.robotPoses;
        return copy;
    }
}
