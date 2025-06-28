package frc.robot.poseestimation.quest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;

public class Quest {
    private final QuestIO questIO;
    private final QuestIOInputsAutoLogged inputs;

    public Quest(Transform2d robotToQuest) {
        questIO = QuestIO.generateQuest(robotToQuest);
        inputs = new QuestIOInputsAutoLogged();
    }

    public void refreshInputs() {
        questIO.updateInputs(inputs);
    }

    public double getTimestamp() {
        return inputs.timestamp;
    }

    public Pose2d getEstimatedPose() {
        return inputs.robotPose;
    }

    public double getBatteryPercent() {
        return inputs.batteryPercent;
    }

    public boolean isResultValid() {
        return inputs.connected && inputs.tracking;
    }
}
