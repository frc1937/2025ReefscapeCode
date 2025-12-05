package frc.robot.poseestimation.quest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import gg.questnav.questnav.PoseFrame;
import org.littletonrobotics.junction.Logger;

public class Quest { //todo hardware manager
    private final QuestIO questIO;
    private final QuestInputs inputs;

    public Quest(Transform2d robotToQuest) {
        questIO = QuestIO.generateQuest(robotToQuest);
        inputs = new QuestInputs();
    }

    public void refreshInputs() {
        questIO.updateInputs(inputs);
        Logger.processInputs("Quest", inputs);
    }

    public void setPose(Pose2d pose2d) {
        questIO.setQuestFieldPose(pose2d);
    }

    public boolean isConnected() {
        return inputs.connected;
    }

    public PoseFrame[] getEstimatedPoses() {
        return inputs.robotPoses;
    }

    public double getBatteryPercent() { //TODO: Add alert if battery is low
        return inputs.batteryPercent;
    }

    public boolean isResultValid() {
        return inputs.connected && inputs.tracking;
    }
}
