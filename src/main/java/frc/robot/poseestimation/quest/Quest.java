package frc.robot.poseestimation.quest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import org.littletonrobotics.junction.Logger;

public class Quest { //todo hardware manager
    private final QuestIO questIO;
    private final QuestIOInputsAutoLogged inputs;

    public Quest(Transform2d robotToQuest) {
        questIO = QuestIO.generateQuest(robotToQuest);
        inputs = new QuestIOInputsAutoLogged();
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

    public double getTimestamp() {
        return inputs.timestamp;
    }

    public Pose2d getEstimatedPose() {
        return inputs.robotPose;
    }

    public double getBatteryPercent() { //TODO: Add alert if battery is low
        return inputs.batteryPercent;
    }

    public boolean isResultValid() {
        return inputs.connected && inputs.tracking;
    }
}
