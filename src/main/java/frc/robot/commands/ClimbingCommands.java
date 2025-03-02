package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.flippable.FlippableRotation2d;
import frc.robot.subsystems.swerve.SwerveCommands;

public class ClimbingCommands {
    public static Command rotateToCage() {
        return SwerveCommands.rotateToTarget(new FlippableRotation2d(Rotation2d.fromDegrees(90), true));
    }
}
