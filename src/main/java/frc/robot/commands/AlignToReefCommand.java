package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AprilTagFieldHelpers;

public class AlignToReefCommand {
    /* This function does stuff
     * TODO: Carson to document this function.
     * It is now a static public function
     */
    static public Command alignToReefCommand(double xOffset, double yOffset, DrivetrainSubsystem drivetrain)
    {
        var alignmentTransform = new Transform2d(xOffset, yOffset, Rotation2d.kZero);
        var goal = AprilTagFieldHelpers.getNearestReefPose(drivetrain.getPose())
            .transformBy(alignmentTransform);

        return new DriveToPoseCommand(goal, drivetrain);
    }
}
