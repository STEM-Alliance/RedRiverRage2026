package frc.robot.commands;

import static frc.robot.Constants.*;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * See https://docs.google.com/document/d/10if4xjAaETTceUVn7l4J-jOCOnm5CJUDS5RAVNIJMQM/edit?tab=t.0
 * The purpose of this is to guarantee the final position is at the goal pose.
*/
public final class ApriltagAlignmentCommandV2 extends Command {
    private final DrivetrainSubsystem m_drivetrain;

    private final Pose2d m_goalPose;
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final PPHolonomicDriveController m_driveController = kPathplannerDriveController;

    public ApriltagAlignmentCommandV2(DrivetrainSubsystem drivetrain, Pose2d goalPose) {
        m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);

        m_goalPose = goalPose;
        m_poseEstimator = m_drivetrain.getPoseEstimator();
    }

    @Override
    public final void execute() {
        PathPlannerTrajectoryState trajectoryState = new PathPlannerTrajectoryState();
        trajectoryState.pose = m_goalPose;

        ChassisSpeeds desiredRobotSpeeds = m_driveController.calculateRobotRelativeSpeeds(
            m_poseEstimator.getEstimatedPosition(), trajectoryState
        );

        desiredRobotSpeeds.omegaRadiansPerSecond = MathUtil.clamp(
            desiredRobotSpeeds.omegaRadiansPerSecond, Units.degreesToRadians(-90.0), Units.degreesToRadians(90.0)
        );

        desiredRobotSpeeds.vxMetersPerSecond = MathUtil.clamp(
            desiredRobotSpeeds.vxMetersPerSecond, -1.0, 1.0
        );

        desiredRobotSpeeds.vyMetersPerSecond = MathUtil.clamp(
            desiredRobotSpeeds.vyMetersPerSecond, -1.0, 1.0
        );

        m_drivetrain.driveRobotSpeeds(desiredRobotSpeeds);
    }

    @Override
    public final void end(boolean interrupted) {}

    @Override
    public final boolean isFinished() {
        Pose2d poseError = m_poseEstimator.getEstimatedPosition().relativeTo(m_goalPose);
        ChassisSpeeds chassisSpeeds = m_drivetrain.getChassisSpeeds();

        boolean isWithinPositionTolerance;
        boolean isWithinRotationTolerance;
        boolean isWithinVelocityTolerance;

        isWithinPositionTolerance = poseError.getTranslation().getNorm() < 0.05;
        isWithinRotationTolerance = poseError.getRotation().getRotations() < Units.degreesToRadians(5.0);
        isWithinVelocityTolerance =
            (m_drivetrain.getVelocityMagnitude(chassisSpeeds).baseUnitMagnitude() < 0.05) &&
            (chassisSpeeds.omegaRadiansPerSecond < Units.degreesToRadians(2.5));

        return (isWithinPositionTolerance && isWithinRotationTolerance && isWithinVelocityTolerance);
    }
}
