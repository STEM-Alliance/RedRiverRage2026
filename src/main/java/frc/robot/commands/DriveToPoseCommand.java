package frc.robot.commands;

import static frc.robot.Constants.kAlignYDistanceLeftBack;
import static frc.robot.Constants.kAlignYDistanceLeftFront;
import static frc.robot.Constants.kAlignYDistanceRightBack;
import static frc.robot.Constants.kAlignYDistanceRightFront;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kAlignmentOffsets;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AprilTagFieldHelpers;

public class DriveToPoseCommand extends Command {
    private Pose2d m_goal;
    private final DrivetrainSubsystem m_drivetrain;

    private final PIDController m_xPID = new PIDController(1.5, 0.0, 0.0);
    private final PIDController m_yPID = new PIDController(1.5, 0.0, 0.0);
    private final PIDController m_omegaPID = new PIDController(2.5, 0.0, 0.0);

    private final ChassisSpeeds m_desiredSpeeds = new ChassisSpeeds();

    private double m_xOffset = 0;
    private double m_yOffset = 0;

    private kAlignmentOffsets m_alignmentOffsets;
    
    public DriveToPoseCommand(double xOffset, kAlignmentOffsets alignmentOffset, DrivetrainSubsystem drivetrain) {
        m_xOffset = xOffset;
        m_drivetrain = drivetrain;
        m_alignmentOffsets = alignmentOffset;
        addRequirements(m_drivetrain);
    }

    public DriveToPoseCommand(Pose2d goal, DrivetrainSubsystem drivetrain) {
        m_goal = goal;
        m_drivetrain = drivetrain;

        m_xPID.setSetpoint(m_goal.getX());
        m_yPID.setSetpoint(m_goal.getY());
        m_omegaPID.setSetpoint(m_goal.getRotation().getRadians());

        m_xPID.setTolerance(0.015);
        m_yPID.setTolerance(0.015);
        m_omegaPID.setTolerance(Units.degreesToRadians(2.5));

        m_omegaPID.enableContinuousInput(0, Units.degreesToRadians(180));

        addRequirements(m_drivetrain);
    }

    @Override
    public final void initialize() {
        if (m_goal == null)
        {
            var alignmentTransform = new Transform2d(m_xOffset, getAlignmentOffset(m_alignmentOffsets), Rotation2d.kZero);
            var goal = AprilTagFieldHelpers.getNearestReefPose(m_drivetrain.getPose())
                .transformBy(alignmentTransform);

            m_xPID.setSetpoint(goal.getX());
            m_yPID.setSetpoint(goal.getY());
            m_omegaPID.setSetpoint(goal.getRotation().getRadians());
    
            m_xPID.setTolerance(0.015);
            m_yPID.setTolerance(0.015);
            m_omegaPID.setTolerance(Units.degreesToRadians(2.5));
    
            m_omegaPID.enableContinuousInput(0, Units.degreesToRadians(180));

            SmartDashboard.putNumber("yOffset", m_yOffset);
        }
    }

    @Override
    public final void execute() {
        Pose2d currentPose = m_drivetrain.getPose();

        m_desiredSpeeds.vxMetersPerSecond = MathUtil.clamp(m_xPID.calculate(currentPose.getX()), -1.5, 1.5);
        m_desiredSpeeds.vyMetersPerSecond = MathUtil.clamp(m_yPID.calculate(currentPose.getY()), -1.5, 1.5);
        m_desiredSpeeds.omegaRadiansPerSecond = MathUtil.clamp(m_omegaPID.calculate(currentPose.getRotation().getRadians()), -Math.PI, Math.PI);

        SmartDashboard.putBoolean("DriveToPose_XAtSetpoint", m_xPID.atSetpoint());
        SmartDashboard.putBoolean("DriveToPose_YAtSetpoint", m_yPID.atSetpoint());
        SmartDashboard.putBoolean("DriveToPose_OmegaAtSetpoint", m_omegaPID.atSetpoint());
        SmartDashboard.putNumber("XDerivative", m_xPID.getErrorDerivative());
        SmartDashboard.putNumber("YDerivative", m_yPID.getErrorDerivative());

        m_drivetrain.driveFieldSpeeds(m_desiredSpeeds);
    }

    @Override
    public final void end(boolean interrupted) {}

    @Override
    public final boolean isFinished() {
        return (m_xPID.atSetpoint() && m_yPID.atSetpoint() && m_omegaPID.atSetpoint());
    }

    private final double getAlignmentOffset(kAlignmentOffsets alignmentOffset) {
        Pose2d currentPose = m_drivetrain.getPose();
        Pose2d nearestReefPose = AprilTagFieldHelpers.getNearestReefPose(currentPose);

        double rotationDifference = currentPose.getRotation().minus(nearestReefPose.getRotation()).getDegrees();
        boolean facingTowards = ((-90 > rotationDifference) && (rotationDifference > -180)) ||
                             ((180 > rotationDifference) && (rotationDifference > 90));

        switch (alignmentOffset) {
            case LEFT:
                return (facingTowards ? kAlignYDistanceLeftFront : kAlignYDistanceLeftBack);

            case RIGHT:
                return (facingTowards ? kAlignYDistanceRightFront : kAlignYDistanceRightBack);
            
            default:
                return 0.0;
        }
    }
}
