package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ApriltagOverride extends Command {
    private final VisionSubsystem m_photonVision;

    private final int m_apriltag;
    private int m_counter = 0;
    private ApriltagAlignment m_alignmentCommand;

    public ApriltagOverride(
        int apriltag,
        double xOffset,
        double yOffset,
        VisionSubsystem[] cameras,
        DrivetrainSubsystem drivetrain
    ) {
        m_apriltag = apriltag;
        m_photonVision = cameras[0];
        //m_drivetrain = drivetrain;

        m_alignmentCommand = new ApriltagAlignment(apriltag, xOffset, yOffset, cameras, drivetrain, false);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_alignmentCommand.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var target = m_photonVision.getClosestCameraTarget();

        if ((target != null) && ((m_apriltag == -1) || (target.fiducialId == m_apriltag))) {
            m_counter++;
        }

        else {
            m_counter = 0;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_alignmentCommand.cancel();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //ChassisSpeeds current = m_drivetrain.getChassisSpeeds();
        //ChassisSpeeds alignment = m_alignmentCommand.m_desiredChassisSpeeds;

        SmartDashboard.putNumber("AprilTagCounter", m_counter);

        // This counter will count up even if a new frame is not recieved. It will at least
        // ensure that there is a delay between the first detection and the interrupt.
        if ((m_counter >= 4)) {
            // The squared speed is used to avoid unnecessary square roots, the real speed doesn't matter.
            //double currentSpeedSquared = Math.pow(current.vxMetersPerSecond, 2) + Math.pow(current.vyMetersPerSecond, 2);
            //double alignmentSpeedSquared = Math.pow(alignment.vxMetersPerSecond, 2) + Math.pow(alignment.vyMetersPerSecond, 2);
            
            return m_photonVision.getClosestCameraTargetDistance() <= 1.0;
        }

        return false;
    }
}
