package frc.robot.commands;

import static frc.robot.Constants.kMaxAlignmentAngularSpeed;
import static frc.robot.Constants.kMaxAlignmentSpeed;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.DataLogHelpers;

/** An example command that uses an example subsystem. */
public class ApriltagAlignment extends Command {
    // TODO: This doesnt account for the camera offset
    private final VisionSubsystem m_photonVision;
    private final DrivetrainSubsystem m_drivetrain;

    private final PIDController m_xPID = new PIDController(1.125, 0.0, 0.0);

    private final PIDController m_yPID = new PIDController(1.125, 0.0, 0.0);

    private final PIDController m_rotPID = new PIDController(2.5, 0.2, 0.0);

    private int m_counter = 0;
    private int m_apriltag;
    private boolean m_output;
    private String prefix; 

    public  ApriltagAlignment(
        int apriltag,
        double xOffset,
        double yOffset,
        VisionSubsystem[] cameras,
        DrivetrainSubsystem drivetrain,
        boolean output
    ) {
        m_output = output;
        m_apriltag = apriltag;
        m_photonVision = cameras[0];

        m_drivetrain = drivetrain;

        m_xPID.setSetpoint(xOffset);
        m_yPID.setSetpoint(yOffset);
        m_rotPID.setSetpoint(Math.PI);
        m_xPID.setTolerance(Constants.kAlignmentXTolerance);
        m_yPID.setTolerance(Constants.kAlignmentYTolerance);
        m_rotPID.setTolerance(Constants.kAlignmentRotTolerance);

        // Integral is only used within +- 12.5 degrees of the target, with -0.1 to 0.1 max influence.
        m_rotPID.setIZone(Units.degreesToRadians(Constants.kAlignmentRotIZone));
        m_rotPID.setIntegratorRange(-Constants.kAlignmentRotIntegrationZone, Constants.kAlignmentRotIntegrationZone);

        if (output) addRequirements(drivetrain);

        prefix = "AT_" + yOffset;

        //System.out.println("+ApriltagAlignment command");

        // SmartDashboard.putData("AT_x_pid", m_xPID);
        // SmartDashboard.putData("AT_y_pid", m_yPID);
        // SmartDashboard.putData("AT_rot_pid", m_rotPID);
    }

    public final void disableOutput() {
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_apriltag = -1;
        m_photonVision.getCamera().takeOutputSnapshot();
        SmartDashboard.putBoolean("FinishedAligning", false);
        m_counter = 0;
    }

    public final ChassisSpeeds m_desiredChassisSpeeds = new ChassisSpeeds();

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        DataLogHelpers.logDouble(1.0, "AlignmentStatus");
        //System.out.println("+ApriltagAlignment.execute");
        var target = m_photonVision.getTargetClosestToCenter();

        // TODO: If the target is lost, the robot will keep rotating.
        // Also, if moving too fast then it can reach the setpoint and then overshoot.
        if (target != null) {
            // SmartDashboard.putNumber("AT_alignment_tag_id", target.fiducialId);
            // SmartDashboard.putNumber("AT_alignment_tag_x", target.bestCameraToTarget.getX());
            // SmartDashboard.putNumber("AT_alignment_tag_y", target.bestCameraToTarget.getY());
            // SmartDashboard.putNumber("AT_alignment_tag_rot", target.bestCameraToTarget.getRotation().getAngle());
            // SmartDashboard.putBoolean("AT_alignment_tag_AtX", m_yPID.atSetpoint());
            // SmartDashboard.putBoolean("AT_alignment_tag_AtY", m_yPID.atSetpoint());
            // SmartDashboard.putBoolean("AT_alignment_tag_AtRot", m_rotPID.atSetpoint());

            m_apriltag = target.fiducialId;
            var x_offset = target.bestCameraToTarget.getMeasureX().baseUnitMagnitude();
            var y_offset = target.bestCameraToTarget.getMeasureY().baseUnitMagnitude();
            var rot = target.bestCameraToTarget.getRotation().getAngle() - 0.03;
            m_desiredChassisSpeeds.vxMetersPerSecond = MathUtil.clamp(-m_xPID.calculate(x_offset), -kMaxAlignmentSpeed * (m_yPID.atSetpoint() ? 1 : 0.5), kMaxAlignmentSpeed * (m_yPID.atSetpoint() ? 1 : 0.5));
            m_desiredChassisSpeeds.vyMetersPerSecond = MathUtil.clamp(-m_yPID.calculate(y_offset), -kMaxAlignmentSpeed, kMaxAlignmentSpeed);
            m_desiredChassisSpeeds.omegaRadiansPerSecond = MathUtil.clamp(m_rotPID.calculate(rot), -kMaxAlignmentAngularSpeed, kMaxAlignmentAngularSpeed);

            if (m_output) m_drivetrain.driveRobotSpeeds(m_desiredChassisSpeeds);

            SmartDashboard.putNumber(prefix + "_XError", m_xPID.getError());
            SmartDashboard.putNumber(prefix + "_YError", m_yPID.getError());
            SmartDashboard.putNumber(prefix + "_RotError", m_rotPID.getError());
            SmartDashboard.putBoolean(prefix + "_atX", m_xPID.atSetpoint());
            SmartDashboard.putBoolean(prefix + "_atY", m_yPID.atSetpoint());
            SmartDashboard.putBoolean(prefix + "_atRot", m_rotPID.atSetpoint());
            SmartDashboard.putNumber(prefix + "_counter", m_counter);
            m_counter++;

            if ((m_counter % 50) == 0)
            {
                m_photonVision.getCamera().takeOutputSnapshot();
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_apriltag = -1;
        m_photonVision.getCamera().takeOutputSnapshot();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_xPID.atSetpoint() && m_yPID.atSetpoint() && m_rotPID.atSetpoint() && (m_apriltag > 0))
        {
            SmartDashboard.putBoolean("FinishedAligning", true);
            m_photonVision.getCamera().takeOutputSnapshot();
            //System.out.println("ApriltagAlignment isFinished");
            return true;
        }
        return false;
    }
}
