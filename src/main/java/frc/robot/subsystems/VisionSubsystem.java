/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private final SwerveDrivePoseEstimator m_swervePoseEstimator;
    private Matrix<N3, N1> curStdDevs;
    private int m_closestTagId = -1;
    private double m_closestTagArea = -1;

    private final AprilTagFieldLayout m_apriltagLayout = AprilTagFieldLayout.loadField(
        AprilTagFields.k2025ReefscapeWelded
    );

    // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    int m_cameraSnapShotCounter = 0;

    public VisionSubsystem(String cameraName, Transform3d cameraTransform, SwerveDrivePoseEstimator swervePoseEstimator) {
        camera = new PhotonCamera(cameraName);
        m_swervePoseEstimator = swervePoseEstimator;

        photonEstimator =
                new PhotonPoseEstimator(m_apriltagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraTransform);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // ----- Simulation
        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim.addAprilTags(m_apriltagLayout);
            // Create simulated camera properties. These can be set to mimic your actual camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.
            cameraSim = new PhotonCameraSim(camera, cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(cameraSim, cameraTransform);

            cameraSim.enableDrawWireframe(true);
        }

        //this.setDefaultCommand(getSnapshotCommand());
    }

    public PhotonCamera getCamera() {
        return camera;
    }

    @Override
    public void periodic() {
        Optional<EstimatedRobotPose> estimatedPose = getEstimatedGlobalPose();

        if (estimatedPose.isPresent()) {
            m_swervePoseEstimator.addVisionMeasurement(
                estimatedPose.get().estimatedPose.toPose2d(),
                estimatedPose.get().timestampSeconds,
                getEstimationStdDevs()
            );
        }

        if (m_cameraSnapShotCounter++ > 5/0.02)
        {
            m_cameraSnapShotCounter = 0;
            camera.takeOutputSnapshot();
        }
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     * {@link getEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        var tags = camera.getAllUnreadResults();
        updateClosestTag(tags);
        for (var change : tags) {
            visionEst = photonEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());

            if (Robot.isSimulation()) {
                visionEst.ifPresentOrElse(
                        est ->
                                getSimDebugField()
                                        .getObject("VisionEstimation")
                                        .setPose(est.estimatedPose.toPose2d()),
                        () -> {
                            getSimDebugField().getObject("VisionEstimation").setPoses();
                        });
            }
        }
        return visionEst;
    }

    /**
     * Update the last tag that was seen as it's position
     */
    public void updateClosestTag(List<PhotonPipelineResult> pipelines)
    {
        // m_closestTagId = -1;
        // m_closestTagArea = -1;
        for (var pipeline : pipelines)
        {
            for (var tag : pipeline.getTargets())
            {
                if (tag.area > m_closestTagArea)
                {
                    m_closestTagArea = tag.area;
                    m_closestTagId = tag.fiducialId;
                }
            }
        }
    }

    public int getClosestTagId()
    {
        return m_closestTagId;
    }

    public double getClosestTagArea()
    {
        return m_closestTagArea;
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = VecBuilder.fill(0.65, 0.65, Math.PI / 16.0); // default 4, 4, 8

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VecBuilder.fill(0.25, 0.25, Math.PI / 32.0);
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                var tagdistance = tagPose.get().toPose2d().getTranslation()
                    .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());

                numTags++;
                avgDist += tagdistance;

                // if (tagdistance > 4.0) {
                //     curStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                //     return;
                // }
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = VecBuilder.fill(7.5, 7.5, 12); // single standard devs
            } else {
                if (DriverStation.isDisabled()) {
                    curStdDevs = estStdDevs.div(2.0);
                }

                else {
                    // One or more tags visible, run the full heuristic.
                    avgDist /= numTags;
                    // Decrease std devs if multiple targets are visible
                    if (numTags > 1) estStdDevs = VecBuilder.fill(0.175, 0.175, Math.PI / 64.0); // multi standard devs
                    // Increase std devs based on (average) distance
                    if ((numTags == 1  && avgDist > 2.0) || (avgDist / numTags > 2.0))
                        estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 10.0)); // / 30
                    curStdDevs = estStdDevs;
                }
            }
        }
    }

    public PhotonTrackedTarget getTargetClosestToCenter() {
        PhotonPipelineResult results = camera.getLatestResult();
        PhotonTrackedTarget bestTarget = null;
        double bestTargetY = Double.MAX_VALUE;

        for (PhotonTrackedTarget target : results.targets) {
            if (target.bestCameraToTarget.getMeasureY().baseUnitMagnitude() < bestTargetY) {
                bestTarget = target;
                bestTargetY = target.bestCameraToTarget.getMeasureY().baseUnitMagnitude();
            }
        }

        return bestTarget;
    }

    public PhotonTrackedTarget getClosestCameraTarget() {
        PhotonPipelineResult results = camera.getLatestResult();
        PhotonTrackedTarget bestTarget = null;

        double bestSquaredDistance = Double.MAX_VALUE;

        for (PhotonTrackedTarget target : results.targets) {
            double targetX = target.bestCameraToTarget.getMeasureX().baseUnitMagnitude();
            double targetY = target.bestCameraToTarget.getMeasureY().baseUnitMagnitude();
            double targetZ = target.bestCameraToTarget.getMeasureZ().baseUnitMagnitude();

            double squaredDistance =
                Math.pow(targetX, 2) +
                Math.pow(targetY, 2) +
                Math.pow(targetZ, 2);
            
            if (squaredDistance < bestSquaredDistance) {
                bestTarget = target;
                bestSquaredDistance = squaredDistance;
            }
        }

        return bestTarget;
    }

    public double getClosestCameraTargetDistance() {
        PhotonPipelineResult results = camera.getLatestResult();
        double bestSquaredDistance = Double.MAX_VALUE;

        for (PhotonTrackedTarget target : results.targets) {
            double targetX = target.bestCameraToTarget.getMeasureX().baseUnitMagnitude();
            double targetY = target.bestCameraToTarget.getMeasureY().baseUnitMagnitude();
            double targetZ = target.bestCameraToTarget.getMeasureZ().baseUnitMagnitude();

            SmartDashboard.putNumber("ATtargetX", targetX);
            SmartDashboard.putNumber("ATtargetY", targetY);
            SmartDashboard.putNumber("ATtargetZ", targetZ);
            SmartDashboard.putNumber("ATtargetRot", target.bestCameraToTarget.getRotation().getAngle());

            double squaredDistance =
                Math.pow(targetX, 2) +
                Math.pow(targetY, 2) +
                Math.pow(targetZ, 2);
            
            if (squaredDistance < bestSquaredDistance) {
                bestSquaredDistance = squaredDistance;
            }
            SmartDashboard.putNumber("AT_sq_distance", bestSquaredDistance);
            SmartDashboard.putNumber("AT_id", target.fiducialId);

        }

        return bestSquaredDistance;
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    // ----- Simulation

    public void simulationPeriodic(Pose2d robotSimPose) {
        visionSim.update(robotSimPose);
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }

    private Command getSnapshotCommand() {
        var command = Commands.repeatingSequence(
            new InstantCommand(() -> {
                camera.takeOutputSnapshot();
                System.out.println("Capturing image");
            }),

            new WaitCommand(5.0)
        );

        command.addRequirements(this);
        command.ignoringDisable(true);

        return command;
    }
}
