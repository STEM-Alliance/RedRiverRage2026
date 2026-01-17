// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.commands.ApriltagAlignmentCommandV2;
import frc.robot.misc.SwerveModule;
import frc.robot.util.ControllerProcessing;
import frc.robot.util.DataLogHelpers;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class DrivetrainSubsystem extends SubsystemBase {
  // https://pathplanner.dev/pplib-build-an-auto.html#create-a-sendablechooser-with-all-autos-in-project

  // Locations for the swerve drive modules relative to the robot center.
  Translation2d m_frontLeftLocation = Constants.kSwerveTranslations[0];
  Translation2d m_frontRightLocation = Constants.kSwerveTranslations[1];
  Translation2d m_backLeftLocation = Constants.kSwerveTranslations[2];
  Translation2d m_backRightLocation = Constants.kSwerveTranslations[3];

  private final int[] frontLeftChannels = Constants.kSwerveFLCanID;
  private final int[] frontRightChannels = Constants.kSwerveFRCanID;
  private final int[] backLeftChannels = Constants.kSwerveBLCanID;
  private final int[] backRightChannels = Constants.kSwerveBRCanID;

  private final SwerveModule m_frontLeft = new SwerveModule(
    frontLeftChannels[0], frontLeftChannels[1], frontLeftChannels[2], frontLeftChannels[3]);

  private final SwerveModule m_frontRight = new SwerveModule(
    frontRightChannels[0], frontRightChannels[1], frontRightChannels[2], frontRightChannels[3]);

  private final SwerveModule m_backLeft = new SwerveModule(
    backLeftChannels[0], backLeftChannels[1], backLeftChannels[2], backLeftChannels[3]);

  private final SwerveModule m_backRight = new SwerveModule(
    backRightChannels[0], backRightChannels[1], backRightChannels[2], backRightChannels[3]);
      
  private final SwerveModule m_modules[] = {m_frontLeft, m_frontRight, m_backLeft, m_backRight};
  private final Pigeon2 m_pigeon2 = new Pigeon2(Constants.kPigeon2CanID);
  private boolean m_climbSpeed = false;

  // Creating my kinematics object using the module locations
  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
  m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  boolean m_turbo = false;

  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
    m_kinematics,
    m_pigeon2.getRotation2d(),
    getModulePositions(),
    Pose2d.kZero
  );

  public SwerveDrivePoseEstimator getPoseEstimator() {
    return m_poseEstimator;
  }

  double m_desiredAngle = 0;

  public RobotConfig m_robotConfig;
  private final Alert m_pathplannerRobotConfigAlert = new Alert(
    "DrivetrainSubsystem: Pathplanner failed to load the robot configuration!",
    AlertType.kError
  );

  private final Field2d m_field;

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutDistance m_distance = Meters.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

  // Create a new SysId routine for characterizing the drive.
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              voltage -> {
                driveVoltage(voltage);
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            getMotorOutput() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(getMotorDistance(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(getMotorVelocity(), MetersPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));

  
    /** Creates a new DriveSubSystem. */
    public DrivetrainSubsystem(Field2d field) {
      m_pigeon2.reset();
      m_field = field;
  
      // Set deviations for the pose estimator vision measurements, rotation is positive infinity since
      // the gyro will give us more accurate results than the vision system. We should also scale this by the
      // distance of the tag detected, longer distances will be less accurate so will have less of an effect
  
      // Actual deviations are 0.5, over time odometry becomes less accurate?
      // Deviaton for rotation should be Double.POSITIVE_INFINITY
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(3.5, 3.5, 6.5));
  
      try {
        m_robotConfig = RobotConfig.fromGUISettings();
        m_pathplannerRobotConfigAlert.close();
      }
      
      catch (Exception e) {
        m_pathplannerRobotConfigAlert.set(true);
        // TODO: I am unsure what effects this will have, should this just disable autonomous?
        m_robotConfig = new RobotConfig(null, null, null, null, null);
    }

    AutoBuilder.configure(
      this::getPose, // Pose2d supplier
      this::resetPose, // Method to reset odometry
      this::getChassisSpeeds, // Robot-relative ChassisSpeeds supplier
      (speeds, feedforwards) -> driveRobotSpeeds(speeds), // Robot-relative driving with optional feedfowards.

      kPathplannerDriveController,

      m_robotConfig, // Robot hardware configuration

      () -> {
        // Boolean supplier that controls when the path is mirrored for the red alliance.
        // This will flip the path being followed to the red side of the field, but the
        // origin will remain on the blue side of the field.
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isEmpty()) {
          // By default, the path will not be mirrored.
          return false;
        }

        return alliance.get() == DriverStation.Alliance.Red;
      },

      this
    );
  }

  public void periodic() {
    updateOdometry();
    m_field.setRobotPose(getPose());
    
    DataLogHelpers.logDouble(m_pigeon2.getYaw().getValue().in(Units.Degrees), "Pigeon2Yaw");
    DataLogHelpers.logDouble(getPose().getX(), "RobotPoseX");
    DataLogHelpers.logDouble(getPose().getY(), "RobotPoseY");
    DataLogHelpers.logDouble(getPose().getRotation().getDegrees(), "RobotPoseDeg");
    DataLogHelpers.logDouble(m_poseEstimator.getEstimatedPosition().getX(), "RobotPoseX2");
    DataLogHelpers.logDouble(m_poseEstimator.getEstimatedPosition().getY(), "RobotPoseY2");
    DataLogHelpers.logDouble(m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), "RobotPoseDeg2");
    SmartDashboard.putNumber("RobotX", getPose().getX());
    SmartDashboard.putNumber("RobotY", getPose().getY());
  }

  public final Command getTeleopDriveCommand(CommandXboxController driverController) {
    return new RunCommand(
      () -> {
        // Since the field is sideways (in the coordinate system) relative to the driver station,
        // the x and y coordinates are flipped (x is horizontal on the joystick, vertical on the field).
        double[] processedXY = ControllerProcessing.getProcessedTranslation(
          driverController.getLeftY(), driverController.getLeftX()
        );

        double processedOmega = ControllerProcessing.getProcessedOmega(driverController.getRightX());

        controllerDrive(
          processedXY[0] * (m_climbSpeed ? kMaxSpeed / 4.0 : kMaxSpeed),
          processedXY[1] * (m_climbSpeed ? kMaxSpeed / 4.0 : kMaxSpeed),
          processedOmega * (m_climbSpeed ? kMaxAngularSpeed / 4.0 : kMaxAngularSpeed),
          true,
          0.02
        );
      },
      this
    ).withName("TeleOpCmd");
  }

  public SwerveModule getModule(int offset) {
    return m_modules[offset];
  }

/**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param m_fieldRelative Whether the provided x and y speeds are relative to the m_field.
   */
  public void controllerDrive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot,
                        //TODO: Test in einsteins
                        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ?
                          m_poseEstimator.getEstimatedPosition().getRotation() :
                          m_poseEstimator.getEstimatedPosition().getRotation().plus(Rotation2d.kPi))
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.kMaxSpeed);

    for (int i = 0; i < 4; i++) {
      m_modules[i].setDesiredState(swerveModuleStates[i]);
      DataLogHelpers.logDouble(swerveModuleStates[i].speedMetersPerSecond, "Swerve_" + i + "_drive");
      DataLogHelpers.logDouble(swerveModuleStates[i].angle.getDegrees(), "Swerve_" + i + "_angle");
    }
  }

  public void driveFieldSpeeds(ChassisSpeeds fieldSpeeds) {
    var relativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      fieldSpeeds, getPose().getRotation());
    
    driveRobotSpeeds(relativeSpeeds);
  }

  public Command driveStraight(double velocity, boolean fieldRelative) {
    return new FunctionalCommand(() -> {}, 
                                 () -> {controllerDrive(velocity, 0, 0, fieldRelative, 0.02);}, 
                                 interrupted -> controllerDrive(velocity, 0, 0, fieldRelative, 0.02),
                                 () -> false, this);
  }

  public ChassisSpeeds pathplannerSpeeds = new ChassisSpeeds();

  public void driveRobotSpeeds(ChassisSpeeds robotSpeeds) {
    // TODO: Make the pathplanner GUI speed limit actually limit speed
    // Use the robot config module config max speeds
    // Should there be a seperate function for pathplanner drive?
    robotSpeeds.vxMetersPerSecond = -robotSpeeds.vxMetersPerSecond;
    robotSpeeds.vyMetersPerSecond = -robotSpeeds.vyMetersPerSecond;
    robotSpeeds.omegaRadiansPerSecond = -robotSpeeds.omegaRadiansPerSecond;

    // robotSpeeds.vxMetersPerSecond = MathUtil.clamp(
    //   robotSpeeds.vxMetersPerSecond, -1.5, 1.5
    // );

    // robotSpeeds.vyMetersPerSecond = MathUtil.clamp(
    //   robotSpeeds.vyMetersPerSecond, -1.5, 1.5
    // );

    // robotSpeeds.omegaRadiansPerSecond = MathUtil.clamp(
    //   robotSpeeds.omegaRadiansPerSecond, -1.0, 1.0
    // );

    if (DriverStation.isAutonomous()) {
      pathplannerSpeeds = robotSpeeds;

      double maxDriveVelocity = m_robotConfig.moduleConfig.maxDriveVelocityMPS;
      double maxRotationalVelocity = m_robotConfig.moduleConfig.maxDriveVelocityRadPerSec;

      robotSpeeds.vxMetersPerSecond = MathUtil.clamp(
        robotSpeeds.vxMetersPerSecond, -maxDriveVelocity, maxDriveVelocity
      );

      robotSpeeds.vyMetersPerSecond = MathUtil.clamp(
        robotSpeeds.vyMetersPerSecond, -maxDriveVelocity, maxDriveVelocity
      );

      robotSpeeds.omegaRadiansPerSecond = MathUtil.clamp(
        robotSpeeds.omegaRadiansPerSecond, -maxRotationalVelocity, maxRotationalVelocity
      );

      SmartDashboard.putNumber("PathplannerVX", robotSpeeds.vxMetersPerSecond);
      SmartDashboard.putNumber("PathplannerVY", robotSpeeds.vyMetersPerSecond);
      SmartDashboard.putNumber("PathplannerOmega", robotSpeeds.omegaRadiansPerSecond);
    }

    var targetSpeeds = ChassisSpeeds.discretize(robotSpeeds, 0.02);
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(targetSpeeds);

    // DataLogHelpers.logDouble(robotSpeeds.vxMetersPerSecond, "Vx");
    // DataLogHelpers.logDouble(robotSpeeds.vyMetersPerSecond, "Vy");
    // DataLogHelpers.logDouble(robotSpeeds.omegaRadiansPerSecond, "Omega");

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.kMaxSpeed);

    for (int i = 0; i < 4; i++) {
      m_modules[i].setDesiredState(swerveModuleStates[i]);
    }
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    };
  }

  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getModuleStates());
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      states[i] = m_modules[i].getState();
    }
    return states;
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_poseEstimator.update(m_pigeon2.getRotation2d(), getModulePositions());
  }

  public void addVisionMeasurements(Pose2d visionMeasurements, double timestamp) {
    m_poseEstimator.addVisionMeasurement(visionMeasurements, timestamp);
  }

  public Pose2d getPose() {
    // return m_odometry.getPoseMeters();
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d resetPose) {
    //System.out.println("Reset pose to " + resetPose);
    m_poseEstimator.resetPosition(m_pigeon2.getRotation2d(), getModulePositions(), resetPose);
  }

  public void setGyro(double robotHeading) {
    m_pigeon2.setYaw(robotHeading);
  }

  public double getContinuousHeading() {
    return m_pigeon2.getYaw().getValue().in(Units.Degrees);
  }

  public Command resetGyro() {
    return new InstantCommand(() -> {m_pigeon2.reset();});
  }

  private void setBrakeMode(boolean enabled) {
    for (int i = 0; i < 4; i++) {
      m_modules[i].setBrake(enabled);
    }
  }

  public Command setBrakeModeCmd() {
    return new InstantCommand(() -> setBrakeMode(true));
  }

  public Command setCoastModeCmd() {
    return new InstantCommand(() -> setBrakeMode(false));
  }

  public Command enableTurbo() {
      return new InstantCommand(() -> m_turbo = true);
  }

  public Command disableTurbo() {
      return new InstantCommand(() -> m_turbo = false);
  }

  public final Command toggleClimbSpeed() {
    return new InstantCommand(() -> {
      m_climbSpeed = !m_climbSpeed;
    });
  }

  // Idea Graciously and Professionally borrowed from FRC 6328 Mechanical Advantage's AdvantageKit.
  public final Command calculateWheelDiameters(CommandXboxController controller) {
    double initialAverageEncoderDistance = 0.0;

    for (SwerveModule module : m_modules) {
      initialAverageEncoderDistance += Math.abs(module.getDriveMotor().getPosition().getValueAsDouble());
    }

    initialAverageEncoderDistance /= m_modules.length;

    SmartDashboard.putNumber("initialAverageEncoderDistance", initialAverageEncoderDistance);

    ProfiledPIDController omegaPID = new ProfiledPIDController(2.0, 0.0, 0.0,
      new TrapezoidProfile.Constraints(Math.PI / 3.0, Math.PI / 3.0 / 1.5));
    
    omegaPID.setGoal(m_pigeon2.getYaw().getValueAsDouble() * Math.PI/180);
    omegaPID.setTolerance(0.5);

    m_pigeon2.reset();

    return new FunctionalCommand(
      () -> {},
      () -> {
        driveFieldSpeeds(new ChassisSpeeds(0.0, 0.0, Math.PI / 6));
          // MathUtil.clamp(omegaPID.calculate(m_pigeon2.getYaw().getValueAsDouble() * Math.PI/180),
          //   -Math.PI / 3.0,
          //   Math.PI / 3.0
          //)));
      },
      interrupted -> {
        double endingAverageEncoderDistance = 0.0;

        for (SwerveModule module : m_modules) {
          endingAverageEncoderDistance += Math.abs(module.getDriveMotor().getPosition().getValueAsDouble());
        }
    
        endingAverageEncoderDistance /= m_modules.length;

        SmartDashboard.putNumber("endingAverageEncoderDistance", endingAverageEncoderDistance);
        SmartDashboard.putNumber("endingYaw", m_pigeon2.getYaw().getValueAsDouble());
      },
      () -> m_pigeon2.getYaw().getValueAsDouble() >= 360,
      this
    );
    // return new RunCommand(
    //   () -> {
    //     double gyroRate = m_pigeon2.getAngularVelocityZDevice().getValueAsDouble() * (Math.PI / 180.0);
    //     double drivetrainRadius = kSwerveTranslations[0].getNorm();
    //     double averageWheelVelocity = 0.0;

    //     for (SwerveModule module : m_modules) {
    //       averageWheelVelocity += module.getDriveMotor()
    //         .getVelocity().getValueAsDouble() * (2.0 * Math.PI / kDriveGearReduction / 60.0);
    //     }

    //     averageWheelVelocity /= m_modules.length;
    //     SmartDashboard.putNumber("MeasuredWheelRadius",
    //       (gyroRate * drivetrainRadius) / averageWheelVelocity
    //     );
    //   }
    // );
  }

  public void setGains(double kp, double ki, double kd, double ks, double kv) {
    m_frontLeft.setGains(kp, ki, kd, ks, kv);
    m_frontRight.setGains(kp, ki, kd, ks, kv);
    m_backLeft.setGains(kp, ki, kd, ks, kv);
    m_backRight.setGains(kp, ki, kd, ks, kv);
  }

  public void printHomePos() {
    double abspos[] = new double[4];
    for (int i = 0; i < 4; i++) { 
      abspos[i] = m_modules[i].getAbsPos();
      //System.out.println("abspos_" + i + ": " + abspos[i]);
    }
    SmartDashboard.putNumberArray("abspos", abspos);
  }

  private final void driveVoltage(Voltage outputVolts) {
    //System.out.println("Unimplemented.");
    // for (SwerveModule module : m_modules) {
    //   module.getDriveMotor().setVoltage(outputVolts);
    // }
  }

  private final double getMotorOutput() {
    double summedAppliedOutput = 0.0;

    for (SwerveModule module : m_modules) {
      summedAppliedOutput += module.getDriveMotor().get();
    }

    return summedAppliedOutput / m_modules.length;
  }

  private final double getMotorDistance() {
    double summedDistance = 0.0;

    for (SwerveModule module : m_modules) {
      summedDistance += module.getDriveEncoderPosition();
    }

    return summedDistance / m_modules.length;
  }

  private final double getMotorVelocity() {
    double summedVelocity = 0.0;

    for (SwerveModule module : m_modules) {
      summedVelocity += module.getDriveEncoderVelocity();
    }

    return summedVelocity / m_modules.length;
  }

  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  // See https://docs.google.com/document/d/10if4xjAaETTceUVn7l4J-jOCOnm5CJUDS5RAVNIJMQM/edit?tab=t.0 for everything below.
  // This is all rough, untested, and copied code that I would like to try but im not sure if it works.
  public LinearVelocity getVelocityMagnitude(ChassisSpeeds cs){
    return MetersPerSecond.of(new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
  }

  public final Rotation2d getVelocityDirection(Pose2d desiredPose) {
    ChassisSpeeds currentChassisSpeeds = getChassisSpeeds();

    return new Rotation2d(currentChassisSpeeds.vxMetersPerSecond, currentChassisSpeeds.vyMetersPerSecond);
  }

  public final Command alignToPose(Pose2d desiredPose) {
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      new Pose2d(m_poseEstimator.getEstimatedPosition().getTranslation(), getVelocityDirection(desiredPose)),
      desiredPose
    );

    if (waypoints.get(0).anchor().getDistance(waypoints.get(1).anchor()) > 0.01) {
      PathPlannerPath path = new PathPlannerPath(
        waypoints,
        new PathConstraints(1.0, 0.5, Math.PI / 6.0, Math.PI / 6.0),
        new IdealStartingState(getVelocityMagnitude(getChassisSpeeds()), m_pigeon2.getRotation2d()),
        new GoalEndState(0.0, desiredPose.getRotation())
      );

      path.preventFlipping = true;

      return AutoBuilder.followPath(path).andThen(
        new ApriltagAlignmentCommandV2(this, desiredPose)
      );
    }

    else {
      return new InstantCommand();
    }
  }

    // public static Pose2d getClosestReefAprilTag(Pose2d pose) {
    //     var alliance = DriverStation.getAlliance();
        
    //     ArrayList<Pose2d> reefPoseList;
    //     if (alliance.isEmpty()) {
    //         reefPoseList = allReefTagPoses;
    //     } else{
    //         reefPoseList = alliance.get() == Alliance.Blue ? 
    //             blueReefTagPoses :
    //             redReefTagPoses;
    //     }


    //     return pose.nearest(reefPoseList);

    // }
}