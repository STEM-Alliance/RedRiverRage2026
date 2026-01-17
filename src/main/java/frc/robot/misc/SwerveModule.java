// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.misc;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.DataLogHelpers;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class SwerveModule {
  private static final double kModuleMaxAngularVelocity = kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = kMaxAngularAcceleration;

  private int m_swerveIndex = -1;

  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;
  // private RelativeEncoder m_driveEncoder;
  // private RelativeEncoder m_turningEncoder;

  //private final AnalogInput m_absolutePos;
  //private final AnalogEncoder m_absolutePos;
  private final CANcoder m_cancoder;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(
    kDriveKp, kDriveKi, kDriveKd);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          kSwerveKp,
          kSwerveKi,
          kSwerveKd,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(
    kDriveKs, kDriveKv, kDriveKa);
  
  private SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(
    kSwerveKs, kSwerveKv);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   * @param driveEncoderChannelA DIO input for the drive encoder channel A
   * @param driveEncoderChannelB DIO input for the drive encoder channel B
   * @param turningEncoderChannelA DIO input for the turning encoder channel A
   * @param turningEncoderChannelB DIO input for the turning encoder channel B
   */
  public SwerveModule(
      int swerveIndex,
      int driveMotorChannel,
      int turningMotorChannel,
      int cancoderChannel) {
    m_swerveIndex = swerveIndex;
    m_driveMotor = new TalonFX(driveMotorChannel);
    m_turningMotor = new TalonFX(turningMotorChannel);
    m_cancoder = new CANcoder(cancoderChannel);

    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();

    currentLimits
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(80.0))
        .withSupplyCurrentLimit(Amps.of(80.0))
        .withSupplyCurrentLowerLimit(Amps.of(80.0));

    m_driveMotor.getConfigurator().apply(currentLimits);
    
    
    currentLimits
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(40.0))
        .withSupplyCurrentLimit(Amps.of(40.0))
        .withSupplyCurrentLowerLimit(Amps.of(40.0));
    m_turningMotor.getConfigurator().apply(currentLimits);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    // TODO: I think this needs to move into the PID controller of the spark max itself.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public final double getDriveEncoderPosition() {
    return m_driveMotor.getPosition().getValueAsDouble() * (2 * Math.PI * kWheelRadius / kDriveGearReduction);
  }

  public final double getDriveEncoderVelocity() {
    return m_driveMotor.getVelocity().getValueAsDouble() * (2 * Math.PI * kWheelRadius / kDriveGearReduction);
  }

  private final SwerveModuleState m_swerveModuleState = new SwerveModuleState();

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    m_swerveModuleState.speedMetersPerSecond = -getDriveEncoderVelocity();
    m_swerveModuleState.angle = new Rotation2d(getAbsPos());

    return m_swerveModuleState;
  }

  public TalonFX getDriveMotor() {
    return m_driveMotor;
  }

  public TalonFX getTurningMotor() {
    return m_turningMotor;
  }

  private final SwerveModulePosition m_swerveModulePosition = new SwerveModulePosition();

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    m_swerveModulePosition.distanceMeters = -getDriveEncoderPosition();
    m_swerveModulePosition.angle = new Rotation2d(getAbsPos());

    return m_swerveModulePosition;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    Rotation2d encoderRotation = new Rotation2d(getAbsPos());//m_turningEncoder.getPosition());

    // Optimizing the state prevents rotations more than 90 degrees.
    desiredState.optimize(encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.

    // This is only used in teleop (disabled in autonomous)
    // since it gives Pathplanner more precise translation control.
    //if (DriverStation.isTeleop()) {
    desiredState.speedMetersPerSecond *= desiredState.angle.minus(encoderRotation).getCos();
    //}

    // Calculate the drive output from the drive PID controller.
    final double drivePID =
        m_drivePIDController.calculate(getDriveEncoderVelocity(), desiredState.speedMetersPerSecond);

    final double driveFF = m_driveFeedforward.calculate(desiredState.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnPID =
      m_turningPIDController.calculate(getAbsPos(), desiredState.angle.getRadians());

    final double turnFF = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.set(MathUtil.clamp(drivePID + driveFF, -1, 1));
    m_turningMotor.set(MathUtil.clamp(turnPID + turnFF, -1, 1));

    DataLogHelpers.logDouble(getAbsPos(), "Swerve_rot_enc_" + m_swerveIndex);
    DataLogHelpers.logDouble(getDriveEncoderPosition(), "Swerve_drive_pos_" + m_swerveIndex);
    DataLogHelpers.logDouble(getDriveEncoderVelocity(), "Swerve_drive_vel_" + m_swerveIndex);
    SmartDashboard.putNumber("Swerve_drive_enc" + m_swerveIndex, getDriveEncoderPosition());
    SmartDashboard.putNumber("Swerve_drive_set" + m_swerveIndex, drivePID + driveFF);
  }

  public void setGains(double kp, double ki, double kd, double ks, double kv) {
    m_turningPIDController.setP(kp);
    m_turningPIDController.setI(ki);
    m_turningPIDController.setD(kd);

    m_turnFeedforward = new SimpleMotorFeedforward(ks, kv);
  }

  public final double getAbsPos() {
    return (m_cancoder.getAbsolutePosition().getValueAsDouble() + 0.5) * 2.0 * Math.PI;
  }

  public void setBrake(boolean enabled) {
    if (enabled) {
      m_driveMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    else {
      m_driveMotor.setNeutralMode(NeutralModeValue.Coast);
    }
  }
}
