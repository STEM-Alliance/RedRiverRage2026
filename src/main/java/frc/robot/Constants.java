// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /********************************************
     * Swerve and Controls
     *******************************************/
    // This controls the speed of the right to left slew rate. Large numbers mean
    // faster response
    static public double kVxSlewRateLimit = 6.0;
    static public double kVySlewRateLimit = 6.0;
    static public double kOmegaSlewRateLimit = 5.0;

    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kOperatorButtonPanelPort = 2;
    public static final double kControllerDeadband = 0.2;
    public static final boolean kControllerScaling = true;

    // Max speeds
    static public double kMaxSpeed = 5.1; // 5.1 meters per second
    static public double kMaxAutonomousSpeed = kMaxSpeed; // this ensures that the max speed 2.5
    // (configured in pathplanne app settings) is never exceeded, should match the pathplanner max.
    static public double kMaxAngularSpeed = 4.0 * Math.PI; // 2 rotations per second 3.5
    static public double kMaxAutonomousAngularSpeed = kMaxAngularSpeed;
    static public double kMaxAngularAcceleration = Math.pow(2.0 * Math.PI, 2);
    static public double GeneralDeadband = 0.2;

    public static final double kMaxAlignmentSpeed = 1.0;
    public static final double kMaxAlignmentAngularSpeed = Math.PI / 1.0;

    // PID and Feedforward gains for the drive motors
    // NOTE: We are running the drive motors in open loop mode, using only the feedfoward.
    static public double kDriveKp = 0.0625;
    static public double kDriveKi = 0;
    static public double kDriveKd = 0;
    static public double kDriveKs = 0.27074 / 12.0;
    static public double kDriveKv = 2.7069 / 12.0;
    static public double kDriveKa = 0.5075 / 12.0;

    // PID and feedforward gains for the swerve motors
    static public double kSwerveKp = 0.375; // 0.0.5
    static public double kSwerveKi = 0; // 0.1
    static public double kSwerveKd = 0; // 0
    static public double kSwerveKs = 0.02;
    static public double kSwerveKv = 0.005;
    static public double kSwerveKa = 0.0;

    public static final PPHolonomicDriveController kPathplannerDriveController = new PPHolonomicDriveController(
        new PIDConstants(7.5, 0.0, 0.0), // Translation PID constants
        new PIDConstants(2.25, 0.0, 0.0) // Rotation PID constants
    );

    // Swerve Hardware
    static public double kWheelRadius = Units.inchesToMeters(3.89 / 2.0);
    //static public double kDriveGearReduction = 6.12;
    static public double kDriveGearReduction = 6.3; // Yes I know this isn't correct
    //static public double kDriveGearReduction = 5.8225911761962503497808040294749;
    static public double kTurningGearReduction = 12.8;
    
    // FL, FR, BL, BR
    static public Translation2d[] kSwerveTranslations = new Translation2d[]{
        new Translation2d(0.276, 0.276),
        new Translation2d(0.276, -0.276),
        new Translation2d(-0.276, 0.276),
        new Translation2d(-0.276, -0.276)
    };

    // Module Index, Drive Motor Channel, Swerve Motor Channel, CANCoder Channel
    static public int[] kSwerveFLCanID = new int[]{0, 1, 2, 3};
    static public int[] kSwerveFRCanID = new int[]{1, 4, 5, 6};
    static public int[] kSwerveBLCanID = new int[]{2, 7, 8, 9};
    static public int[] kSwerveBRCanID = new int[]{3, 10, 11, 12};

    static public int kPigeon2CanID = 13;

    // NOTE: These seem to drift
    //static public double[] kZeroPosition = new double[]{0.963, 0.486, 0.070, 0.093};
    static public double[] kZeroPosition = new double[]{0.238, 0.2101, 0.704, 0.092};
    static public double kEncoderRes = 4096;

    /********************************************
     * Elevator and Shooter
     *******************************************/
    static public double kElevatorKp = 0.2125;
    static public double kElevatorKi = 0;
    //static public double kElevatorMotorRotationsToInches = 1.484375;
    static public double kElevatorMotorRotationsToInches = 1;
    static public double kElevatorMaxDrive = 1;
    static public double kElevatorMaxVelocity = 140;
    static public double kElevatorMaxAcceleration = 100;
    static public double kElevatorDistanceOffset = 27.2;

    static public enum kElevatorSetpoints {
        L4      (61),
        L3      (32.5 + 1.5),
        L2      (17.25 + 2.0),
        L1      (5),
        CLIMB   (8),
        INTAKE  (0),
        IDLE    (0);
        
        private final double m_elevatorSetpoint;

        kElevatorSetpoints(double elevatorSetpoint) {
            m_elevatorSetpoint = elevatorSetpoint;
        }

        public double getAsDouble() {return m_elevatorSetpoint;}
    }

    static public double kShooterKp = 25.0;
    static public double kShooterKi = 3.325;
    static public enum kShooterSetpoints {
        CLIMB,
        INITIAL,
        L4,
        L123,
        INTAKE,
        IDLE;
        // CLIMB      (0.52),
        // INITIAL    (0.652), //0.3638
        // L4_2       (0.94), //45
        // L4_1       (0.64),
        // L3_2       (0.96),
        // L3_1       (0.62),
        // L2_2       (0.96), 
        // L2_1       (0.62), 
        // L1_2       (0.96),
        // L1_1       (0.62),
        // INTAKE2    (0.948 + 0.015),
        // INTAKE1    (0.615 - 0.015),
        // IDLE       (0.796); //0.534

        // private final double m_shooterSetpoints;

        // kShooterSetpoints(double shooterSetpoint) {
        //     m_shooterSetpoints = shooterSetpoint;
        // }

        // public double getAsDouble() {return m_shooterSetpoints;}
    }

    static public double kShooterMaxVelocity = 2;
    static public double kShooterMaxAcceleration = kShooterMaxVelocity/0.15;

    static public int kRumbleTimer = 100;

    // Positive offsets to the right.
    static public double kAlignXDistanceLeft = 0.42;
    static public double kAlignYDistanceLeftFront = -0.266 + Units.inchesToMeters(1.0);
    static public double kAlignYDistanceLeftBack = -0.133;
    static public double kAlignXDistanceRight = 0.42;
    static public double kAlignYDistanceRightFront = 0.0575 + Units.inchesToMeters(0.65);
    static public double kAlignYDistanceRightBack = 0.0195 + Units.inchesToMeters(7.0);
    

    /********************************************
     * Alignment settings
     *******************************************/
    static public double kAlignmentXTolerance = 0.02;
    static public double kAlignmentYTolerance = 0.02;
    static public double kAlignmentRotTolerance= 0.03;
    static public double kAlignmentRotIZone = Units.degreesToRadians(12.5);
    static public double kAlignmentRotIntegrationZone = 0.1;

    static public enum kAlignmentOffsets {
        LEFT,
        RIGHT
    }

    /********************************************
     * Motor Current Limits
     * P = VI
     * I = P / V
     *******************************************/
    static public int NeoLimit = 50;
    static public int Neo550Limit = 30;
    static public int BagMotorLimit = 30; // Max power is 149 W, 12.4 A
    static public int M775ProLimit = 15; // Max power 347 W, 28.9 A
    static public int CIMSLimit = 28; // Max power 337 W, 28.0 A
    // https://firstwiki.github.io/wiki/denso-window-motor
    static public int WindowLimit = 15; // This seems safe
}
