package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.kElevatorSetpoints;
import frc.robot.Constants.kShooterSetpoints;
import frc.robot.util.DataLogHelpers;
import frc.robot.util.AprilTagFieldHelpers;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX m_elevatorMotor;
    private final SparkMax m_shooterMotor;
    private final StatusSignal<Angle> m_elevatorPosition;
    private final DigitalInput m_limitHigh = new DigitalInput(0);
    private final DigitalInput m_limitLow = new DigitalInput(3);
    private final AbsoluteEncoder m_intakePos;
    private final Supplier<Pose2d> m_getPoseSupplier;

    private double m_elevatorSetpoint = 0.0;
    private final ProfiledPIDController m_elevatorPID = new ProfiledPIDController(
        Constants.kElevatorKp, Constants.kElevatorKi, 0.0, new TrapezoidProfile.Constraints(Constants.kElevatorMaxVelocity, Constants.kElevatorMaxAcceleration)
    );

    private double m_shooterSetpoint = 0.0;
    private final ProfiledPIDController m_shooterPID = new ProfiledPIDController(
        Constants.kShooterKp, Constants.kShooterKi, 0.0, new TrapezoidProfile.Constraints(Constants.kShooterMaxVelocity, Constants.kShooterMaxVelocity)
    );

    // For double[]s, the first value is facing towards, the second value is facing away.
    private static final double m_climbSetpoint = 0.43;
    private static final double m_initialSetpoint = 0.04;
    private static final double[] m_l4Setpoints = new double[]{0.05, 0.34};
    private static final double[] m_l123Setpoints = new double[]{0.0400, 0.04 + 0.3};
    private static final double[] m_intakeSetpoints = new double[]{0.34, 0.03};
    private static final double m_idleSetpoint = 0.188;

    //private final IntakeSubsystem m_intake = new IntakeSubsystem(11, 0);
    // dio 1 and 3

    public ElevatorSubsystem(int elevatorMotorID, int shooterMotorID, Supplier<Pose2d> getPoseSupplier) {
        m_getPoseSupplier = getPoseSupplier;
        m_elevatorMotor = new TalonFX(elevatorMotorID);
        m_elevatorPosition = m_elevatorMotor.getPosition();

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();

        currentLimits
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(Amps.of(60.0))
            .withSupplyCurrentLimit(Amps.of(60.0))
            .withSupplyCurrentLowerLimit(Amps.of(60.0));
        
        m_elevatorMotor.getConfigurator().apply(currentLimits);

        m_shooterMotor = new SparkMax(shooterMotorID, MotorType.kBrushed);
        m_shooterMotor.getEncoder().setPosition(0.0);

        m_elevatorPID.setIZone(1);
        m_elevatorPID.setIntegratorRange(-0.3, 0.3);

        m_shooterPID.setIZone(0.15);
        m_shooterPID.setIntegratorRange(-0.5, 0.5);

        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        SparkMaxConfig shooterConfig = new SparkMaxConfig();

        elevatorConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.NeoLimit);
        // I tried to call the invert function and change the factor to -1. In both cases it crashes
        elevatorConfig.encoder
            // Magic numbers found through trial and error; ask Joe why our CAD isnt accurate.
            // (output is in inches, inches per second(?))
            .positionConversionFactor((1.5 / 0.54969295458888695567845703186279 * Math.PI * (25.0 / 12.0)) / 25.0)
            .velocityConversionFactor((1.5 / 0.54969295458888695567845703186279 * Math.PI * (25.0 / 12.0)) / 25.0 / 60.0);
        
        shooterConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.WindowLimit);
        shooterConfig.encoder
            .positionConversionFactor(1.0)
            .velocityConversionFactor(1.0);
        shooterConfig.limitSwitch.
            forwardLimitSwitchEnabled(false).
            reverseLimitSwitchEnabled(false);

        //m_elevatorMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_intakePos = m_shooterMotor.getAbsoluteEncoder();
        setShooterSetpoint(kShooterSetpoints.IDLE);

        m_elevatorPID.setTolerance(0.5);
        m_shooterPID.setTolerance(0.025);
        m_shooterPID.enableContinuousInput(0.0, 1.0);
        m_shooterPID.setGoal(m_intakePos.getPosition());
        setDefaultCommand(getControlLoopCommand());
    }

    public void periodic() {
        m_elevatorPosition.refresh();
        
        SmartDashboard.putNumber("ElevatorSubsystem/Elevator Current", m_elevatorMotor.getStatorCurrent().refresh().getValueAsDouble());
        SmartDashboard.putNumber("ElevatorSubsystem/Shooter Current", m_shooterMotor.getOutputCurrent());
        DataLogHelpers.logDouble(m_shooterMotor.getMotorTemperature(), "ElevatorSubsystem/Shooter Temperature");
        SmartDashboard.putNumber("ElevatorPos", -m_elevatorPosition.getValue().in(Rotations) * (1.5 / 0.54969295458888695567845703186279 * Math.PI * (25.0 / 12.0)) / 25.0 / 60.0);
        SmartDashboard.putBoolean("LimitLow", m_limitLow.get());
        SmartDashboard.putBoolean("LimitHigh", m_limitHigh.get());
        SmartDashboard.putNumber("IntakePos", m_intakePos.getPosition());
    }

    public final Command getControlLoopCommand() {
        return new RunCommand(
            () -> {elevatorControlLoop(); shooterControlLoop();},
            this
        ).withName("ElevatorCmd");
    }

    public final void setElevatorSetpoint(kElevatorSetpoints setpoint) {
        m_elevatorSetpoint = setpoint.getAsDouble();
        m_elevatorPID.setGoal(m_elevatorSetpoint);
    }

    public final Command setElevatorSetpointCommand(kElevatorSetpoints setpoint) {
        return new InstantCommand(() -> {setElevatorSetpoint(setpoint);});
    }

    private final void elevatorControlLoop() {
        /*
         * Negative motor speeds drive up and motor rotations will be negative
         * Positive motor speeds drive down and motor rotations will be positive.
         * If you zero the elevator at the bottom and drive to the top you will have a value of 60.8.
         * Rotations to distance from the ground:
         * 19.84375 - 47.25
         * 36.04199 - 63 3/4
         * 54.6875 - 82 1/2
         * 61.686 - 89 3/4
         */
        SmartDashboard.putData("ElevatorPID", m_elevatorPID);
        //double elevatorOutput = -MathUtil.clamp(m_elevatorPID.calculate(m_elevatorPosition.getValue().in(Rotations) * 1.484375);// * (1.5 / 0.54969295458888695567845703186279 * Math.PI * (25.0 / 12.0)) / 25.0 / 60.0), -0.6, 0.75);
        double elevatorOutput = -MathUtil.clamp(
            m_elevatorPID.calculate(-m_elevatorPosition.getValue().in(Rotations) * Constants.kElevatorMotorRotationsToInches), 
            -Constants.kElevatorMaxDrive,
            Constants.kElevatorMaxDrive);
        SmartDashboard.putNumber("elevatorOutput", elevatorOutput);

        DataLogHelpers.logDouble(elevatorOutput, "ElevatorSubsystem/Elevator PID Output");
        SmartDashboard.putNumber("Elevator Position", -m_elevatorPosition.getValue().in(Rotations) * Constants.kElevatorMotorRotationsToInches);// * (1.5 / 0.54969295458888695567845703186279 * Math.PI * (25.0 / 12.0)) / 25.0 / 60.0);
        SmartDashboard.putNumber("Error", m_elevatorPID.getPositionError());

        if ((elevatorOutput > 0.0) && (m_limitLow.get())) {
            m_elevatorMotor.set(elevatorOutput);
        }

        else if ((elevatorOutput < 0.0) && (m_limitHigh.get())) {
            m_elevatorMotor.set(elevatorOutput);
        }

        else {
            m_elevatorMotor.set(0.0);
        }
    }

    public final void setShooterSetpoint(kShooterSetpoints setpoint) {
        Pose2d currentPose = m_getPoseSupplier.get();
        Pose2d nearestTagPose;

        if (setpoint == kShooterSetpoints.INTAKE) {
            nearestTagPose = AprilTagFieldHelpers.getNearestStationPose(currentPose);
        }

        else {
            nearestTagPose = AprilTagFieldHelpers.getNearestReefPose(currentPose);
        }

        double rotationDifference = currentPose.getRotation().minus(nearestTagPose.getRotation()).getDegrees();
        // from -90 to -180 (wraps) and from 180 to 90, it shouldn't invert.
        // from -90 to 90 (continous), it should invert.

        int i;

        if (
            ((-90 > rotationDifference) && (rotationDifference > -180)) ||
            ((180 > rotationDifference) && (rotationDifference > 90))
        ) {
            i = 0;
        }

        else {
            i = 1;
        }

        switch (setpoint) {
            case CLIMB:
                m_shooterSetpoint = m_climbSetpoint;
                break;

            case INITIAL:
                m_shooterSetpoint = m_initialSetpoint;
                break;

            case L4:
                m_shooterSetpoint = m_l4Setpoints[i];
                break;

            case L123:
                m_shooterSetpoint = m_l123Setpoints[i];
                break;

            case INTAKE:
                m_shooterSetpoint = m_intakeSetpoints[i];
                break;

            case IDLE:
                m_shooterSetpoint = m_idleSetpoint;
                break;

            default:
                break;
        }

        SmartDashboard.putBoolean("ShooterFlipped", i == 1);

        m_shooterPID.setGoal(m_shooterSetpoint);
    }

    public final Command setShooterSetpointCommand(kShooterSetpoints setpoint) {
        return new InstantCommand(() -> {setShooterSetpoint(setpoint);});
    }

    private final void shooterControlLoop() {
        SmartDashboard.putData("ShooterPID", m_shooterPID);
        double shooterOutput = MathUtil.clamp(-m_shooterPID.calculate(m_intakePos.getPosition()), -0.8, 0.8);
        SmartDashboard.putNumber("ShooterOutput", shooterOutput);

        DataLogHelpers.logDouble(shooterOutput, "ElevatorSubsystem/Shooter PID Output");
        DataLogHelpers.logDouble(m_shooterSetpoint, "ElevatorSubsystem/Shooter PID Goal");

        m_shooterMotor.set(shooterOutput);
    }

    public final Command setState(
        kElevatorSetpoints elevatorSetpoint,
        kShooterSetpoints shooterSetpoint
    ) {
        return new InstantCommand(() -> {
            if (elevatorSetpoint != null) setElevatorSetpoint(elevatorSetpoint);
            if (shooterSetpoint != null) setShooterSetpoint(shooterSetpoint);
        });
    }

    public final Command setStateIdle() {
        return setState(kElevatorSetpoints.IDLE, kShooterSetpoints.IDLE);
    }

    public final Command zeroHeight() {
        return new FunctionalCommand(
            () -> {},
            () -> {m_elevatorMotor.setPosition(Rotations.of(0.0)); m_elevatorMotor.set(0.10);},
            interrupted -> stop(),
            () -> !m_limitLow.get(),
            this
        );
    }

    public final Command emergencyStop() {
        return new InstantCommand(() -> m_elevatorMotor.set(0.0), this);
    }

    public final Boolean stop()
    {
        m_elevatorMotor.set(0);
        //m_elevatorEncoder.setPosition(0.0); // TODO: Fix
        return true;
    }

    public final Command up() {
        return new FunctionalCommand(() -> {}, 
                                     ()->{m_elevatorMotor.set(-0.1);},
                                     interrupted -> stop(),
                                     ()->false);
    }

    public final Command down() {
        return new FunctionalCommand(() -> {}, 
                                     ()->{m_elevatorMotor.set(0.1);},
                                     interrupted -> stop(),
                                     ()->false);
    }

    public final Boolean stopRotate()
    {
        m_shooterMotor.set(0);
        return true;
    }

    public final Command cw() {
        return new FunctionalCommand(() -> {}, 
                                     ()->{m_shooterMotor.set(0.8);},
                                     interrupted -> stopRotate(),
                                     ()->false);
    }

    public final Command ccw() {
        return new FunctionalCommand(() -> {}, 
                                     ()->{m_shooterMotor.set(-0.8);},
                                     interrupted -> stopRotate(),
                                     ()->false);
    }

    public final Command atSetpoints() {
        return new FunctionalCommand(
            () -> {},
            () -> {},
            interrupted -> {},
            () -> (m_elevatorPID.atGoal() && m_shooterPID.atGoal())
        );
    }

    // public final Command startIntaking() {
    //     return new InstantCommand(() -> {
    //         setState(null, kShooterSetpoints.INTAKE);
    //     }).alongWith(m_intake.startIntaking());
    // }

    // public final Command stopIntaking() {
    //     return new InstantCommand(() -> {
    //         setState(null, kShooterSetpoints.IDLE);
    //     }).alongWith(m_intake.stopIntaking());
    // }

    // public final Command startShooting() {
    //     return new InstantCommand(() -> {
    //         setState(null, kShooterSetpoints.SHOOT);
    //     }).alongWith(m_intake.startShooting());
    // }

    // public final Command stopShooting() {
    //     return new InstantCommand(() -> {
    //         setState(null, kShooterSetpoints.IDLE);
    //     }).alongWith(m_intake.stopShooting());
    // }
}
