package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem.kColors;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX m_intakeMotor;
    private final TofDistanceSubsystem m_enterSensor;
    private final TofDistanceSubsystem m_exitSensor;
    private boolean m_previousCoralDetected = false;
    private boolean m_coralDetected = false;
    private int m_rumbleCounter = 0;
    private final CommandXboxController m_controller;
    private final LEDSubsystem m_ledSubsystem;
    private final Trigger m_exitSensorTrigger = new Trigger(this::isIntakeLoaded);

    public IntakeSubsystem(
        int intakeMotorID,
        int enterSensorID,
        int exitSensorID,
        LEDSubsystem ledSubsystem,
        CommandXboxController controller
    ) {
        m_intakeMotor = new TalonFX(intakeMotorID);
        m_enterSensor = new TofDistanceSubsystem(enterSensorID);
        m_exitSensor = new TofDistanceSubsystem(exitSensorID);
        m_ledSubsystem = ledSubsystem;
        m_controller = controller;

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();

        currentLimits
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(Amps.of(30.0))
            .withSupplyCurrentLimit(Amps.of(30.0))
            .withSupplyCurrentLowerLimit(Amps.of(30.0));

        m_intakeMotor.getConfigurator().apply(currentLimits);

        m_ledSubsystem.setColor(
            isIntakeLoaded() ? kColors.GREEN : kColors.STROBE_RED
        );

        new Trigger(this::isIntakeLoaded).onChange(new InstantCommand(() -> {
            syncLEDSubsystem();
        }));
    }

    public void periodic() {
        if (m_coralDetected & !m_previousCoralDetected)
        {
            m_rumbleCounter = Constants.kRumbleTimer;
        }
        m_previousCoralDetected = m_coralDetected;
        if (m_rumbleCounter > 0)
        {
            m_controller.getHID().setRumble(RumbleType.kBothRumble, 1.0);
            m_rumbleCounter--;
        }
        else 
        {
            m_controller.getHID().setRumble(RumbleType.kBothRumble, 0);
        }

        SmartDashboard.putNumber("IntakeCurrent", m_intakeMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("EnterDistance", m_enterSensor.get_distance());
        SmartDashboard.putNumber("ExitDistance", m_exitSensor.get_distance());
    }

    public final boolean checkAndStopIntake()
    {
        //if (m_tofSensorDebouncer.calculate(isIntakeLoaded()))
        if (isIntakeLoaded())
        {
            m_intakeMotor.set(0);
            return true;
        }
        return false;
    }

    public final Command startIntaking() {
        return new FunctionalCommand(
            () -> {m_intakeMotor.set(0.15);}, //System.out.println("Set 1.0");},
            () -> {},//System.out.println("Intaking...");},
            interrupted -> {m_intakeMotor.set(0.0);},
            () -> checkAndStopIntake(),
            this
        ).andThen(reverseIntakeCommand());
    }

    public final Command runIntake() {
        return new FunctionalCommand(
            () -> {m_intakeMotor.set(0.15);},
            () -> {},
            interrupted -> {m_intakeMotor.set(0.0);},
            () -> false
        ).andThen(reverseIntakeCommand());
    }

    public final Command stopIntaking() {
        return new InstantCommand(
            () -> {m_intakeMotor.set(0.0);},// System.out.println("Set 0.0");},
            this
        );
    }

    public final Command startShooting() {
        return new InstantCommand(
            () -> {m_intakeMotor.set(0.3);}, //System.out.println("Set 1.0");},
            this
        );
    }

    public final Command stopShooting() {
        return new InstantCommand(
            () -> {m_intakeMotor.set(0.0);}, //System.out.println("Set 0.0");},
            this
        );
    }

    private final Command reverseIntakeCommand() {
        return new InstantCommand(() -> {m_intakeMotor.set(-0.15);})
            .andThen(new WaitCommand(0.1))
            .andThen(new InstantCommand(() -> {m_intakeMotor.set(0.0);}));
    }

    // public final Command autonomousShoot() {
    //     return new FunctionalCommand(
    //         () -> m_intakeMotor.set(1.0),
    //         () -> {},
    //         interrupted -> {},
    //         () -> isIntakeNotLoaded(),
    //         this
    //     ).until
    // }

    public final boolean isIntakeLoaded() {
        if (m_enterSensor.is_within_threshold(75))
        {
            m_coralDetected = true;
        }
        else
        {
            m_coralDetected = false;
        }
        return m_coralDetected;
    }

    public final boolean isIntakeNotLoaded() {
        return !isIntakeLoaded();
    }

    public final void syncLEDSubsystem() {
        if (isIntakeLoaded()) {
            m_ledSubsystem.setGreen();
        }

        else {
            m_ledSubsystem.setAlliance();
        }
    }
}
