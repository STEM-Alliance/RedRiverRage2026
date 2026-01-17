package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    private final DoubleSolenoid m_climberSolenoid;
    private final DoubleSolenoid m_clawSolenoid;
    private final Compressor m_compressor;

    public ClimbSubsystem(
        int climberSolenoidFID, int climberSolenoidRID,
        int clawSolenoidFID, int clawSolenoidRID
    ) {
        m_climberSolenoid = new DoubleSolenoid(
            PneumaticsModuleType.REVPH, climberSolenoidFID, climberSolenoidRID);
        
        m_clawSolenoid = new DoubleSolenoid(
            PneumaticsModuleType.REVPH, clawSolenoidFID, clawSolenoidRID);
        
        m_compressor = new Compressor(PneumaticsModuleType.REVPH);
        m_compressor.enableDigital();

        m_climberSolenoid.set(Value.kReverse);
        m_clawSolenoid.set(Value.kForward);
    }

    public final Command toggleClimber() {
        return new InstantCommand(() -> {
            //System.out.println("toggleClimber");
            m_climberSolenoid.toggle();
        });
    }

    public final Command toggleClaw() {
        return new InstantCommand(() -> {
            //System.out.println("toggleClaw");
            m_clawSolenoid.toggle();
        });
    }
}
