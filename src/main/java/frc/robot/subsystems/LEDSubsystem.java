package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    // See https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf.
    public static enum kColors {
        STROBE_RED  (-0.11),
        STROBE_BLUE (-0.09),
        RED         (0.61),
        GREEN       (0.77),
        BLUE        (0.87);

        private final double m_color;

        kColors(double color) {
            m_color = color;
        }
        public double getValue() {return m_color;}
    }

    private final Spark m_blinkinController;

    public LEDSubsystem(int blinkinID) {
        m_blinkinController = new Spark(blinkinID);
        setAlliance();
    }

    public void setStrobeAlliance() {
        if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue) {
            setColor(kColors.STROBE_BLUE);
        }

        else {
            setColor(kColors.STROBE_RED);
        }
    }

    public void setGreen() {
        setColor(kColors.GREEN);
    }

    public void setAlliance() {
        if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue) {
            setColor(kColors.BLUE);
        }

        else {
            setColor(kColors.RED);
        }
    }

    public void setColor(kColors color) {
        m_blinkinController.set(color.getValue());
        SmartDashboard.putNumber("LEDSubsystem/BlinkinPWN", color.getValue());
    }
}
