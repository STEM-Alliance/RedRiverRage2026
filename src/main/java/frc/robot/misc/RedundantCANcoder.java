package frc.robot.misc;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public final class RedundantCANcoder {
    private final CANcoder m_absoluteEncoder;
    private final RelativeEncoder m_relativeEncoder;

    private boolean m_absoluteEncoderConnected;
    private double m_absoluteEncoderRotation;
    private double m_relativeEncoderVelocity;

    private double m_rotation;

    public RedundantCANcoder(CANcoder absoluteEncoder, SparkMax motorController) {
        m_absoluteEncoder = absoluteEncoder;
        m_relativeEncoder = motorController.getEncoder();
    }

    public final void syncEncoders() {
        if (m_absoluteEncoder.isConnected()) {
            m_absoluteEncoderConnected = true;

            m_absoluteEncoderRotation = (m_absoluteEncoder.getAbsolutePosition().getValueAsDouble() +
                0.5) * 2 * Math.PI;
                
            m_relativeEncoderVelocity = m_relativeEncoder.getVelocity();

            m_rotation = m_absoluteEncoderRotation;
        }

        else {
            if (m_absoluteEncoderConnected) {
                m_absoluteEncoderConnected = false;

                double relativeEncoderCurrentVelocity = m_relativeEncoder.getVelocity();

                m_relativeEncoder.setPosition(
                    (m_absoluteEncoderRotation +
                    ((relativeEncoderCurrentVelocity + m_relativeEncoderVelocity) / 2.0 * 0.02))
                );

                reportDisconnected();
            }

            m_rotation = m_relativeEncoder.getPosition();
        }
    }

    private final void reportDisconnected() {
        DriverStation.reportError(
            "RedundantCANcoder CANcoder disconnected." +
            "CANcoder ID: " + m_absoluteEncoder.getDeviceID(),
            false
        );
    }

    public final double getRotation() {
        return m_rotation;
    }

    public final Rotation2d getRotation2D() {
        return new Rotation2d(getRotation());
    }
}
