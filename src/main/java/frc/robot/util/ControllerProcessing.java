package frc.robot.util;

import static frc.robot.Constants.*;

import edu.wpi.first.math.MathUtil;

public final class ControllerProcessing {
    private static final double[] m_processedTranslation = new double[]{0.0, 0.0};
    
    /**
     * Processing for the drivers translation inputs, applies a deadband to the normal of a 
     * {@code Translation2d} using the driver's x and y inputs, as well as applying a scaling
     * function if kControllerScaling is {@code true}.
     * 
     * @param x The unprocessed controller x input.
     * @param y The unprocessed controller y input.
     * @return A {@code Translation2d} with the processed controller x and y inputs.
    */
    public static final double[] getProcessedTranslation(double x, double y) {
        // Since the translation uses two axis, the processing is slightly more complex than the
        // omega processing. Instead of directly using the deadband and scaling, they are applied
        // to the normal of (x, y), and then the normal is applied to the x and y individually.
        double unprocessedNorm = Math.hypot(x, y);

        double processedNorm = MathUtil.applyDeadband(unprocessedNorm, kControllerDeadband);
        processedNorm = kControllerScaling ? scaledInput(processedNorm) : processedNorm;

        // Something something floating point imprecision (probably unnecessary).
        m_processedTranslation[0] = MathUtil.clamp(x * processedNorm, -1.0, 1.0);
        m_processedTranslation[1] = MathUtil.clamp(y * processedNorm, -1.0, 1.0);

        DataLogHelpers.logDouble(x, "ControllerProcessing/X");
        DataLogHelpers.logDouble(y, "ControllerProcessing/Y");
        DataLogHelpers.logDouble(m_processedTranslation[0], "ControllerProcessing/ProcessedX");
        DataLogHelpers.logDouble(m_processedTranslation[1], "ControllerProcessing/ProcessedY");

        return m_processedTranslation;
    }

    /**
     * Processing for the drivers omega input, applies a deadband to the input, as
     * well as applying a scaling function if kControllerScaling is {@code true}.
     * 
     * @param omega The unprocessed controller omega input.
     * @return A {@code double} with the processed controller omega input.
    */
    public static final double getProcessedOmega(double omega) {
        // Since the omega only uses one axis, the processing can directly use the deadband and scaling.
        double processedOmega = MathUtil.applyDeadband(omega, kControllerDeadband);
        processedOmega = kControllerScaling ? scaledInput(processedOmega) : processedOmega;

        // Something something floating point imprecision (probably unnecessary).
        processedOmega = MathUtil.clamp(processedOmega, -1.0, 1.0);

        DataLogHelpers.logDouble(omega, "ControllerProcessing/Omega");
        DataLogHelpers.logDouble(processedOmega, "ControllerProcessing/ProcessedOmega");

        return processedOmega;
    }

    /**
     * Scales a given input using the function 0.7x^3 + 0.3x, where x is the input.
     * 
     * @param input The input to be scaled.
     * @return A {@code double} with the scaled input.
    */
    private static final double scaledInput(double input) {
        double scaledInput = Math.pow(
            (0.7 * input), 3.0
        ) + (0.3 * input);

        // If the function is an even function where f(x) = f(-x), then this should be
        // included. This will make the sign of the function's output match the input.
        //scaledInput *= Math.signum(input);

        return scaledInput;
    }
}
