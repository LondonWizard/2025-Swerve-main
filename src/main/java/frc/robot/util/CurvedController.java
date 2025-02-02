package frc.robot.util;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
/**
 * The ScaleInput class provides methods for scaling and curving the input
 * values,
 * typically from joystick inputs, to allow for more precise control of the
 * robot.
 */
public class CurvedController {
    CommandXboxController con;

    public CurvedController(CommandXboxController con) {
        this.con = con;
    }

    /**
     * Applies an exponential curve to the input to enhance precision for small
     * values.
     * 
     * @param input     The input value to be curved, expected to be in the range
     *                  [-1, 1].
     * @param intensity The intensity of the curve, a higher value makes the curve
     *                  steeper, expected to be in the range [0, 20].
     * @return The curved input value, still in the range [-1, 1].
     */
    private static double curve(double input, double intensity) {
        // Apply an exponential function to the input value to scale it non-linearly
        return Math.exp((Math.abs(input) - 1) * intensity) * input;
    }

    public Vector<N2> curvedTranslation() {
        final double x = con.getLeftX();
        final double y = con.getLeftY();
        final double magnitude = Math.sqrt(x * x + y * y);
        final double newMagnitude = curve(magnitude, 1.5);
        final double ratio = newMagnitude / magnitude;
        return VecBuilder.fill(x * ratio, y * ratio);
    }

    public double curvedTranslationX() {
        return curvedTranslation().get(0);
    }

    public double curvedTranslationY() {
        return curvedTranslation().get(1);
    }

    public double curvedRotation() {
        return curve(con.getRightX(), 0.3);
    }
}
