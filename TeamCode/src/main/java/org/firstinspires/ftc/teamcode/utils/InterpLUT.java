package org.firstinspires.ftc.teamcode.utils;

import java.util.TreeMap;


/**
 * A Look-Up Table (LUT) that uses linear interpolation to estimate
 * values for points not explicitly defined in the table.
 */
public class InterpLUT {

    // TreeMap automatically sorts the keys (x-values), which is essential for interpolation.
    private final TreeMap<Double, Double> table = new TreeMap<>();

    /**
     * Adds a data point (key and value) to the look-up table.
     * @param key The x-value (input).
     * @param value The y-value (output).
     */
    public void add(double key, double value) {
        table.put(key, value);
    }

    /**
     * Retrieves an interpolated value for a given key.
     * @param key The x-value for which to find the corresponding y-value.
     * @return The interpolated y-value.
     */
    public double get(double key) {
        if (table.isEmpty()) {
            throw new IllegalStateException("LUT is empty. Cannot interpolate.");
        }

        // Check if the key is an exact match
        if (table.containsKey(key)) {
            return table.get(key);
        }

        // Find the nearest keys below and above the given key
        Double lowerKey = table.floorKey(key);
        Double higherKey = table.ceilingKey(key);

        // Handle extrapolation (key is outside the defined range)
        if (lowerKey == null) {
            return table.ceilingEntry(key).getValue(); // Use the smallest value
        }
        if (higherKey == null) {
            return table.floorEntry(key).getValue(); // Use the largest value
        }

        // Get the values for the surrounding keys
        Double lowerValue = table.get(lowerKey);
        Double higherValue = table.get(higherKey);

        // Perform linear interpolation
        return interpolate(lowerKey, lowerValue, higherKey, higherValue, key);
    }

    /**
     * Helper method to perform linear interpolation between two points.
     * The equation is a + (b - a) * t, where t is the fraction between a and b.
     */
    private double interpolate(double x1, double y1, double x2, double y2, double x) {
        // Calculate the fraction (t)
        double fraction = (x - x1) / (x2 - x1);
        // Perform the interpolation calculation
        return y1 + (y2 - y1) * fraction;
    }

    /*
    public static void main(String[] args) {
        InterpolatingLUT table = new InterpolatingLUT();

        // Add data points
        table.add(0.0, 0.0);
        table.add(1.0, 10.0);
        table.add(2.0, 30.0);
        table.add(3.0, 50.0);

        // Get values (interpolation and direct lookup)
        double val1 = table.get(0.5); // Interpolate between (0.0, 0.0) and (1.0, 10.0)
        double val2 = table.get(1.5); // Interpolate between (1.0, 10.0) and (2.0, 30.0)
        double val3 = table.get(2.0); // Direct lookup
        double val4 = table.get(3.5); // Extrapolation (uses value at 3.0)

        System.out.println("Value at 0.5: " + val1); // Expected: 5.0
        System.out.println("Value at 1.5: " + val2); // Expected: 20.0
        System.out.println("Value at 2.0: " + val3); // Expected: 30.0
        System.out.println("Value at 3.5: " + val4); // Expected: 50.0
    }
    */
}