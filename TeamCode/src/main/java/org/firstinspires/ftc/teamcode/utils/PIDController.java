package org.firstinspires.ftc.teamcode.utils;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDController {
    private final double Kp, Ki, Kd, Ks;
    private double integralSum = 0;
    private double lastError = 0;
    private double currentTarget = 0;
    private final ElapsedTime timer = new ElapsedTime();


    private static final double I_MAX = 0.7; // Max value for integral
    private static final double S_TOLERANCE = 5; // Tolerance to turn off static power in ticks

    // Sets sPID values when initializing
    public PIDController(double Ks, double Kp, double Ki, double Kd) {
        this.Ks = Ks;
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    // Gets the target position and the current position and returns output of sPID
    public double update(double target, double currentPos, Telemetry telemetry) {
        // Get delta time and reset timer, sets to 1ms if delta time is less
        double dt = timer.seconds();
        if (dt <= 0) dt = 1e-3;
        timer.reset();

        // Check if target has changed and reset integral sum
        if (currentTarget != target){
            integralSum = 0;
            lastError = 0;
            currentTarget = target;
        }

        // Calculate the error
        double error = target - currentPos;

        // Proportional term
        double proportional = Kp * error;

        // Integral term, clamping the output to the limit
        integralSum += error * dt;
        double integral = Ki * integralSum;
        integral = clamp(integral, -I_MAX, I_MAX);

        // Derivative term
        double derivative = (Kd * (error - lastError) / dt);
        lastError = error;

        // If the motor moves too fast, reset the integral
        if(abs(derivative) > 0.015){
            integralSum = 0;
        }

        // Calculate total output
        double output = proportional + integral + derivative;

        // Add power if out of tolerance
        if(abs(error) > S_TOLERANCE){
            output += Ks * Math.signum(error);
        }

        // Clamp the output
        output = clamp(output, -1, 1);

        // Output telemetry
        telemetry.addData("Error", error);
        telemetry.addData("P", proportional);
        telemetry.addData("I", integral);
        telemetry.addData("D", derivative);
        telemetry.addData("Output", output);
        return output;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}

