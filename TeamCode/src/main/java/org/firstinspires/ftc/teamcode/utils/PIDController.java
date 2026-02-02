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


    private static final double I_MAX = 0.7;
    private static final double S_TOLERANCE = 75;

    public PIDController(double Ks, double Kp, double Ki, double Kd) {
        this.Ks = Ks;
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public double update(double target, double state, Telemetry telemetry) {
        // Get delta time and reset timer
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
        double error = target - state;

        // Proportional term
        double proportional = Kp * error;

        // Integral term
        integralSum += error * dt;
        double integral = Ki * integralSum;
        integral = clamp(integral, -I_MAX, I_MAX);

        // Derivative term
        double derivative = (Kd * (error - lastError) / dt);
        lastError = error;

        if(abs(derivative) > 0.015){
            integralSum = 0;
        }

        // Calculate total output
        double output = proportional + integral + derivative;

        // Add power if not on target
        if(abs(error) > S_TOLERANCE){
            output += Ks * Math.signum(error);
        }

        // Prevent derivative from making output go negative
//        if (Math.signum(output) != Math.signum(error)
//                && Math.abs(error) > PID_TOLERANCE) {
//            output = 0;
//        }

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

