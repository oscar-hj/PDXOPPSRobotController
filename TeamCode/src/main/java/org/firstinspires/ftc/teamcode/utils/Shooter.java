package org.firstinspires.ftc.teamcode.utils;

import static java.lang.Math.abs;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class Shooter {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    DcMotorEx shooterMotor;
    CRServo hoodAngleServo;

    public static double P = 150;
    public static double I = 6.5;
    public static double D = 20;
    public static double F = 0.7;

    public Shooter(HardwareMap hwm, Telemetry tel){
        this.hardwareMap = hwm;
        this.telemetry = tel;
    }

    // Initialize the shooter
    public void init(String motorName, String hoodServoName){
        shooterMotor = hardwareMap.get(DcMotorEx.class, motorName);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        hoodAngleServo = hardwareMap.get(CRServo.class, hoodServoName);
    }

    // Sets the shooter's RPM
    public void primeShooter(double rpm){
        shooterMotor.setVelocityPIDFCoefficients(P, I, D, F);
        telemetry.addData("Target RPM", -rpm);
        telemetry.addData("Shooter Motor RPM", (shooterMotor.getVelocity()/28.0) * 60);

        rpm *= 28.0;
        rpm /= 60.0;

        shooterMotor.setVelocity(-rpm);
    }

//    public void hoodAngle(double angle){
//        double targetAngle = (angle - 3) / 42.0;
////        hoodAngleServo.setPosition(targetAngle);
//    }

//    public void manualHoodAngle(double power){
//        hoodAngleServo.setPower(power);
//    }

    public double getRPM(){
        double rpm = shooterMotor.getVelocity();
        rpm *= 60;
        rpm /= 28;

        return abs(rpm);
    }

    public boolean isAtRPM(int targetRPM){
        return getRPM() > targetRPM;
    }
}
