package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    DcMotorEx shooterMotor;
    Servo hoodAngleServo;
    public Shooter(HardwareMap hwm, Telemetry tel){
        this.hardwareMap = hwm;
        this.telemetry = tel;
    }

    public void init(String motorName, String hoodServoName){
        shooterMotor = hardwareMap.get(DcMotorEx.class, motorName);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        hoodAngleServo = hardwareMap.get(Servo.class, hoodServoName);
    }

    public void primeShooter(int rpm){
        rpm *= 6;
        shooterMotor.setVelocity(rpm);
    }
    
    public void changeAngle(double angle){
        double targetAngle = (angle - 3) / 42.0;
        hoodAngleServo.setPosition(targetAngle);
    }

    public double getRPM(){
        return shooterMotor.getVelocity();
    }
}
