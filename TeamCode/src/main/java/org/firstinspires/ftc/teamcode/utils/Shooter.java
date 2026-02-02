package org.firstinspires.ftc.teamcode.utils;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    DcMotorEx shooterMotor;
    CRServo hoodAngleServo;
    public Shooter(HardwareMap hwm, Telemetry tel){
        this.hardwareMap = hwm;
        this.telemetry = tel;
    }

    public void init(String motorName, String hoodServoName){
        shooterMotor = hardwareMap.get(DcMotorEx.class, motorName);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        hoodAngleServo = hardwareMap.get(CRServo.class, hoodServoName);
    }

    public void primeShooter(double rpm){
        rpm *= 28.0;
        rpm /= 60.0;

        telemetry.addData("Shooter Motor TPS", shooterMotor.getVelocity());
        telemetry.addData("Shooter Motor RPM", (shooterMotor.getVelocity()/28.0) * 60);

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
