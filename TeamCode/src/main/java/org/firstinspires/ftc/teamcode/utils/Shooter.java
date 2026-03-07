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
    InterpLUT lookUpTable = new InterpLUT();

    public static double P = 200;
    public static double I = 1.5;
    public static double D = 10;
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

        createPoints();
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

    public void createPoints(){
        this.lookUpTable.add(34, 2800);
        this.lookUpTable.add(40, 2900);
        this.lookUpTable.add(55, 3100);
        this.lookUpTable.add(60, 3200);
        this.lookUpTable.add(70, 3300);
    }


    public int LUTToRPM(double distance){
        return (int) this.lookUpTable.get(distance);
    }

    public int distanceToRPM(double distance){
        if(distance < 40){
            return 2900;
        }else if(distance < 55){
            return 3000;
        }else if(distance < 60){
            return 3100;
        }else if(distance < 70){
            return 3200;
        }else if(distance > 70){
            return 3300;
        }else{
            return 2800;
        }
    }
}
