package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Attachments {
    HardwareMap hardwareMap;
    DcMotorEx outtakeMotorLeft, outtakeMotorRight;
    DcMotor intakeMotor;
    CRServo rollerServo, wristServo;
    public Attachments(HardwareMap hwMp){
        this.hardwareMap = hwMp;
    }

    public void initAttachments(){
        outtakeMotorLeft = hardwareMap.get(DcMotorEx.class, "oml");
        outtakeMotorRight = hardwareMap.get(DcMotorEx.class, "omr");

        outtakeMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outtakeMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        outtakeMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rollerServo = hardwareMap.get(CRServo.class, "roller");
        wristServo = hardwareMap.get(CRServo.class, "wrist");
    }


    /**
        @param rpm in rotations per second
     */
    public void runOuttake(double rpm){
        rpm *= 6;
        outtakeMotorLeft.setVelocity(rpm, AngleUnit.DEGREES);
        outtakeMotorRight.setVelocity(rpm, AngleUnit.DEGREES);
    }

    // roller servo methods
    public void forwardRoll(){
        rollerServo.setPower(1);
    }
    public void backwardsRoll(){
        rollerServo.setPower(-1);
    }


    // wrist servo methods
    public void outtakeAngle(){
        wristServo.setPower(1);
    }
    public void cubbyAngle(){
        wristServo.setPower(0);
    }

    // intake servo methods
    public void runIntake(double power){
        intakeMotor.setPower(power);
    }
}
