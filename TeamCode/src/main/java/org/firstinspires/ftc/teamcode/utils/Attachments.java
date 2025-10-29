package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Attachments {
    HardwareMap hardwareMap;
    DcMotorEx outtakeMotorLeft, outtakeMotorRight;
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
    }


    /**
        @param rpm in rotations per second
     */
    public void runOuttake(double rpm){
        rpm *= 6;
        outtakeMotorLeft.setVelocity(rpm, AngleUnit.DEGREES);
        outtakeMotorRight.setVelocity(rpm, AngleUnit.DEGREES);
    }

}
