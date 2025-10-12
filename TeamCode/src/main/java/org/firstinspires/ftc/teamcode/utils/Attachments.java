package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Attachments {
    HardwareMap hardwareMap;
    DcMotor outtakeMotorLeft, outtakeMotorRight;
    public Attachments(HardwareMap hwMp){
        this.hardwareMap = hwMp;
    }

    public void initAttachments(){
        outtakeMotorLeft = hardwareMap.get(DcMotor.class, "oml");
        outtakeMotorRight = hardwareMap.get(DcMotor.class, "omr");

        outtakeMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtakeMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        outtakeMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void activateOuttake(){
        outtakeMotorLeft.setPower(1);
        outtakeMotorRight.setPower(1);
    }

    public void deactivateOuttake(){
        outtakeMotorLeft.setPower(0);
        outtakeMotorRight.setPower(0);
    }
}
