package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Spindex {
    //TODO: Change values of these
    public final double STOREPOS1 = 0.2;
    public final double STOREPOS2 = 0.2;
    public final double STOREPOS3 = 0.2;
    public final double OUTPOS1 = 0.2;
    public final double OUTPOS2 = 0.2;
    public final double OUTPOS3 = 0.2;
    public enum color {
        GREEN,
        PURPLE
    }

    HardwareMap hardwareMap;
    Telemetry telemetry;
    Servo spinServo;
    CRServo lowTransServo, highTransServo;

    public Spindex(HardwareMap hwMp, Telemetry tele){
        hardwareMap = hwMp;
        telemetry = tele;
    }

    public void init(String spinServoName, String lowTransferServoName, String highTransferServoName){
        spinServo = hardwareMap.get(Servo.class, spinServoName);
        lowTransServo = hardwareMap.get(CRServo.class, lowTransferServoName);
        highTransServo = hardwareMap.get(CRServo.class, highTransferServoName);
    }

    public void spinTo(double spinPos){
        //TODO: Set the positions
        spinServo.setPosition(spinPos);
    }

}
