package org.firstinspires.ftc.teamcode.utils;
import org.firstinspires.ftc.teamcode.utils.ColorSensorRev.DetectedColor;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Spindex {
    public enum SpindexState{
        STORE1,
        STORE2,
        STORE3,
        OUT1,
        OUT2,
        OUT3;
    }

    public DetectedColor[] memorizedStorage = {DetectedColor.UNKNOWN, DetectedColor.UNKNOWN, DetectedColor.UNKNOWN};
    public SpindexState spindexState;

    HardwareMap hardwareMap;
    Telemetry telemetry;
    Servo spinServo;
    CRServo lowTransServo, highTransServo;
    ColorSensorRev colorSensor = new ColorSensorRev();

    public Spindex(HardwareMap hwMp, Telemetry tele){
        hardwareMap = hwMp;
        telemetry = tele;
    }

    public void init(String spinServoName, String lowTransferServoName, String highTransferServoName){
        spinServo = hardwareMap.get(Servo.class, spinServoName);
        setPosition(SpindexState.STORE1);

        lowTransServo = hardwareMap.get(CRServo.class, lowTransferServoName);
        highTransServo = hardwareMap.get(CRServo.class, highTransferServoName);
        colorSensor.init(hardwareMap, "colorSensor");
    }

    public void intakeSpindex(boolean fastMode){
        // detect current color
        DetectedColor detectedColor = colorSensor.getDetectedColor(telemetry);

        // get storage index
        int storageIndex;
        switch (spindexState){
            case STORE1:
                storageIndex = 0;
                break;
            case STORE2:
                storageIndex = 1;
                break;
            case STORE3:
                storageIndex = 2;
                break;
            default:
                return;
        }

        // If color is detected, set color of ball into the memorized storage
        switch (detectedColor){
            case BLUE:
                this.memorizedStorage[storageIndex] = DetectedColor.BLUE;
                break;
            case GREEN:
                this.memorizedStorage[storageIndex] = DetectedColor.GREEN;
                break;
            case UNKNOWN:
                if(fastMode){
                    this.memorizedStorage[storageIndex] = DetectedColor.UNKNOWN;
                    telemetry.addLine("Color indexed as unknown");
                }else{
                    telemetry.addLine("Color not found.");
                }
                break;
            default:
                telemetry.addData("INTAKE ERROR!!!!! Detected Color", detectedColor);
                break;
        }
    }

    public void setPosition(SpindexState targetState){
        // TODO: Change servo positions
        switch (targetState){
            case STORE1:
                spinServo.setPosition(0.1);
                break;
            case STORE2:
                spinServo.setPosition(0.2);
                break;
            case STORE3:
                spinServo.setPosition(0.3);
                break;
            case OUT1:
                spinServo.setPosition(0.4);
                break;
            case OUT2:
                spinServo.setPosition(0.5);
                break;
            case OUT3:
                spinServo.setPosition(0.6);
                break;
        }
        this.spindexState = targetState;
    }

    public void activateTransfer(boolean direction){
        if (direction){
            lowTransServo.setPower(1);
            highTransServo.setPower(1);
        }else{
            lowTransServo.setPower(-1);
            highTransServo.setPower(-1);
        }
    }

    public void deactivateTransfer(){
        lowTransServo.setPower(0);
        highTransServo.setPower(0);
    }

    public void setStorage(DetectedColor pos1, DetectedColor pos2, DetectedColor pos3){
        memorizedStorage = new DetectedColor[]{pos1, pos2, pos3};
    }

}
