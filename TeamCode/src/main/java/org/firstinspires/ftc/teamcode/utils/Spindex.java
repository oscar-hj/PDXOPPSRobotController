package org.firstinspires.ftc.teamcode.utils;
import org.firstinspires.ftc.teamcode.utils.ColorSensorRev.DetectedColor;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

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
    public int homePos = 0;
    
    // The list goes: store position 1 2 3 and then outtake position 1 2 3
    public int[] spindexEncoderOffsets = {0, 0, 0, 0, 0, 0};
    public DetectedColor[] memorizedStorage = {DetectedColor.UNKNOWN, DetectedColor.UNKNOWN, DetectedColor.UNKNOWN};
    public SpindexState spindexState;

    HardwareMap hardwareMap;
    Telemetry telemetry;
    DcMotorEx spinMotor;
    CRServo transferServo;
    TouchSensor magneticSwitch;
    ColorSensorRev colorSensor = new ColorSensorRev();

    public Spindex(HardwareMap hwMp, Telemetry tele){
        hardwareMap = hwMp;
        telemetry = tele;
    }

    public void init(String spinMotorName, String transferServoName, String colorSensorName, String magneticSwitchName){
        spinMotor = hardwareMap.get(DcMotorEx.class, spinMotorName);
        magneticSwitch = hardwareMap.get(TouchSensor.class, magneticSwitchName);
        transferServo = hardwareMap.get(CRServo.class, transferServoName);

        spinMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        homeSpindex();
//        gotoPosition(SpindexState.STORE1);

        colorSensor.init(hardwareMap, colorSensorName);
    }

    /*
     *    Spindex Methods
     */

    public void homeSpindex(){
        spinMotor.setPower(-0.1);
        while (!magneticSwitch.isPressed()){
            telemetry.addLine("Homing...");
            telemetry.update();
        }
        spinMotor.setPower(0);
        homePos = spinMotor.getCurrentPosition();
    }

    public void intakeSpindex(boolean fastMode){
        // detect current color
        DetectedColor detectedColor = colorSensor.getDetectedColor(telemetry);


        // If color is detected, set color of ball into the memorized storage
        switch (detectedColor){
            case BLUE:
                this.memorizedStorage[spindexState.ordinal()] = DetectedColor.BLUE;
                break;
            case GREEN:
                this.memorizedStorage[spindexState.ordinal()] = DetectedColor.GREEN;
                break;
            case UNKNOWN:
                if(fastMode){
                    this.memorizedStorage[spindexState.ordinal()] = DetectedColor.UNKNOWN;
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

    public void gotoPosition(SpindexState targetState){
        spinMotor.setTargetPosition(homePos + spindexEncoderOffsets[targetState.ordinal()]);
    }

    public void manualSpindex(double direction){
        spinMotor.setPower(direction);
        telemetry.addData("Encoder Value", spinMotor.getCurrentPosition());
        telemetry.addData("Spindex Power", spinMotor.getPower());
        telemetry.addData("Switch Value", magneticSwitch.getValue());
    }

    /*
     *  Transfer Methods
     */
    public void activateTransfer(boolean direction){
        if (direction){
            transferServo.setPower(-1);
        }else{
            transferServo.setPower(1);
        }
    }

    public void deactivateTransfer(){
        transferServo.setPower(0);
    }

    public void setStorage(DetectedColor pos1, DetectedColor pos2, DetectedColor pos3){
        memorizedStorage = new DetectedColor[]{pos1, pos2, pos3};
    }

}
