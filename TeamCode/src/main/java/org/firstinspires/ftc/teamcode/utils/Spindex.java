package org.firstinspires.ftc.teamcode.utils;

import android.os.Environment;
import android.util.Log;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

public class Spindex {
    PIDController pidController = new PIDController(0.1, 2.5e-4, 2e-5, 1e-5);
    HardwareMap hardwareMap;
    Telemetry telemetry;
    DcMotorEx spinMotor;
    CRServo transferServo;
    TouchSensor magneticSwitch;
    DistanceSensor backSensor, frontSensor;
    ElapsedTime ballTime = new ElapsedTime();
    Shooter shooter;

    public boolean ballStoring = false;
    public boolean hasShot = false;
    public int homeValue;
    public DriveTrain driveTrain;
    public SpindexState spindexState;
    public Offset currentPos;

    public enum SpindexState {
        INTAKING,
        SHOOTING,
        MOVING
    }
    public enum Offset  {
        STORE1(1200),
        SHOOT1(3400),
        STORE2(4100),
        SHOOT2(6300),
        STORE3(6800),
        SHOOT3(9000);

        private final int offset;
        private Offset(final int offset) { this.offset = offset; }
        public int getOffset() { return this.offset; }
    }

    public Spindex(HardwareMap hwMp, Telemetry tele, DriveTrain driveTrain){
        this.hardwareMap = hwMp;
        this.telemetry = tele;
        this.driveTrain = driveTrain;
    }
    public Spindex(HardwareMap hwMp, Telemetry tele, Shooter shoot){
        this.hardwareMap = hwMp;
        this.telemetry = tele;
        this.shooter = shoot;
    }
    public Spindex(HardwareMap hwMp, Telemetry tele){
        this.hardwareMap = hwMp;
        this.telemetry = tele;
    }

    public void init(String spinMotorName, String transferServoName, String magneticSwitchName, String frontDistanceSensorName, String backDistanceSensorName, boolean doHome){
        spinMotor = hardwareMap.get(DcMotorEx.class, spinMotorName);
        magneticSwitch = hardwareMap.get(TouchSensor.class, magneticSwitchName);
        transferServo = hardwareMap.get(CRServo.class, transferServoName);
        frontSensor = hardwareMap.get(DistanceSensor.class, frontDistanceSensorName);
        backSensor = hardwareMap.get(DistanceSensor.class, backDistanceSensorName);

        spinMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        currentPos = Offset.STORE1;
        if(doHome){
            homeSpindex();
        }else{
            homeValue = loadHome();
        }
    }

    public void homeSpindex(){
        do{
            spinMotor.setPower(-0.5);
            telemetry.addLine("Homing...");
            telemetry.update();
        }while (magneticSwitch.isPressed());
        do{
            spinMotor.setPower(-0.5);
            telemetry.addLine("Homing...");
            telemetry.update();
        }while (!magneticSwitch.isPressed());

        spinMotor.setPower(0);
        telemetry.addLine("Home!");
        homeValue = spinMotor.getCurrentPosition() + 400;
        currentPos = Offset.STORE1;

        telemetry.update();
    }

    public void goToPos(Offset spinPos){
        int targetPos = homeValue + spinPos.getOffset();
        int currentPos = spinMotor.getCurrentPosition();
        double pidPower = -pidController.update(targetPos, currentPos, telemetry);
        int tolerance = 50;  // in ticks

        telemetry.addData("PIDPower", pidPower);


        if (!(currentPos < targetPos + tolerance && currentPos > targetPos - tolerance)){
            spindexState = SpindexState.MOVING;
            spinMotor.setPower(pidPower);
        }else{
            switch (spinPos){
                case STORE1:
                case STORE2:
                case STORE3:
                    spindexState = SpindexState.INTAKING;
                    driveTrain.driveMode = DriveTrain.DriveMode.OP_DRIVE;
                    break;
                case SHOOT1:
                case SHOOT2:
                case SHOOT3:
                    spindexState = SpindexState.SHOOTING;
                    driveTrain.driveMode = DriveTrain.DriveMode.AUTO_POS;
                    break;
            }
            spinMotor.setPower(0);
        }

        this.currentPos = spinPos;
    }

    public void nextPos(){
        switch (currentPos){
            case STORE1:
                currentPos = Offset.SHOOT1;
                break;
            case STORE2:
                currentPos = Offset.SHOOT2;
                break;
            case STORE3:
                currentPos = Offset.SHOOT3;
                break;
            case SHOOT1:
                currentPos = Offset.STORE2;
                break;
            case SHOOT2:
                currentPos = Offset.STORE3;
                break;
            case SHOOT3:
                currentPos = Offset.STORE3;
                break;
        }
    }

    public void manualSpindex(double direction){
        spinMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spinMotor.setPower(direction);
        telemetry.addData("Encoder Value", spinMotor.getCurrentPosition());
        telemetry.addData("Offset Value", (homeValue - spinMotor.getCurrentPosition()));
        telemetry.addData("Spindex Power", spinMotor.getPower());
        telemetry.addData("Switch Value", magneticSwitch.getValue());
    }

    public void intakeSpindex(){
        if ((frontSensor.getDistance(DistanceUnit.INCH) < 2 || backSensor.getDistance(DistanceUnit.INCH) < 2) && !(spindexState == SpindexState.MOVING)){
            if(ballStoring){
                if(ballTime.time() > 0.5) {
                    switch (currentPos) {
                        case STORE1:
                            currentPos = Offset.STORE2;
                            ballStoring = false;
                            break;
                        case STORE2:
                            currentPos = Offset.STORE3;
                            ballStoring = false;
                            break;
                        case STORE3:
                            currentPos = Offset.SHOOT3;
                            ballStoring = false;
                            break;
                    }
                }
            }else{
                ballTime.reset();
                ballStoring = true;
            }
        }
    }

    public void shootSpindex(){

        // If the spindex is not in position to shoot, don't run the function.
        if(spindexState != SpindexState.SHOOTING){
            return;
        }

        // If RMP drops below 4000 then the shooter has shot
        if(shooter.getRPM() < 4000){
            hasShot = true;
        }

        switch (currentPos){
            case SHOOT1:
                if(!hasShot){
                    activateTransfer(true);
                }else{
                    hasShot = false;
                    currentPos = Offset.SHOOT2;
                    activateTransfer(false);
                }
                break;

            case SHOOT2:
                if(!hasShot){
                    activateTransfer(true);
                }else{
                    hasShot = false;
                    currentPos = Offset.SHOOT3;
                    activateTransfer(false);
                }
                break;

            case SHOOT3:
                if(!hasShot){
                    activateTransfer(true);
                }else{
                    hasShot = false;
                    currentPos = Offset.STORE1;
                    activateTransfer(false);
                }
                break;
        }
    }

    public void saveHome(){
        String path = Environment.getExternalStorageDirectory().getPath() + "/FIRST/home.txt";
        double home = homeValue;

        String output = String.format(Locale.ENGLISH, "%f", home);

        try{
            FileWriter writer = new FileWriter(path);
            writer.write(output);
            writer.close();
        } catch (IOException e){
            Log.e("Spindex Homing", "Failed to save home value", e);
        }
    }

    public int loadHome(){
        String path = Environment.getExternalStorageDirectory().getPath() + "/FIRST/home.txt";
        try{
            BufferedReader reader = new BufferedReader(new FileReader(path));
            String line = reader.readLine();
            reader.close();

            return Integer.parseInt(line);
        } catch (IOException| NullPointerException | NumberFormatException e){
            Log.e("Spindex", "Loading home failed, setting default home!!!", e);
            return 0;
        }
    }

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

}
