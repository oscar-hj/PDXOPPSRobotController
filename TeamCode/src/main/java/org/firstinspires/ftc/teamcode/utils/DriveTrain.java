package org.firstinspires.ftc.teamcode.utils;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.sin;

import android.os.Environment;
import android.util.Log;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.Constants;
import com.pedropathing.geometry.Pose;


public class DriveTrain {
    public enum DriveMode{
        OP_DRIVE,
        AUTO_POS
    }
    public DriveMode driveMode;
    public Follower follower;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Gamepad gamepad;
    public static DcMotor frontLeft, frontRight, backLeft, backRight;
    public static double deadzone = 0.05;

    /**
     * Initialize the Method inside the runOpMode method
     *
     * @param hwMap   Takes the hardwareMap of the robot
     * @param gp1     Takes the gamepad that is used to drive the robot (gamepad1)
     * @param tel     Takes the telemetry for the opMode
     */
    public DriveTrain(HardwareMap hwMap, Gamepad gp1, Telemetry tel) {
        this.hardwareMap = hwMap;
        this.gamepad = gp1;
        this.telemetry = tel;
    }

    /**
     * Initializes the motors
     *
     * @param FLM_NAME Name for Front Left Motor
     * @param FRM_NAME Name for Front Right Motor
     * @param BLM_NAME Name for Back Left Motor
     * @param BRM_NAME Name for Back Right Motor
     */
    public void init(String FLM_NAME, String FRM_NAME, String BLM_NAME, String BRM_NAME){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(loadPose());
        follower.update();

        frontLeft = hardwareMap.get(DcMotor.class, FLM_NAME);
        frontRight = hardwareMap.get(DcMotor.class, FRM_NAME);
        backLeft = hardwareMap.get(DcMotor.class, BLM_NAME);
        backRight = hardwareMap.get(DcMotor.class, BRM_NAME);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveMode = DriveMode.OP_DRIVE;
    }

    /**
     * Control:
     * <p></p>
     * Left stick drives the robot front, back, side to side, and diagonally. Right stick rotates
     * the robot. Left bumper activates slow mode, right bumper activates boost mode.
     */
    public void Drive2D(){
        double speed;
        double valx = gamepad.left_stick_x;
        double valy = gamepad.left_stick_y;
        double valr = gamepad.right_stick_x;

        if(gamepad.left_bumper){
            // slow speed
            speed = 0.1;
        } else if (gamepad.right_bumper) {
            // boost speed
            speed = 1;
        } else {
            // normal speed
            speed = 0.5;
        }

        if(abs(valx) > deadzone | abs(valy) > deadzone | abs(valr) > deadzone) {
            frontLeft.setPower((valy - valx - valr) * speed);
            frontRight.setPower((valy + valx + valr) * speed);
            backLeft.setPower((valy + valx - valr) * speed);
            backRight.setPower((valy - valx + valr) * speed);
        } else{
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }

    public void Drive2DField(){
        double speed;
        double valx = gamepad.left_stick_x;
        double valy = gamepad.left_stick_y;
        double valr = gamepad.right_stick_x;

        if(gamepad.left_bumper){
            // slow speed
            speed = 0.1;
        } else if (gamepad.right_bumper) {
            // boost speed
            speed = 1;
        } else {
            // normal speed
            speed = 0.5;
        }

        double theta = atan2(valx, valy);
        double r = hypot(valx, valy);

        theta = theta - follower.getHeading();

        double newValx = r * sin(theta);
        double newValy = r * cos(theta);

        if(abs(valx) > deadzone | abs(valy) > deadzone | abs(valr) > deadzone) {
            frontLeft.setPower((newValy - newValx - valr) * speed);
            frontRight.setPower((newValy + newValx + valr) * speed);
            backLeft.setPower((newValy + newValx - valr) * speed);
            backRight.setPower((newValy - newValx + valr) * speed);
        } else{
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }

    public void Drive2DFieldFreeLook(){
        double speed;
        double valx = gamepad.left_stick_x;
        double valy = gamepad.left_stick_y;
        double valrx = gamepad.right_stick_x;
        double valry = gamepad.right_stick_y;

        if(gamepad.left_bumper){
            // slow speed
            speed = 0.1;
        } else if (gamepad.right_bumper) {
            // boost speed
            speed = 1;
        } else {
            // normal speed
            speed = 0.5;
        }

        double theta = atan2(valx, valy);
        double r = hypot(valx, valy);

        theta = theta - follower.getHeading();

        double newValx = r * sin(theta);
        double newValy = r * cos(theta);

        if(valrx > 0.4 || valry > 0.4){
            double targetTheta = atan2(valrx, valry);
            double currentTheta = abs(follower.getHeading() % PI);
        }

        if(abs(valx) > deadzone | abs(valy) > deadzone | abs(valrx) > deadzone) {
            frontLeft.setPower((newValy - newValx - valrx) * speed);
            frontRight.setPower((newValy + newValx + valrx) * speed);
            backLeft.setPower((newValy + newValx - valrx) * speed);
            backRight.setPower((newValy - newValx + valrx) * speed);
        } else{
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }


    public void updatePose(){
        follower.update();
        telemetry.addData("Pose", follower.getPose());
    }

    public void savePose(){
        String path = Environment.getExternalStorageDirectory().getPath() + "/FIRST/pose.txt";
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double h = follower.getPose().getHeading();

        String output = String.format(Locale.ENGLISH, "%f,%f,%f", x, y, h);

        try{
            FileWriter writer = new FileWriter(path);
            writer.write(output);
            writer.close();
        } catch (IOException e){
            Log.e("DriveTrain", "Failed to save Pose", e);
        }
    }

    public Pose loadPose(){
        String path = Environment.getExternalStorageDirectory().getPath() + "/FIRST/pose.txt";
        try{
            BufferedReader reader = new BufferedReader(new FileReader(path));
            String line = reader.readLine();
            reader.close();

            String[] savedPose = line.split(",");
            double x = Double.parseDouble(savedPose[0]);
            double y = Double.parseDouble(savedPose[1]);
            double h = Double.parseDouble(savedPose[2]);

            return new Pose(x, y, h);
        } catch (IOException| NullPointerException | NumberFormatException e){
            Log.e("DriveTrain", "Loading pose failed, setting default pose", e);
            return new Pose(0, 0, 0);
        }
    }

    public void resetPose(){
        String path = Environment.getExternalStorageDirectory().getPath() + "/FIRST/pose.txt";
        double x = 0;
        double y = 0;
        double h = 0;

        String output = String.format(Locale.ENGLISH, "%f,%f,%f", x, y, h);

        try{
            FileWriter writer = new FileWriter(path);
            writer.write(output);
            writer.close();
        } catch (IOException e){
            Log.e("DriveTrain", "Failed to save Pose", e);
        }
    }


    public void printMotorTelemetry(Telemetry telemetry){
        telemetry.addData("Front Left", frontLeft.getPower());
        telemetry.addData("Front Right", frontRight.getPower());
        telemetry.addData("Back Left", backLeft.getPower());
        telemetry.addData("Back Right", backRight.getPower());
    }
}
