package org.firstinspires.ftc.teamcode.utils;

import static java.lang.Math.abs;

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
    public Follower follower;
    public HardwareMap hardwareMap;
    public Gamepad gamepad;
    DcMotor frontLeft, frontRight, backLeft, backRight;

    /**
     * Initialize the Method inside the runOpMode method
     *
     * @param hardwareMap   Takes the hardwareMap of the robot
     * @param gp1           Takes the gamepad that is used to drive the robot (gamepad1)
     */
    public DriveTrain(HardwareMap hardwareMap, Gamepad gp1) {
        this.hardwareMap = hardwareMap;
        this.gamepad = gp1;
    }

    /**
     * Initializes the motors using the names; fl, fr, bl, br.
     */
    public void initMotors(){
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");

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

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(loadPose());
        follower.startTeleopDrive();
        follower.update();
    }

    /**
     * Control:
     * <p>
     * Left stick drives the robot front, back, side to side, and diagonally. Right stick rotates
     * the robot. Left bumper activates slow mode, right bumper activates boost mode.
     */
    public void Drive2D(){
        double speed;
        double deadzone = 0.05;
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


    public void updatePose(Telemetry telemetry){
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
        } catch (IOException | NumberFormatException e){
            Log.e("DriveTrain", "Loading pose failed, setting default pose", e);
            return new Pose(0, 0, 0);
        }
    }

}
