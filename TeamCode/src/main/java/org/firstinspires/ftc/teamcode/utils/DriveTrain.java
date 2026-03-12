package org.firstinspires.ftc.teamcode.utils;

import static java.lang.Math.abs;

import android.os.Environment;
import android.util.Log;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.pedropathing.geometry.Pose;


@Configurable
public class DriveTrain {
    public enum DriveMode{
        OP_DRIVE,
        AUTO_POS
    }

    static public double k = 0.05;
    static public double p = 0.007;
    static public double i = 0;
    static public double d = 0;
    public double speedSlow, speedNorm, speedFast;
    public DriveMode driveMode;
    public Follower follower;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public PIDController anglePID = new PIDController(0, 0, 0, 0);
    public Gamepad gamepad;
    public static DcMotor frontLeft, frontRight, backLeft, backRight;
    public static double deadzone = 0.05;

    /**
     * Initialize the Method inside the runOpMode method for teleOp
     *
     * @param hwMap   Takes the hardwareMap of the robot
     * @param gp1     Takes the gamepad that is used to drive the robot (gamepad1)
     * @param tel     Takes the telemetry for the opMode
     */
    public DriveTrain(HardwareMap hwMap, Gamepad gp1, Telemetry tel, double slow, double norm, double fast) {
        this.hardwareMap = hwMap;
        this.gamepad = gp1;
        this.telemetry = tel;

        this.speedSlow = slow;
        this.speedNorm = norm;
        this.speedFast = fast;
    }

    /**
     * Initialize the Method inside the runOpMode method for AUTO
     *
     * @param hwMap   Takes the hardwareMap of the robot
     * @param tel     Takes the telemetry for the opMode
     */
    public DriveTrain(HardwareMap hwMap, Telemetry tel, Follower follower) {
        this.hardwareMap = hwMap;
        this.telemetry = tel;
        this.follower = follower;
    }

    /**
     * Initializes the motors and
     *
     * @param FLM_NAME Name for Front Left Motor
     * @param FRM_NAME Name for Front Right Motor
     * @param BLM_NAME Name for Back Left Motor
     * @param BRM_NAME Name for Back Right Motor
     */
    public void init(String FLM_NAME, String FRM_NAME, String BLM_NAME, String BRM_NAME){
        follower = Constants.createFollower(hardwareMap, FLM_NAME, FRM_NAME, BLM_NAME, BRM_NAME);
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
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

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
    public void Drive2D(GP gp){
        double speed;
        double valx = gp.LSX;
        double valy = gp.LSY;
        double valr = gp.RSX;

        if(gp.LB){
            // slow speed
            speed = speedSlow;
        } else if (gp.RB) {
            // boost speed
            speed = speedFast;
        } else {
            // normal speed
            speed = speedNorm;
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

    public void Drive2DWithAlign(double angleDifference, GP gp){
        double speed;
        double valx = gp.LSX;
        double valy = gp.LSY;
        double valr = gp.RSX;

        if(gp.LB){
            // slow speed
            speed = -speedSlow;
        } else if (gp.RB) {
            // boost speed
            speed = -speedFast;
        } else {
            // normal speed
            speed = -speedNorm;
        }

        double currAngle = Math.toDegrees(follower.getPose().getHeading());
        double targetAngle = currAngle - angleDifference;

        anglePID = new PIDController(k, p, i, d);

        // PID for wheels, tolerance in degrees
        double wheelPower = -anglePID.update(targetAngle, currAngle, 0.5, telemetry, false);

        if(abs(valx) > deadzone | abs(valy) > deadzone | abs(valr) > deadzone | abs(wheelPower) > 0.05) {
            frontLeft.setPower(-((valy - valx - valr) * speed) + wheelPower);
            frontRight.setPower(-((valy + valx + valr) * speed) - wheelPower);
            backLeft.setPower(-((valy + valx - valr) * speed) + wheelPower);
            backRight.setPower(-((valy - valx + valr) * speed) - wheelPower);
        } else{
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }

//    public void selfAlign(double angleDifference){
//        double x = follower.getPose().getX();
//        double y = follower.getPose().getY();
//        double currAngle = follower.getPose().getHeading();
//        double targetAngle = currAngle - Math.toRadians(angleDifference);
//
//        path = follower.pathBuilder()
//                .addPath(new Path(new BezierLine(follower::getPose, new Pose(x, y, targetAngle))))
//                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, targetAngle, 0.8))
//                .build();
//
//        follower.followPath(path, true);
////        follower.setTeleOpDrive(0, 0, targetAngle, false);
//        follower.update();
//    }

//    public void selfAlign(double angleDifference){
//        double currAngle = follower.getPose().getHeading();
//        double targetAngle = currAngle - Math.toRadians(angleDifference);
//
//        anglePID = new PIDController(k, p, i, d);
//
//        double wheelPower = -anglePID.update(targetAngle, currAngle, 0.5, telemetry, true);
//
//        frontLeft.setPower(wheelPower);
//        backLeft.setPower(wheelPower);
//        frontRight.setPower(-wheelPower);
//        backRight.setPower(-wheelPower);
//    }

//    public void Drive2DField(){
//        double speed;
//        double valx = gamepad.left_stick_x;
//        double valy = gamepad.left_stick_y;
//        double valr = gamepad.right_stick_x;
//
//        if(gamepad.left_bumper){
//            // slow speed
//            speed = 0.1;
//        } else if (gamepad.right_bumper) {
//            // boost speed
//            speed = 1;
//        } else {
//            // normal speed
//            speed = 0.5;
//        }
//
//        double theta = atan2(valx, valy);
//        double r = hypot(valx, valy);
//
//        theta = theta - follower.getHeading();
//
//        double newValx = r * sin(theta);
//        double newValy = r * cos(theta);
//
//        if(abs(valx) > deadzone | abs(valy) > deadzone | abs(valr) > deadzone) {
//            frontLeft.setPower((newValy - newValx - valr) * speed);
//            frontRight.setPower((newValy + newValx + valr) * speed);
//            backLeft.setPower((newValy + newValx - valr) * speed);
//            backRight.setPower((newValy - newValx + valr) * speed);
//        } else{
//            frontLeft.setPower(0);
//            frontRight.setPower(0);
//            backLeft.setPower(0);
//            backRight.setPower(0);
//        }
//    }

//    public void Drive2DFieldFreeLook(){
//        double speed;
//        double valx = gamepad.left_stick_x;
//        double valy = gamepad.left_stick_y;
//        double valrx = gamepad.right_stick_x;
//        double valry = gamepad.right_stick_y;
//
//        if(gamepad.left_bumper){
//            // slow speed
//            speed = 0.1;
//        } else if (gamepad.right_bumper) {
//            // boost speed
//            speed = 1;
//        } else {
//            // normal speed
//            speed = 0.5;
//        }
//
//        double theta = atan2(valx, valy);
//        double r = hypot(valx, valy);
//
//        theta = theta - follower.getHeading();
//
//        double newValx = r * sin(theta);
//        double newValy = r * cos(theta);
//
//        if(valrx > 0.4 || valry > 0.4){
//            double targetTheta = atan2(valrx, valry);
//            double currentTheta = abs(follower.getHeading() % PI);
//        }
//
//        if(abs(valx) > deadzone | abs(valy) > deadzone | abs(valrx) > deadzone) {
//            frontLeft.setPower((newValy - newValx - valrx) * speed);
//            frontRight.setPower((newValy + newValx + valrx) * speed);
//            backLeft.setPower((newValy + newValx - valrx) * speed);
//            backRight.setPower((newValy - newValx + valrx) * speed);
//        } else{
//            frontLeft.setPower(0);
//            frontRight.setPower(0);
//            backLeft.setPower(0);
//            backRight.setPower(0);
//        }
//    }


    public void updatePose(){
        follower.update();
//        telemetry.addData("Pose", follower.getPose());
    }

    public void savePose(Follower follow){
        String path = Environment.getExternalStorageDirectory().getPath() + "/FIRST/pose.txt";
        double x = follow.getPose().getX();
        double y = follow.getPose().getY();
        double h = follow.getPose().getHeading();

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

//    public void resetPose(){
//        String path = Environment.getExternalStorageDirectory().getPath() + "/FIRST/pose.txt";
//        double x = 0;
//        double y = 0;
//        double h = 0;
//
//        String output = String.format(Locale.ENGLISH, "%f,%f,%f", x, y, h);
//
//        try{
//            FileWriter writer = new FileWriter(path);
//            writer.write(output);
//            writer.close();
//        } catch (IOException e){
//            Log.e("DriveTrain", "Failed to save Pose", e);
//        }
//    }


//    public void printMotorTelemetry(Telemetry telemetry){
//        telemetry.addData("Front Left", frontLeft.getPower());
//        telemetry.addData("Front Right", frontRight.getPower());
//        telemetry.addData("Back Left", backLeft.getPower());
//        telemetry.addData("Back Right", backRight.getPower());
//    }
}
