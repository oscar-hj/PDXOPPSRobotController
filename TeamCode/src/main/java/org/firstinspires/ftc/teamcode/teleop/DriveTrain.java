package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.abs;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.Constants;
import com.pedropathing.geometry.Pose;


public class DriveTrain {
    public Follower follower;
    public HardwareMap hardwareMap;
    public static Pose startingPose;
    DcMotor frontLeft, frontRight, backLeft, backRight;

    public DriveTrain(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

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
        follower.setStartingPose(new Pose());
        follower.update();
        follower.startTeleopDrive();
    }

    /**
     *
     * @param valx      Takes the x value of the left joystick
     * @param valy      Takes the y value of the left joystick
     * @param valr      Takes the x value of the right joystick (for rotation)
     * @param speed     Takes a number 0-1 for a speed multiplier
     */
    public void Drive2D(double valx, double valy, double valr, double speed){
        double deadzone = 0.05;

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

    public void updatePedro(Telemetry telemetry){
        follower.update();
        telemetry.addData("Pose", follower.getPose());
    }

    public Pose getPose(){
        return follower.getPose();
    }

}
