package org.firstinspires.ftc.teamcode.testcode;

import static java.lang.Math.abs;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Intake Test")
public class intakeTest extends LinearOpMode {
    DcMotor FLM, FRM, BLM, BRM;
    DcMotor intakeMotor;
    //int
    double gp1LSX, gp1LSY, gp1RSX, gp1RSY, gp1LT, gp1RT, gp2LSY, gp2RSY, gp2LT, gp2RT, gp2LSX, gp2RSX;
    boolean gp1LB, gp1RB, gp1Dup, gp1Ddown, gp2DPadUp, gp2DPadDown, gp2DPadLeft, gp2DPadRight, gp2PS, gp2A, gp2B, gp2X, gp2Y, gp2LB, gp2RB, gp2Back;
    double motorSpeedgp1, motorSpeedgp2, slowSpeed = 0.1, normSpeed = 0.5, boostSpeed = 1, deadzone = 0.05;
    @Override
    public void runOpMode(){
        initAll();
        waitForStart();
        
        
        if(opModeIsActive()){
            while (opModeIsActive()){
                readGP1();
                readGP2();

                if(gp1LB){
                    motorSpeedgp1 = slowSpeed;
                } else if (gp1RB) {
                    motorSpeedgp1 = boostSpeed;
                } else {
                    motorSpeedgp1 = normSpeed;
                }

                // 2D movement and rotation
                if(abs(gp1LSX) > deadzone | abs(gp1LSY) > deadzone | abs(gp1RSX) > deadzone) {
                    drive2D(gp1LSX, gp1LSY, gp1RSX, motorSpeedgp1);
                } else{
                    if(gp1Dup){
                        drive2D(-1, 0, 0, motorSpeedgp1);
                    } else if(gp1Ddown){
                        drive2D(1, 0, 0, motorSpeedgp1);
                    }else stopDriveMotors();

                }


                if(gamepad1.left_trigger > deadzone){
                    intakeMotor.setPower(1);
                } else if (gamepad1.right_trigger > deadzone) {
                    intakeMotor.setPower(-1);
                } else{
                    intakeMotor.setPower(0);
                }
            }
        }
    }

    public void initAll(){
        BRM = hardwareMap.get(DcMotor.class, "br");
        FRM = hardwareMap.get(DcMotor.class, "fr");
        BLM = hardwareMap.get(DcMotor.class, "bl");
        FLM = hardwareMap.get(DcMotor.class, "fl");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        BLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BLM.setDirection(DcMotor.Direction.REVERSE);
        FLM.setDirection(DcMotor.Direction.REVERSE);


        BLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void readGP1(){
        /*
        Needs Adjustment
        Left Stick X,Y: Omnidirectional Movement
        Right Stick X: Turning

        Left Bumper: Slow Mode
        Right Bumper: Turbo Mode

        Left Trigger: Pivot Hooks away
        Right Trigger: Pivot Hooks towards
         */

        gp1LSX = gamepad1.left_stick_x;
        gp1LSY = gamepad1.left_stick_y;

        gp1RSX = gamepad1.right_stick_x;
        gp1RSY = gamepad1.right_stick_y;

        gp1LT = gamepad1.left_trigger;
        gp1RT = gamepad1.right_trigger;

        gp1LB = gamepad1.left_bumper;
        gp1RB = gamepad1.right_bumper;

        gp1Dup = gamepad1.dpad_up;
        gp1Ddown = gamepad1.dpad_down;
    }

    public void readGP2(){
        /*
        Needs Adjustment
        Left Bumper: Slow Mode
        Right Bumper: Turbo Mode

        Left Trigger: Pivot Slide Down
        Right Trigger: Pivot Slide Up
        PS: Auto pivot slide to specimen height

        D-Pad Up: Slide Up
        D-Pad Down: Slide Down

        A (Right Button): Open Claw
        B (Bottom Button): Close Claw

        Right Stick X: Rotate Claw
         */
        gp2LSY = gamepad2.left_stick_y;
        gp2LSX = gamepad2.left_stick_x;

        gp2RSY = gamepad2.right_stick_y;
        gp2RSX = gamepad2.left_stick_x;

        gp2LT = gamepad2.left_trigger;
        gp2RT = gamepad2.right_trigger;

        gp2RB = gamepad2.right_bumper;
        gp2LB = gamepad2.left_bumper;

        gp2DPadUp = gamepad2.dpad_up;
        gp2DPadDown = gamepad2.dpad_down;
        gp2DPadLeft = gamepad2.dpad_left;
        gp2DPadRight = gamepad2.dpad_right;

        gp2A = gamepad2.a;
        gp2B = gamepad2.b;
        gp2X = gamepad2.x;
        gp2Y = gamepad2.y;

        gp2PS = gamepad2.ps;

        gp2Back = gamepad2.back;
    }

    public void drive2D(double valx, double valy, double valr, double speed){
        FLM.setPower((valy - valx - valr) * speed);
        FRM.setPower((valy + valx + valr) * speed);
        BLM.setPower((valy + valx - valr) * speed);
        BRM.setPower((valy - valx + valr) * speed);

        getDriveMotorTele();
    }

    public void getDriveMotorTele(){
        telemetry.addData("frontLeft", FLM.getPower());
        telemetry.addData("frontRight", FRM.getPower());
        telemetry.addData("backLeft", BLM.getPower());
        telemetry.addData("backRight", BRM.getPower());
    }

    public void stopDriveMotors() {
        FLM.setPower(0);
        FRM.setPower(0);
        BLM.setPower(0);
        BRM.setPower(0);

        getDriveMotorTele();
    }
}
