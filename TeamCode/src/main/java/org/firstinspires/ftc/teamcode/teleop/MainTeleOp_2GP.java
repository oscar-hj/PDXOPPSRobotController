package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.abs;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "MainTeleOp_2GP")
public class MainTeleOp_2GP extends LinearOpMode{
    DcMotor FLM, FRM, BLM, backRightMotor;
    DcMotor pivotMotor, slideMotor, hookMotor;
    CRServo clawServo, pivotServo;
    HuskyLens lens;
    int pivotPos, specimenPickupPos, slidePos, slidePivotLock, slideLimit;
    double gp1LSX, gp1LSY, gp1RSX, gp1RSY, gp1LT, gp1RT, gp2LSY, gp2RSY, gp2LT, gp2RT, gp2LSX, gp2RSX;
    boolean gp1LB, gp1RB, gp1Dup, gp1Ddown, gp2DPadUp, gp2DPadDown, gp2DPadLeft, gp2DPadRight, gp2PS, gp2A, gp2B, gp2X, gp2Y, gp2LB, gp2RB, gp2Back;
    double motorSpeedgp1, motorSpeedgp2, slowSpeed = 0.1, normSpeed = 0.5, boostSpeed = 1, deadzone = 0.05;

    @Override
    public void runOpMode() {
        initAll();
        waitForStart();

        if(opModeIsActive()){
            pivotPos = pivotMotor.getCurrentPosition();
            slidePos = slideMotor.getCurrentPosition();

            slideMotor.setPower(slidePos);
            pivotMotor.setTargetPosition(pivotPos);

            specimenPickupPos = pivotMotor.getCurrentPosition() + 1340;
            slidePivotLock = pivotMotor.getCurrentPosition() + 400;
            slideLimit = (int) (slideMotor.getCurrentPosition() + (2500 * (537.7 / 1425.1)));
            while(opModeIsActive()) {
                //reads the gamepad inputs and assigns them to variables
                readGP1();
                readGP2();


                // GAMEPAD 1 CONTROL
                // sets speed
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
                    }
                    else if(gp1Ddown){
                        drive2D(1, 0, 0, motorSpeedgp1);
                    } else stopDriveMotors();

                }



                // rotate hooks
                if (gp1LT > deadzone){
                    hookMotor.setPower(gp1LT * motorSpeedgp1);
                } else if (gp1RT > deadzone){
                    hookMotor.setPower(-gp1RT * motorSpeedgp1);
                } else{
                    hookMotor.setPower(0);
                }


                //GAMEPAD 2 CONTROL
                // sets speed
                if(gp2LB){
                    motorSpeedgp2 = slowSpeed;
                } else if (gp2RB) {
                    motorSpeedgp2 = boostSpeed;
                } else {
                    motorSpeedgp2 = 0.3;
                }

                // resets pivot pos
                if(gp2Back){
                    specimenPickupPos = pivotMotor.getCurrentPosition() + 1380;
                    slidePivotLock = pivotMotor.getCurrentPosition() + 400;
                    slideLimit = slideMotor.getCurrentPosition() + 2500;
                }

                //pivot slide
                if (abs(gp2RSY) > deadzone){
                    if (gp2RSY > 0 && pivotPos >= specimenPickupPos){
                        lockPivotMotor(pivotPos);
                        continue;
                    }
                    pivotPos = drivePivotMotor(gp2RSY, motorSpeedgp2);
                } else if (!gp2PS){
                    lockPivotMotor(pivotPos);
                }

                // auto position slide (HOLD)
//                if (gp2PS){
//                    pivotPos = specimenPickupPos;
//                    lockPivotMotor(pivotPos);
//                }

                // slide
                if (gp2DPadUp) {
                    if (pivotMotor.getCurrentPosition() <= slidePivotLock){
                        slidePos = driveSlideMotor(1, 1);
                    } else if (pivotMotor.getCurrentPosition() >= slidePivotLock){
                        if (slideMotor.getCurrentPosition() >= slideLimit){
                            lockSlideMotor(slidePos);
                        } else {
                            slidePos = driveSlideMotor(1, 1);
                        }
                    } else{
                        lockSlideMotor(slidePos);
                    }
//                    if (pivotMotor.getCurrentPosition() <= slidePivotLock){
//                        slidePos = driveSlideMotor(1, 1);
//                    } else if (pivotMotor.getCurrentPosition() >= slidePivotLock && slideMotor.getCurrentPosition() <= slideLimit){
//                        slidePos = driveSlideMotor(1, 1);
//                    } else{
//                        lockPivotMotor(slidePos);
//                    }
                }else if(gp2DPadDown){
                    slidePos = driveSlideMotor(-1, 1);
                }else{
                    lockSlideMotor(slidePos);
                }

                // open/close claw
                if (gp2LT > deadzone){
                    clawServo.setPower(-0.2);
                } else if (gp2RT > deadzone) {
                    clawServo.setPower(1);
                }

                // rotate claw
                // vertical
                if(gp2Y){
                    pivotServo.setPower(0.8);
                }
                // horizontal
                if(gp2A){
                    pivotServo.setPower(0.05);
                }
                // 45 degrees left
                if(gp2X){
                    pivotServo.setPower(0.3);
                }
                // 45 degrees right
                if(gp2B){
                    pivotServo.setPower(-1);
                }

                telemetry.update();
            }
        }
    }


    public void getDriveMotorTele(){
        telemetry.addData("frontLeft", FLM.getPower());
        telemetry.addData("frontRight", FRM.getPower());
        telemetry.addData("backLeft", BLM.getPower());
        telemetry.addData("backRight", backRightMotor.getPower());
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
    public void initAll(){
        backRightMotor = hardwareMap.get(DcMotor.class, "BRM");
        FRM = hardwareMap.get(DcMotor.class, "FRM");
        BLM = hardwareMap.get(DcMotor.class, "BLM");
        FLM = hardwareMap.get(DcMotor.class, "FLM");
        pivotMotor = hardwareMap.get(DcMotor.class, "pivotMotor");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        hookMotor = hardwareMap.get(DcMotor.class, "hookMotor");

        BLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        FRM.setDirection(DcMotor.Direction.REVERSE);
//        pivotMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        BLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        clawServo = hardwareMap.get(CRServo.class, "clawServo");
        pivotServo = hardwareMap.get(CRServo.class, "pivotServo");

        lens = hardwareMap.get(HuskyLens.class, "lens");
        lens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    }

    public void drive2D(double valx, double valy, double valr, double speed){
        FLM.setPower((valy - valx - valr) * speed);
        FRM.setPower((valy + valx + valr) * speed);
        BLM.setPower((valy + valx - valr) * speed);
        backRightMotor.setPower((valy - valx + valr) * speed);

        getDriveMotorTele();
    }

    public int drivePivotMotor(double val, double speed) {
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotMotor.setPower(val * speed);
        telemetry.addData("pivotEncoder", pivotMotor.getCurrentPosition());
        return pivotMotor.getCurrentPosition();
    }

    public void lockPivotMotor(int pivotEncoderPosition){
        pivotMotor.setTargetPosition(pivotEncoderPosition);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("pivotEncoder", pivotEncoderPosition);
        pivotMotor.setPower(0.5);
    }

    public int driveSlideMotor(double val, double speed) {
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setPower(val * speed);
        telemetry.addData("slideMotor", slideMotor.getCurrentPosition());
        return slideMotor.getCurrentPosition();
    }

    public void lockSlideMotor(int slideEncoderPosition){
        slideMotor.setTargetPosition(slideEncoderPosition);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("slideEncoder", slideEncoderPosition);
        slideMotor.setPower(1);
    }

    public void stopDriveMotors() {
        FLM.setPower(0);
        FRM.setPower(0);
        BLM.setPower(0);
        backRightMotor.setPower(0);

        getDriveMotorTele();
    }

/*
    public void autoRotate(){
        HuskyLens.Block[] blockArr = lens.blocks();

        if (blockArr.length >= 2){
            telemetry.addLine("WARNING: THERE ARE 2 OR MORE SAMPLES DETECTED");
        }
    }
    public void doHusky(){
        /* Color IDs
        YELLOW: ??
        RED: ??
        BLUE: ??
        * /
        HuskyLens.Block[] blockArr = lens.blocks();

        for (HuskyLens.Block block : blockArr) {
            telemetry.addData("Block", block.toString());
        }
    }
*/
}