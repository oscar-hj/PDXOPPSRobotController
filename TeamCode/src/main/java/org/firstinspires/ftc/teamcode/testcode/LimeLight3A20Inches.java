package org.firstinspires.ftc.teamcode.testcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Disabled
@TeleOp(name = "Limelight AprilTag Distance", group = "Test")
public class LimeLight3A20Inches extends LinearOpMode {

    private DcMotor FLM, FRM, BLM, BRM;
    //private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {

        Limelight3A limelight;

        // Map hardware
        FLM = hardwareMap.get(DcMotor.class, "FLM");
        FRM = hardwareMap.get(DcMotor.class, "FRM");
        BLM = hardwareMap.get(DcMotor.class, "BLM");
        BRM = hardwareMap.get(DcMotor.class, "BRM");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Set motor directions (adjust if robot moves wrong way)
        //FRM.setDirection(DcMotor.Direction.REVERSE);
        //BRM.setDirection(DcMotor.Direction.REVERSE);
        BLM.setDirection(DcMotor.Direction.REVERSE);
        FLM.setDirection(DcMotor.Direction.REVERSE);

        // Stop motors before start
        stopAllMotors();

        // Set Limelight pipeline to 8
        limelight.pipelineSwitch(8);
        limelight.start();

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                telemetry.addLine("AprilTag Detected");

                // Estimate distance in inches (Limelight gives targetArea, tx, ty, etc.)
                /////double distanceInches = result.getBotPose_TargetSpace()[2] * 39.37; // meters to inches
                // create formula from excel
                // double angleA1, angleA2
                // double cameraHeight_H1, aprilTagHeight_H2
                // double angleToRadian =  (angleA1+angleA2)*(Math.PI / 180)
                // distanceCal = (aprilTagHeight_H2 - cameraHeight_H1) / Math.tan(angleToRadian)

                double angleA1 = 26.9;
                double angleA2 = result.getTy();
                double cameraHeight_H1 = 13.9;
                double aprilTagHeight_H2 = 33.5;
                double angleToRadian =  (angleA1+angleA2)*(Math.PI / 180);
                double distanceCal = (aprilTagHeight_H2 - cameraHeight_H1) / Math.tan(angleToRadian);



                //telemetry.addData("Distance (in)", "%.2f", distanceInches);
                telemetry.addData("Distance (in)", "%.2f", distanceCal);

                Pose3D botpose = result.getBotpose();
                if (botpose != null) {
                    double x = botpose.getPosition().x;
                    double y = botpose.getPosition().y;
                    telemetry.addData("MT1 Location", "(" + x + ", " + y + ")");
                }
                //telemetry.addData("Botpose", botpose.toString());

                // Smooth drive logic with slower speed
                double targetDistance = 30.000; // target distance in inches
                double tolerance = 1;     // acceptable range
                double Kp = 0.25;            // proportional constant for slower movement
                double maxPower = 0.2;       // maximum motor power (slower)

                double error = distanceCal - targetDistance;

                if (Math.abs(error) > tolerance) {
                    // proportional motor power
                    double power = error * Kp;

                    // clamp power to maxPower
                    power = Math.max(-maxPower, Math.min(maxPower, power));

                    if (power > 0) {
                        driveForward(power);
                        //telemetry.addData("Action", "Moving Forward");
                    } else {
                        driveBackward(-power);
                        //telemetry.addData("Action", "Moving Backward");
                    }

                    //telemetry.addData("Motor Power", "%.2f", power);
                } else {
                    stopDriving();
                    //telemetry.addData("Action", "At Correct Distance (~2.5 in)");
                }


            } else {
                telemetry.addLine("No AprilTag detected");
                stopAllMotors();
            }



            telemetry.update();
        }

        stopAllMotors();
    }

    private void moveAll(double power) {
        FLM.setPower(power);
        FRM.setPower(power);
        BLM.setPower(power);
        BRM.setPower(power);
    }

    private void driveForward(double power) {
        FLM.setPower(power);
        FRM.setPower(power);
        BLM.setPower(power);
        BRM.setPower(power);
    }

    private void driveBackward(double power) {
        FLM.setPower(-power);
        FRM.setPower(-power);
        BLM.setPower(-power);
        BRM.setPower(-power);
    }

    private void stopDriving() {
        FLM.setPower(0);
        FRM.setPower(0);
        BLM.setPower(0);
        BRM.setPower(0);
    }

    private void stopAllMotors() {
        moveAll(0);
    }
}