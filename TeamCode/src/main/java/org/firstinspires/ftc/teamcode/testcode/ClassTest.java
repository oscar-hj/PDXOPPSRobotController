package org.firstinspires.ftc.teamcode.testcode;

import org.firstinspires.ftc.teamcode.teleop.DriveTrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Class Test")
public class ClassTest extends LinearOpMode {
    double motorSpeedgp1, motorSpeedgp2, slowSpeed = 0.1, normSpeed = 0.5, boostSpeed = 1;

    @Override
    public void runOpMode(){
        DriveTrain driveTrain = new DriveTrain(hardwareMap);
        driveTrain.initMotors();

        waitForStart();
        if(opModeIsActive()){
            while (opModeIsActive()){

                // sets the drive speed multiplier
                if(gamepad1.left_bumper){
                    motorSpeedgp1 = slowSpeed;
                } else if (gamepad1.right_bumper) {
                    motorSpeedgp1 = boostSpeed;
                } else {
                    motorSpeedgp1 = normSpeed;
                }

                // 2D Drive
                driveTrain.Drive2D(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, motorSpeedgp1);


                driveTrain.updatePedro(telemetry);
                telemetry.update();
            }
        }
    }

}
