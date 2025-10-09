package org.firstinspires.ftc.teamcode.testcode;

import org.firstinspires.ftc.teamcode.utils.DriveTrain;
import org.firstinspires.ftc.teamcode.utils.GP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Class Test")
public class ClassTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        // Makes a DriveTrain object taking the hardwareMap and gamepad1, loads the pose, and
        // initializes motors and PedroPathing follower (for tracking pose).
        DriveTrain driveTrain = new DriveTrain(hardwareMap, gamepad1);
        driveTrain.loadPose();
        driveTrain.initMotors();

        // makes 2 gamepad objects for gp1 and gp2
        GP gp1 = new GP(gamepad1);
        GP gp2 = new GP(gamepad2);

        waitForStart();
        if(opModeIsActive()){
            while (opModeIsActive()){
                // reads controllers and updates the pose
                gp1.readGP();
                gp2.readGP();
                driveTrain.updatePose(telemetry);

                // 2D Drive
                driveTrain.Drive2D();


                // updates telemetry
                telemetry.update();
            }

            // saves the pose to use in another program
            driveTrain.savePose();
        }
    }

}
