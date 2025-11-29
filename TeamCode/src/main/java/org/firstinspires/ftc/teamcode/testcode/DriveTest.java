package org.firstinspires.ftc.teamcode.testcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Attachments;
import org.firstinspires.ftc.teamcode.utils.DriveTrain;
import org.firstinspires.ftc.teamcode.utils.GP;

@TeleOp(name = "Drive Test")
public class DriveTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        // Makes a DriveTrain object taking the hardwareMap and gamepad1, loads the pose, and
        // initializes motors and PedroPathing follower (for tracking pose).
        DriveTrain driveTrain = new DriveTrain(hardwareMap, gamepad1);
        driveTrain.initMotors("fl", "fr", "bl", "br");

        Attachments attachments = new Attachments(hardwareMap);
        attachments.initAttachments();

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


                if(gp1.A){
                    attachments.runOuttake(1);
                } else if (gp1.B){
                    attachments.runOuttake(0);
                }

                // updates telemetry
                driveTrain.printMotorTelemetry(telemetry);
                telemetry.update();
            }

            // saves the pose to use in another program
            driveTrain.savePose();
        }
    }

}
