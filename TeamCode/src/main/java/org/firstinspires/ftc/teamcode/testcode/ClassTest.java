package org.firstinspires.ftc.teamcode.testcode;

import org.firstinspires.ftc.teamcode.utils.DriveTrain;
import org.firstinspires.ftc.teamcode.utils.GP;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.Shooter;
import org.firstinspires.ftc.teamcode.utils.Spindex;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Class Test")
public class ClassTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        // Makes a DriveTrain object taking the hardwareMap and gamepad1, loads the pose, and
        // initializes motors and PedroPathing follower (for tracking pose).
        DriveTrain driveTrain = new DriveTrain(hardwareMap, gamepad1, telemetry);
        driveTrain.init("fl", "fr", "bl", "br");

        // initialize the intake class
        Intake intake = new Intake(hardwareMap, telemetry);
        intake.init("intakeMotor");

        // initialize the spindex class
        Spindex spindex = new Spindex(hardwareMap, telemetry);
        spindex.init("spinServo", "LTServo", "HTServo");

        // initialize the shooter class
        Shooter shooter = new Shooter(hardwareMap, telemetry);
        shooter.init("shooterMotor", "hoodServo");

        // makes 2 gamepad objects for gp1 and gp2
        GP gp1 = new GP(gamepad1);
        GP gp2 = new GP(gamepad2);

        waitForStart();
        if(opModeIsActive()){
            while (opModeIsActive()){
                // reads controllers and updates the pose
                gp1.readGP();
                gp2.readGP();
                driveTrain.updatePose();

                // 2D Drive, field oriented
                driveTrain.Drive2DFieldFreeLook();

                // When PS is pressed, prime shooter and move to the shooting point
                if(gp1.PS){
                    shooter.primeShooter(6000);
                    //TODO: Add function for
                }

//                if(gp1.PS){
//                    driveTrain.resetPose();
//                    driveTrain.follower.setPose(driveTrain.loadPose());
//                }

                // updates telemetry
                driveTrain.printMotorTelemetry(telemetry);
                telemetry.update();
            }

            // saves the pose to use in another program
            driveTrain.savePose();
        }
    }

}
