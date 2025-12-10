package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.DriveTrain;
import org.firstinspires.ftc.teamcode.utils.GP;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.Shooter;
import org.firstinspires.ftc.teamcode.utils.Spindex;


@TeleOp(name = "MainTeleOp_2GP")
public class MainTeleOp_2GP extends LinearOpMode {
    GP gp1 = new GP(gamepad1);
    GP gp2 = new GP(gamepad2);

    @Override
    public void runOpMode() {
        // Initialize drive train
        DriveTrain driveTrain = new DriveTrain(hardwareMap, gamepad1, telemetry);
        driveTrain.init("fl", "fr", "bl", "br");

        // Initialize intake
        Intake intake = new Intake(hardwareMap, telemetry);
        intake.init("intakeMotor");

        // Initialize spindex
        Spindex spindex = new Spindex(hardwareMap, telemetry);
        spindex.init("spinServo", "LTServo", "HTServo");

        // Initialize Shooter
        Shooter shooter = new Shooter(hardwareMap, telemetry);
        shooter.init("shooterMotor", "hoodServo");


        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                //reads the gamepad inputs and assigns them to variables
                gp1.readGP();
                gp2.readGP();

                // Check driveMode of Drivetrain for auto position or operator driving
                if(driveTrain.driveMode == DriveTrain.DriveMode.OP_DRIVE){
                    // Runs driving function on gp1
                    driveTrain.Drive2DFieldFreeLook();

                    // Runs spindex
                    spindex.intakeSpindex(true);

                    // Manually activate/deactivate intake
                    if(gp2.A){
                        intake.forwardIntake();
                    }else if(gp2.B){
                        intake.stopIntake();
                    }else if(gp2.Y){
                        intake.backwardsIntake();
                    }

                    if(gp1.A){
                        shooter.primeShooter(6000);
                    }

                    if(gp1.DPU){
                        spindex.activateTransfer(true);
                    }else if(gp1.DPD){
                        spindex.activateTransfer(false);
                    }else{
                        spindex.deactivateTransfer();
                    }

                }else{
                    // AUTO is on to auto drive the robot to the shooting place
                    // TODO: Add functionality to auto drive robot to shooting place
                }


                telemetry.update();
            }
        }
    }
}