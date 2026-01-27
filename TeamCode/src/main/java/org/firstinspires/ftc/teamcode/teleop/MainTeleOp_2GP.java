package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.DriveTrain;
import org.firstinspires.ftc.teamcode.utils.GP;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.Shooter;
import org.firstinspires.ftc.teamcode.utils.Spindex;


@TeleOp(name = "MainTeleOp_2GP")
public class MainTeleOp_2GP extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize controllers
        GP gp1 = new GP(gamepad1);
        GP gp2 = new GP(gamepad2);

        // Initialize drive train
        DriveTrain driveTrain = new DriveTrain(hardwareMap, gamepad1, telemetry);
        driveTrain.init("fl", "fr", "bl", "br");

        // Initialize intake
        Intake intake = new Intake(hardwareMap, telemetry);
        intake.init("intakeMotor");

        // Initialize spindex
        Spindex spindex = new Spindex(hardwareMap, telemetry, driveTrain);
        spindex.init("spinMotor", "transferServo", "magneticSwitch", "frontDistanceSensor", "backDistanceSensor");
        ElapsedTime spindexTimer = new ElapsedTime();

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
                if(true){
                    // Runs driving function on gp1
                    driveTrain.Drive2D();

                    // Runs spindex
                    // spindex.intakeSpindex(true);

                    // Manually activate/deactivate intake
                    if(gp2.A){
                        intake.forwardIntake();
                    }else if(gp2.B){
                        intake.backwardsIntake();
                    }else {
                        intake.stopIntake();
                    }

                    // Shooter functions
                    if(gp1.A){
                        shooter.primeShooter(10000);
                    } else if(gp1.B){
                        shooter.primeShooter(0);
                    }

                    if(gp1.LT > 0.1){
                        shooter.manualHoodAngle(gp1.LT);
                    } else if(gp1.RT > 0.1){
                        shooter.manualHoodAngle(gp1.RT);
                    }


                    if(spindexTimer.time() > 0.2 && gamepad2.dpad_right){
                        spindex.nextPos();
                        spindexTimer.reset();
                    }
                    spindex.intakeSpindex();
                    spindex.goToPos(spindex.currentPos);
                    telemetry.addData("Target State", spindex.currentPos);

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