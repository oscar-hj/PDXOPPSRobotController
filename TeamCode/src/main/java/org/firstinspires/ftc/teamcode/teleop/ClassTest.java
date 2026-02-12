package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.abs;

import org.firstinspires.ftc.teamcode.utils.DriveTrain;
import org.firstinspires.ftc.teamcode.utils.GP;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.Shooter;
import org.firstinspires.ftc.teamcode.utils.Spindex;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Class Test")
public class ClassTest extends LinearOpMode {
    boolean shootingActive = false;
    int spindexSpeed = 0;
    @Override
    public void runOpMode(){
        // Makes a DriveTrain object, loads the pose, and PedroPathing follower (for tracking pose).
        DriveTrain driveTrain = new DriveTrain(hardwareMap, gamepad1, telemetry);
        driveTrain.init("fl", "fr", "bl", "br");

        // initialize the intake class
        Intake intake = new Intake(hardwareMap, telemetry);
        intake.init("intakeMotor");

        // initialize the shooter class
        Shooter shooter = new Shooter(hardwareMap, telemetry);
        shooter.init("shooterMotor", "hoodServo");

        // initialize the spindex class
        Spindex spindex = new Spindex(hardwareMap, telemetry, shooter, driveTrain);
        spindex.init("spinMotor", "transferServo",
                "magneticSwitch", "frontDistanceSensor",
                "backDistanceSensor", true);

        // makes 2 gamepad objects for gp1 and gp2
        GP gp1 = new GP(gamepad1);
        GP gp2 = new GP(gamepad2);


        waitForStart();
        if(opModeIsActive()){
            while (opModeIsActive()){
                //reads the gamepad inputs and assigns them to variables
                gp1.readGP();
                gp2.readGP();

                // Updates pose for the follower
                driveTrain.updatePose();

                // Check driveMode of Drivetrain for auto position or operator driving
                if(driveTrain.driveMode == DriveTrain.DriveMode.OP_DRIVE){
                    // Sets shooting false for next shooting run and deactivates shooter
                    shootingActive = false;
                    shooter.primeShooter(0);

                    // Runs driving function on gp1
                    driveTrain.Drive2D();

                    // Manually activate/deactivate intake
                    if(gp1.RT > 0.05){
                        intake.forwardIntake();
                    }else if(gp1.LT > 0.05){
                        intake.backwardsIntake();
                    }else {
                        intake.stopIntake();
                    }

                    // Runs spindex functions
                    spindex.intakeSpindex();
                    spindex.goToPos(spindex.targetPos, true);
                    telemetry.addData("Spindex State", spindex.spindexState);


                // Activates when spindex detects when 3 balls are in the spindex
                // TODO: Make override
                }else if(driveTrain.driveMode == DriveTrain.DriveMode.AUTO_POS){
                    // Runs driving function on gp1
                    driveTrain.Drive2D();

                    // Tells drivers we are in shooting mode
                    telemetry.addLine();
                    telemetry.addLine();
                    telemetry.addLine("SHOOTING");
                    telemetry.addLine();
                    telemetry.addLine();

                    // Run spindex
                    spindex.goToPos(spindex.targetPos, true);

                    // D-Pad up is far shooting, D-Pad down in close shooting, PS to turn off
                    if(gamepad1.dpad_up){
                        spindexSpeed = 4800;
                        shootingActive = true;
                    }
                    if(gamepad1.dpad_down){
                        spindexSpeed = 3400;
                        shootingActive = true;
                    }
                    if(gamepad1.ps){
                        spindexSpeed = 0;
                        shootingActive = false;
                    }

                    // Runs shooter and spindex with ability to override spindex PID
                    if(shootingActive && abs(gp2.LSX) < 0.1){
                        shooter.primeShooter(spindexSpeed);
                        spindex.shootSpindex(spindexSpeed);
                    }else{
                        shooter.primeShooter(spindexSpeed);
                        spindex.manualSpindex(-gp2.LSX);
                    }
                }

                // Update telemetry
                telemetry.update();
            }

            // saves the pose to use in another program
            driveTrain.savePose();
        }
    }

}
