package org.firstinspires.ftc.teamcode.testcode;

import static java.lang.Math.abs;

import org.firstinspires.ftc.teamcode.utils.DriveTrain;
import org.firstinspires.ftc.teamcode.utils.GP;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.Shooter;
import org.firstinspires.ftc.teamcode.utils.Spindex;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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
        Spindex spindex = new Spindex(hardwareMap, telemetry, driveTrain);
        spindex.init("spinMotor", "transferServo", "magneticSwitch", "frontDistanceSensor", "backDistanceSensor");

        // initialize the shooter class
        Shooter shooter = new Shooter(hardwareMap, telemetry);
        shooter.init("shooterMotor", "hoodServo");

        // makes 2 gamepad objects for gp1 and gp2
        GP gp1 = new GP(gamepad1);
        GP gp2 = new GP(gamepad2);

        ElapsedTime spindexTimer = new ElapsedTime();

        waitForStart();
        if(opModeIsActive()){
            while (opModeIsActive()){
                //reads the gamepad inputs and assigns them to variables
                gp1.readGP();
                gp2.readGP();

                // Check driveMode of Drivetrain for auto position or operator driving
                if(driveTrain.driveMode == DriveTrain.DriveMode.OP_DRIVE){
                    // Runs driving function on gp1
                    driveTrain.Drive2D();

                    // Turn off shooter
                    shooter.primeShooter(0);

                    // Manually activate/deactivate intake
                    if(gp2.A){
                        intake.forwardIntake();
                    }else if(gp2.B){
                        intake.backwardsIntake();
                    }else {
                        intake.stopIntake();
                    }

                    // Spindex functions
                    spindex.intakeSpindex();
                    spindex.goToPos(spindex.currentPos);
                    telemetry.addData("Spindex State", spindex.spindexState);

                }else if(driveTrain.driveMode == DriveTrain.DriveMode.AUTO_POS){
                    // Runs driving function on gp1
                    driveTrain.Drive2D();

                    shooter.primeShooter(6000);

                    if(gamepad1.a){
                        spindex.shootSpindex();
                    }

                    // AUTO is on to auto drive the robot to the shooting place
                    // TODO: Add functionality to auto drive robot to shooting place
                }


                telemetry.update();
            }

            // saves the pose to use in another program
            driveTrain.savePose();
        }
    }

}
