package org.firstinspires.ftc.teamcode.teleop;

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
    boolean shootingActive = false;
    int spindexSpeed = 0;
    @Override
    public void runOpMode(){
        // Makes a DriveTrain object taking the hardwareMap and gamepad1, loads the pose, and
        // initializes motors and PedroPathing follower (for tracking pose).
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
        spindex.init("spinMotor", "transferServo", "magneticSwitch", "frontDistanceSensor", "backDistanceSensor", true);

        // makes 2 gamepad objects for gp1 and gp2
        GP gp1 = new GP(gamepad1);
        GP gp2 = new GP(gamepad2);

//        ElapsedTime spindexTimer = new ElapsedTime();

        waitForStart();
        if(opModeIsActive()){
            while (opModeIsActive()){
                //reads the gamepad inputs and assigns them to variables
                gp1.readGP();
                gp2.readGP();

                // Check driveMode of Drivetrain for auto position or operator driving
                if(driveTrain.driveMode == DriveTrain.DriveMode.OP_DRIVE){
                    shootingActive = false;
                    // Runs driving function on gp1
                    driveTrain.Drive2D();

                    // Turn off shooter
                    shooter.primeShooter(0);

                    // Manually activate/deactivate intake
                    if(gp1.RT > 0.05){
                        intake.forwardIntake();
                    }else if(gp1.LT > 0.05){
                        intake.backwardsIntake();
                    }else {
                        intake.stopIntake();
                    }

                    // Spindex functions
                    spindex.intakeSpindex();
                    spindex.goToPos(spindex.currentPos, true);
                    telemetry.addData("Spindex State", spindex.spindexState);

                }else if(driveTrain.driveMode == DriveTrain.DriveMode.AUTO_POS){
                    // Runs driving function on gp1
                    telemetry.addLine();
                    telemetry.addLine();
                    telemetry.addLine("SHOOTING");
                    telemetry.addLine();
                    telemetry.addLine();

                    driveTrain.Drive2D();
                    spindex.goToPos(spindex.currentPos, true);
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

                    if(shootingActive && abs(gp2.LSX) < 0.1){
                        shooter.primeShooter(spindexSpeed);
                        spindex.shootSpindex(spindexSpeed);
                    }else{
                        shooter.primeShooter(spindexSpeed);
                        spindex.manualSpindex(-gp2.LSX);
                    }
                }


                telemetry.update();
            }

            // saves the pose to use in another program
            driveTrain.savePose();
        }
    }

}
