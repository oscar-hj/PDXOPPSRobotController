package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.abs;

import org.firstinspires.ftc.teamcode.utils.DriveTrain;
import org.firstinspires.ftc.teamcode.utils.GP;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.Shooter;
import org.firstinspires.ftc.teamcode.utils.Spindex;
import org.firstinspires.ftc.teamcode.utils.TheLimeLight;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp(name = "Class Test")
public class ClassTest extends LinearOpMode {
    public static int shooterRPM = 0;
    boolean shootingActive, canShoot = false;
    int spindexSpeed = 0;
    @Override
    public void runOpMode(){
        // telemetry.setMsTransmissionInterval(20);
        // Makes a DriveTrain object, loads the pose, and PedroPathing follower (for tracking pose).
        DriveTrain driveTrain = new DriveTrain(hardwareMap, gamepad1, telemetry, 0.1, 0.7, 1);
        driveTrain.init("fl", "fr", "bl", "br");

        // initialize the intake class
        Intake intake = new Intake(hardwareMap, telemetry);
        intake.init("intakeMotor");

        // initialize the shooter class
        Shooter shooter = new Shooter(hardwareMap, telemetry);
        shooter.init("shooterMotor", "hoodServo");

        // initialize the spindex class
        Spindex spindex = new Spindex(hardwareMap, telemetry, shooter, driveTrain);
        spindex.init("spinMotor", "kickServo", "magneticSwitch", "distanceSensor", "transferMotor", true);

        // Initialize limelight
        TheLimeLight limeLight = new TheLimeLight(hardwareMap, telemetry);
        limeLight.init("limelight");

        // makes 2 gamepad objects for gp1 and gp2
        GP gp1 = new GP(gamepad1);
        GP gp2 = new GP(gamepad2);


        waitForStart();
        if(opModeIsActive()){
            while (opModeIsActive()){
                //reads the gamepad inputs and assigns them to variables
                gp1.update();
                gp2.update();

                // Updates pose for the follower
                driveTrain.updatePose();

                // Check driveMode of Drivetrain for auto position or operator driving
                if(driveTrain.driveMode == DriveTrain.DriveMode.OP_DRIVE){
                    // Sets shooting false for next shooting run and deactivates shooter
                    shootingActive = false;
                    spindexSpeed = 0;
                    shooter.primeShooter(0);

                    // Runs driving function on gp1
                    driveTrain.Drive2D(gp1);

                    // Manually activate/deactivate intake
                    if(gp1.RT > 0.05){
                        intake.forwardIntake();
                    }else if(gp1.LT > 0.05){
                        intake.backwardsIntake();
                    }else {
                        intake.stopIntake();
                    }

                    // Runs spindex functions
                    spindex.updateKicker();
                    spindex.intakeSpindex();
                    spindex.goToPos(spindex.targetPos, true);
                    telemetry.addData("Spindex State", spindex.spindexState);
                    telemetry.addData("Current Position", spindex.targetPos);

                    // Overwrites spindex to go into shooting mode
                    if(gp1.PS){
                        spindex.targetPos = Spindex.Offset.SHOOT1;
                    }
                // Activates when spindex detects when 3 balls are in the spindex
                }else if(driveTrain.driveMode == DriveTrain.DriveMode.AUTO_POS){
                    // Runs driving function with automatic alignment on gp1
                    driveTrain.Drive2DWithAlign(limeLight.getTargetXOffset() - 0.5, gp1);

                    // Can run intake backwards in case the robot intakes 4 balls
                    if(gp1.LT > 0.05){
                        intake.backwardsIntake();
                    }else{
                        intake.stopIntake();
                    }

                    //Update limelight and RPM
                    limeLight.updateTelemetry();
                    shooterRPM = shooter.LUTToRPM(limeLight.getDistance());

                    // Tells drivers we are in shooting mode
                    telemetry.addLine();
                    telemetry.addLine();
                    telemetry.addLine("SHOOTING");
                    telemetry.addLine();
                    telemetry.addLine();

                    // Run spindex
                    spindex.goToPos(spindex.targetPos, true);
                    spindex.updateKicker();

                    // D-Pad up is far shooting, D-Pad down in close shooting, PS to turn off
                    if(gamepad1.dpad_up){
                        spindexSpeed = shooterRPM;
                        canShoot = true;
                        shootingActive = true;
                    }
                    if(gp1.DPD){
                        spindexSpeed = 0;
                        shootingActive = false;
                    }

                    // Runs shooter and spindex with ability to override spindex PID
                    if(shootingActive && abs(gp2.LSX) < 0.1){
                        shooter.primeShooter(spindexSpeed);
                        spindex.shootSpindex(spindexSpeed, canShoot);
                        canShoot = false;
                    }else{
                        shooter.primeShooter(spindexSpeed);
                        spindex.manualSpindex(-gp2.LSX);
                    }
                }

                // Update telemetry
                telemetry.update();
            }

            limeLight.stop();
            // saves the pose to use in another program
            driveTrain.savePose(driveTrain.follower);
        }
    }

}
