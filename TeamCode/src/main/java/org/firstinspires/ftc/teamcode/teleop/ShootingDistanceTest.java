package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.DriveTrain;
import org.firstinspires.ftc.teamcode.utils.GP;
import org.firstinspires.ftc.teamcode.utils.Shooter;
import org.firstinspires.ftc.teamcode.utils.Spindex;


@Configurable
@TeleOp(name = "ShootingDistanceTest")
public class ShootingDistanceTest extends LinearOpMode {
    public static int highSpeed = 5200;
    public static int highMidSpeed = 4600;
    public static int lowMidSpeed = 4000;
    public static int lowSpeed = 3400;

    @Override
    public void runOpMode() {
        // Initialize controllers
        GP gp1 = new GP(gamepad1);
        GP gp2 = new GP(gamepad2);



        // Initialize drive train
        DriveTrain driveTrain = new DriveTrain(hardwareMap, gamepad1, telemetry);
        driveTrain.init("fl", "fr", "bl", "br");

        Spindex spindex = new Spindex(hardwareMap, telemetry, driveTrain);
        spindex.init("spinMotor", "kickServo", "magneticSwitch", "distanceSensor", "transferMotor", true);

        telemetry.setMsTransmissionInterval(10);

        // Initialize Shooter
        Shooter shooter = new Shooter(hardwareMap, telemetry);
        shooter.init("shooterMotor", "hoodServo");

        int shooterRPM = 0;

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                //reads the gamepad inputs and assigns them to variables
                gp1.readGP();
                gp2.readGP();

                // Runs driving function on gp1
                driveTrain.Drive2D();

                // Shooter functions
                if(gp1.DPU){
                    shooterRPM = highSpeed;
                } else if(gp1.DPL){
                    shooterRPM = highMidSpeed;
                } else if(gp1.DPR){
                    shooterRPM = lowMidSpeed;
                }else if(gp1.DPD){
                    shooterRPM = lowSpeed;
                }else if(gp1.B){
                    shooterRPM = 0;
                }

                shooter.primeShooter(shooterRPM);


                telemetry.update();
            }
        }
    }
}