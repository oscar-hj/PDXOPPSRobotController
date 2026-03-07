package org.firstinspires.ftc.teamcode.teleop;

//import android.util.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.InterpLUT;
import org.firstinspires.ftc.teamcode.utils.TheLimeLight;

//@Disabled
//Config
@TeleOp(name="Motor Adjust with distance Alt", group="Test")
public class Example extends LinearOpMode{
    InterpLUT controlPoints;
    TheLimeLight limelight;
    //distance calculation
    public static double velocity = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        // Instantiate your motor class
        limelight = new TheLimeLight(hardwareMap,telemetry);
        limelight.init("limelight");
//        limelight.limelight.start();

        double distance = 0;
        double targetSpeed = 0;
        //Flywheel = hardwaredMap.get(DcMotorEx.class, "GBmotor");
        //distance calculation
        controlPoints = new InterpLUT();
        createControlPoints();
        //controlPoints.createLUT();

        telemetry.addData("", limelight.getDetectedAprilTagIDs());
        telemetry.addData("Status", "DCMotor initialized...");
        telemetry.addData("Status", "Limelight initialized. Waiting for Start.");
        limelight.updateTelemetry();
        telemetry.update();

        waitForStart();

        while(!isStopRequested()) {
            limelight.updateTelemetry();
            distance = limelight.getDistance();
            if (distance != 0){
                targetSpeed = controlPoints.get(distance);
//                flywheel.motor.setVelocity(targetSpeed*28.0/60.0);
            }

            telemetry.addData("distance", distance);
            telemetry.addData("targetSpeed", targetSpeed);
//            telemetry.addData("Encoder Position", flywheel.motor.getCurrentPosition());
//            telemetry.addData("Velocity (ticks/s)", flywheel.motor.getVelocity());
            telemetry.update();

        }

        limelight.stop();
    }

    public void createControlPoints() {
        // Initialize and add data points (distance, speed)
        /*
        controlPoints.add(28, 2900);
        controlPoints.add(41, 3000);
        controlPoints.add(44, 3100);
        controlPoints.add(54, 3200);
        controlPoints.add(60, 3300);
        controlPoints.add(68, 3400);
        controlPoints.add(78, 3500);
         */
        controlPoints.add(28, 1900);
        controlPoints.add(41, 2000);
        controlPoints.add(44, 2100);
        controlPoints.add(54, 2200);
        //controlPoints.add(57, 2250);//Mid
        controlPoints.add(60, 2300);
        //controlPoints.add(64, 2350);//Mid
        controlPoints.add(68, 2400);
        controlPoints.add(78, 2500);
    }
}