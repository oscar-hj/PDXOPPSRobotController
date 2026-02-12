package org.firstinspires.ftc.teamcode.testcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.dfrobot.HuskyLens.Block;

@Disabled
@TeleOp(name = "Servo Control with HuskyLens (Corrected Directions)", group = "PDX OPPS")
public class HalloweenCandy2 extends LinearOpMode {

    private HuskyLens hlo;
    private Servo servo_1, servo_2, servo_3, servo_4;

    // Individual start positions
    private double startPos1 = 0.23; // REV
    private double startPos2 = 0.46; // GoBilda
    private double startPos3 = 0.23; // GoBilda
    private double startPos4 = 0.35; // GoBilda

    private double moveAmount = 50.0 / 180.0; // 50° ≈ 0.28
    private long moveDelay = 20; // ms per step
    private double step = 0.01;  // smoothness

    @Override
    public void runOpMode() throws InterruptedException {

        // Hardware mapping
        hlo = hardwareMap.get(HuskyLens.class, "hlo");
        servo_1 = hardwareMap.get(Servo.class, "servo_1");
        servo_2 = hardwareMap.get(Servo.class, "servo_2");
        servo_3 = hardwareMap.get(Servo.class, "servo_3");
        servo_4 = hardwareMap.get(Servo.class, "servo_4");

        // Set initial positions
        servo_1.setPosition(startPos1);
        servo_2.setPosition(startPos2);
        servo_3.setPosition(startPos3);
        servo_4.setPosition(startPos4);

        // Initialize HuskyLens
        hlo.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.addLine("Ready - HuskyLens Tag Recognition Mode");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (hlo.knock()) {
                Block[] blocks = hlo.blocks();

                if (blocks.length > 0) {
                    int id = blocks[0].id;
                    telemetry.addData("Detected Tag ID", id);
                    telemetry.update();

                    if (id == 1 || id == 21) {
                        moveServoSlow(servo_1, "servo_1", startPos1, false); // REV
                    } else if (id == 2 || id == 22) {
                        moveServoSlow(servo_2, "servo_2", startPos2, true); // reversed
                    } else if (id == 3 || id == 23) {
                        moveServoSlow(servo_3, "servo_3", startPos3, true); // reversed
                    } else if (id == 4 || id == 20) {
                        moveServoSlow(servo_4, "servo_4", startPos4, false); // normal
                    }

                    sleep(3000); // cooldown

                } else {
                    telemetry.addLine("No Tag in frame");
                    telemetry.update();
                }

            } else {
                telemetry.addLine("No Tag in frame");
                telemetry.update();
            }

            sleep(50);
        }
    }

    private void moveServoSlow(Servo servo, String name, double startPos, boolean reversed) throws InterruptedException {
        double upPos = startPos; // starting UP
        double downPos;

        if (reversed) {
            downPos = Math.min(1.0, startPos + moveAmount); // reversed servo goes down by increasing position
        } else {
            downPos = Math.max(0.0, startPos - moveAmount); // normal servo goes down by decreasing position
        }

        telemetry.addData("Now Lowering Candy", "on " + name);
        telemetry.update();

        // Move DOWN slowly
        if (reversed) {
            for (double pos = upPos; pos <= downPos; pos += step) {
                servo.setPosition(pos);
                sleep(moveDelay);
            }
        } else {
            for (double pos = upPos; pos >= downPos; pos -= step) {
                servo.setPosition(pos);
                sleep(moveDelay);
            }
        }

        sleep(1000); // hold 1s

        // Move BACK UP slowly
        if (reversed) {
            for (double pos = downPos; pos >= upPos; pos -= step) {
                servo.setPosition(pos);
                sleep(moveDelay);
            }
        } else {
            for (double pos = downPos; pos <= upPos; pos += step) {
                servo.setPosition(pos);
                sleep(moveDelay);
            }
        }

        telemetry.addData(name + " Movement", "Up: %.2f Down: %.2f", upPos, downPos);
        telemetry.update();
    }
}