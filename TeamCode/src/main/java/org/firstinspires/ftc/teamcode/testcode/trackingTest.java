package org.firstinspires.ftc.teamcode.testcode;

import static java.lang.Math.abs;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.lang.reflect.Array;

@TeleOp(name = "Tracking Test", group = "test")
public class trackingTest extends LinearOpMode {
    HuskyLens huskyLens;

    HuskyLens.Block[] blocks;

    @Override
    public void runOpMode(){
        initAll();
        waitForStart();
        
        
        if(opModeIsActive()){
            while (opModeIsActive()){
                blocks = huskyLens.blocks();

                telemetry.addData("Blocks", blocks);
//                for(int i = 0; i < blocks.length; i++){
//                    telemetry.addData("Data", blocks[i]);
//                }

            }
        }
    }

    public void initAll(){
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        huskyLens.initialize();
    }


}
