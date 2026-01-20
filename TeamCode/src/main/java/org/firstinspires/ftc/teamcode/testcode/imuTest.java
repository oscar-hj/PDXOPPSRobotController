package org.firstinspires.ftc.teamcode.testcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.myIMU;
@Disabled
@TeleOp(name = "IMU-Test", group = "Sensor")
public class imuTest extends OpMode {

    myIMU bench = new myIMU();

    @Override
    public void init(){
        bench.init(hardwareMap);
    }

    @Override
    public void loop(){
        telemetry.addData("hHeading", bench.getHeading(AngleUnit.DEGREES));
    }

}
