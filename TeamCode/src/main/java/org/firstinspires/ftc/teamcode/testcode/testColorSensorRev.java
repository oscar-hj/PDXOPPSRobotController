package org.firstinspires.ftc.teamcode.testcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.ColorSensorRev;

@Disabled
@TeleOp(name = "ColorRGB", group = "Sensor")
public class testColorSensorRev extends OpMode {
    ColorSensorRev bench = new ColorSensorRev();
    ColorSensorRev.DetectedColor detectedColor;

    @Override
    public void init() {
        bench.init(hardwareMap, "sensor_0");
    }

    @Override
    public void loop() {
        detectedColor = bench.getDetectedColor(telemetry);
        telemetry.addData("Detected Color: ", detectedColor);
    }
}


/*
@TeleOp(name = "ColorRGB", group = "Sensor")
public class ColorSensorTesting extends OpMode {
    SensorColor bench = new SensorColor();
    SensorColor.DetectedColor detectedColor;

    @Override
    public void init() {
        bench.init(hardwareMap);
    }

    @Override
    public void loop() {
        detectedColor = bench.getDetectedColor(telemetry);
        telemetry.addData("Detected Color: ", detectedColor);
    }
}
*/
