package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensorRev {
    NormalizedColorSensor colorSensor;


    public enum DetectedColor {
        RED,
        BLUE,
        YELLOW,
        GREEN,
        PURPLE,
        UNKNOWN
    }

    public void init(HardwareMap hwMap, String sensorName){
        colorSensor = hwMap.get(NormalizedColorSensor.class,sensorName);
        //colorSensor = hwMap.get(NormalizedColorSensor.class,"sensor_0");
        colorSensor.setGain(8);
    }

    public DetectedColor getDetectedColor(Telemetry telemetry) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors(); // return 4 values

        float normRed;
        float normGreen;
        float normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

        telemetry.addData("Checking R-G-B to detect color...",0);
        telemetry.addData("red", normRed);
        telemetry.addData("green", normGreen);
        telemetry.addData("blue", normBlue);

        /*
        red, green, blue;
        RED = >.35, <.3, <.3;
        YELLOW = >.5, >.9, <.6;
        BLUE = <.2, <.5, >.5;
        */

        if (normRed > 0.35 && normGreen < 0.3 && normBlue < 0.3) {
            return DetectedColor.RED;
        } else if (normRed > 0.5 && normGreen > 0.9 && normBlue < 0.6) {
            return DetectedColor.YELLOW;
        }
        else if (normRed < 0.2 && normGreen < 0.5 && normBlue > 0.5) {
            return DetectedColor.BLUE;
        }
        else if (normRed > 0.2 && normGreen < 0.45 && normBlue > 0.4) {
            return DetectedColor.PURPLE;
        }
        else if (normRed < 0.2 && normGreen > 0.35 && normBlue < 0.4) {
            return DetectedColor.GREEN;
        }
        else {
            return DetectedColor.UNKNOWN;
        }

        //
    }
        /*

         // TODO add if statements for specific colors added
        /#
        /#

        if (normRed  0.35 and normGreen  0.3 and normBlue  0.3) {
            return DetectedColor.RED;
        } else if (normRed  0.5 and normGreen  0.9 and normBlue  0.6) {
           Return DetectedColor.Yellow;
        }
        else if (normRed  0.2 and normGreen  0.5 normBlue  0.5) {
            return DetectedColor.BLUE;
         }
         else {

         return DetectedColor.UNKNOWN;
         }


    }
    */
}
