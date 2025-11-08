package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSense1 {
    NormalizedColorSensor colorSensor;
    boolean isGreen;
    boolean isPurple;

    public enum detectedColor {
        PURPLE,
        GREEN,
        ERROR
    }

    public void init(HardwareMap hwMap) {
        colorSensor = hwMap.get(NormalizedColorSensor.class, "colorSensor1");
        colorSensor.setGain(8);
    }

    public detectedColor getDetectedColor(Telemetry telemetry) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors(); //return 4 values

        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

        telemetry.addData("red", normRed);
        telemetry.addData("green", normGreen);
        telemetry.addData("blue", normBlue);

        isGreen = false;
        isPurple = false;

        //Green
        if (normRed > 0.02 && normRed < 0.04 && normGreen > 0.09 && normGreen < 0.14 && normBlue > 0.07 && normBlue < 0.11) {
            isGreen = true;
        } else if (normRed > 0.04 && normRed < 0.07 && normGreen > 0.04 && normGreen < 0.07 && normBlue > 0.09 && normBlue < 0.13) { //Purple
            isPurple = true;
        }

        telemetry.addData("isGreen", isGreen);
        telemetry.addData("isPurple", isPurple);


        if (isGreen) {
            return detectedColor.GREEN;
        } else if (isPurple) {
            return detectedColor.PURPLE;
        } else {
            return detectedColor.ERROR;
        }

    }

}
