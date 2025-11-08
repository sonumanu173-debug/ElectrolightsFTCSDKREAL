package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.nio.charset.CharacterCodingException;

public class TestBenchColor {
    NormalizedColorSensor colorSensor;

    public enum detectedColor {
        PURPLE,
        GREEN,
        UNKNOWN
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

        // TODO ADD IF STATEMENTS FOR SPECIFIC COLORS ADDED
        /*

        if (normRed > 0.35 && normGreen < 0.3 && normBlue < 0.3) {
            return detectedColor.PURPLE;
        } else if (normRed > 0.5 && normGreen < 0.5 && normBlue < 0.5) {
            return detectedColor.GREEN;
        } else {
            return detectedColor.UNKNOWN;
        }


         */
        return detectedColor.UNKNOWN;
    }

}
