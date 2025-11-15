package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.core.subsystems.Subsystem;

public class ColorSense2 implements Subsystem {
    static NormalizedColorSensor colorSensor;

    public static final ColorSense2 INSTANCE = new ColorSense2();
    public ColorSense2() { }
    static boolean isGreen;
    static boolean isPurple;

    public enum detectedColor {
        PURPLE,
        GREEN,
        ERROR
    }

    public void init(HardwareMap hwMap) {
        colorSensor = hwMap.get(NormalizedColorSensor.class, "colorSensor2");
        colorSensor.setGain(8);
    }

    public static detectedColor getDetectedColor(Telemetry telemetry) {
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
        if (normRed > 0.01 && normRed < 0.05 && normGreen > 0.08 && normGreen < 0.15 && normBlue > 0.06 && normBlue < 0.12) {
            isGreen = true;
        } else if (normRed > 0.03 && normRed < 0.08 && normGreen > 0.03 && normGreen < 0.08 && normBlue > 0.08 && normBlue < 0.14) { //Purple
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
