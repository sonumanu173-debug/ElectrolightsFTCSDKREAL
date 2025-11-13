package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ColorSense1;

@TeleOp
public class ColorSensorTest extends OpMode {
    ColorSense1 bench = new ColorSense1();
    ColorSense1.detectedColor detectedColor;

    @Override
    public void init() {
        bench.init(hardwareMap);
    }

    @Override
    public void loop() {
        detectedColor = bench.getDetectedColor(telemetry);
        telemetry.addData("Color Detected", detectedColor);
        telemetry.update();
    }

}