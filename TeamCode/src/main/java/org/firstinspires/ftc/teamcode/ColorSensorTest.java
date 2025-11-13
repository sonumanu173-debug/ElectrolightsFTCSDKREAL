package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ColorSense1;

import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp
public class ColorSensorTest extends NextFTCOpMode {
    ColorSense1 bench = new ColorSense1();
    ColorSense1.detectedColor detectedColor;

    @Override
    public void onInit() {
        bench.init(hardwareMap);
    }

    @Override
    public void onUpdate() {
        detectedColor = bench.getDetectedColor(ActiveOpMode.telemetry());
        telemetry.addData("Color Detected", detectedColor);
        telemetry.update();
    }

}