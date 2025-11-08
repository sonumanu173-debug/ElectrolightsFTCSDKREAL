package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.TestBenchColor;

public class ColorSensorTest extends OpMode {
    TestBenchColor bench = new TestBenchColor();
    TestBenchColor.detectedColor detectedColor;

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
