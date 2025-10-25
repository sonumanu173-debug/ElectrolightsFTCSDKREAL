package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class LauncherTesting extends OpMode {

    private DcMotorEx motor;
    private static final double TICKS_PER_REV = 28;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "launchingmotor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start() {
        motor.setPower(1);
    }

    @Override
    public void loop() {
        double ticksPerSecond = motor.getVelocity();

        double rpm = (ticksPerSecond / TICKS_PER_REV) * 60.0;

        telemetry.addData("Motor RPM", rpm);
        telemetry.addData("graph:Motor RPM", rpm); // Panels graph
        telemetry.update();

    }
}