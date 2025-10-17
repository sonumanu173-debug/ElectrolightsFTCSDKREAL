package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class LauncherTesting extends OpMode {

    private DcMotor motor;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "launchingmotor"); // launching motor is name in control hub
    }

    @Override
    public void start() {
        motor.setPower(0.75);
    }

    public void loop() {

    }

}
