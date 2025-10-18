package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.MechanumDrive;

@TeleOp
public class MechanumFieldOrientatedOpMode extends OpMode {

    MechanumDrive drive = new MechanumDrive();
    double forward, strafe, rotate;

    @Override
    public void init() {
        //drive.init();
    }

    @Override
    public void loop() {
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        drive.driveFieldRelative(forward, strafe, rotate);
    }

}

