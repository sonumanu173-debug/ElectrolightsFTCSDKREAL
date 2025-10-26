package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import dev.nextftc.ftc.NextFTCOpMode;

public class Launching extends NextFTCOpMode {

    private DcMotorEx launcher;
    

    @Override
    public void onInit() {
        launcher = hardwareMap.get(DcMotorEx.class, "launchingmotor");
    }

    @Override
    public void onWaitForStart() {
        super.onWaitForStart();
    }

    @Override
    public void onStartButtonPressed() {
        launcher.setPower();
    }

    @Override
    public void onUpdate() {
        super.onUpdate();
    }

    @Override
    public void onStop() {
        super.onStop();
    }

}
