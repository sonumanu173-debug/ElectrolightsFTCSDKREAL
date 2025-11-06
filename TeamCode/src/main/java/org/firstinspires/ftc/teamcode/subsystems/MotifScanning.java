package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;

public class MotifScanning extends NextFTCOpMode {

    private Limelight3A limelight;
    private IMUEx imu;
    public int tagID;

    @Override
    public void onInit() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(7); //motif april tag pipeline
    }

    public void onStartButtonPressed() {
        limelight.start();
    }

    @Override
    public void onUpdate() {

    }

}
