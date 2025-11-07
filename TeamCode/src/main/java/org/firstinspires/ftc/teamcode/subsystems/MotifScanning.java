package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import dev.nextftc.ftc.NextFTCOpMode;

public class MotifScanning extends NextFTCOpMode {

    private Limelight3A limelight;
    LLResult llResult;

    @Override
    public void onInit() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public void onStartButtonPressed() {
        limelight.start();
    }

    @Override
    public void onUpdate() {
        llResult = limelight.getLatestResult();
    }

    public Motif() {
        
    }

    @Override
    public void onDisable() {
        limelight.stop();
    }

}