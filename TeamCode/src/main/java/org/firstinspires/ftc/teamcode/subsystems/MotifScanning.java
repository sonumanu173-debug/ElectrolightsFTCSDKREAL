package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import dev.nextftc.ftc.NextFTCOpMode;

public class MotifScanning extends NextFTCOpMode {

    private Limelight3A limelight;
    LLResult llResult;
    public int tagID;

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

    public int Motif() {
        limelight.pipelineSwitch(1);
        if (llResult.isValid() && llResult != null) {
            tagID = 23;
        }
        limelight.pipelineSwitch(2);
        if (llResult.isValid() && llResult != null) {
            tagID = 22;
        }
        limelight.pipelineSwitch(3);
        if (llResult.isValid() && llResult != null) {
            tagID = 21;
        }
        return tagID;
    }

    @Override
    public void onStop() {
        limelight.stop();
    }

}