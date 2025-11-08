package org.firstinspires.ftc.teamcode.subsystems;



import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;



public class MotifScanning implements Subsystem{

    public static final MotifScanning INSTANCE = new MotifScanning();
    private MotifScanning(){
    }
    public int tagID;
    private Limelight3A limelight;
    LLResult llResult;
    @Override
    public void initialize(){

        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.start();

    }

    public int findMotif(){
        tagID = -1;
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

    public void periodic(){
        llResult = limelight.getLatestResult();
    }

}