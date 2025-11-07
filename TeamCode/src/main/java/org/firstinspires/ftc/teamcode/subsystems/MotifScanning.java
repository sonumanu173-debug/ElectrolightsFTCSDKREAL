package org.firstinspires.ftc.teamcode.subsystems;



import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.NextFTCOpMode;



public class MotifScanning implements Subsystem{
    private Limelight3A limelight;
    LLResult llResult;
    private HardwareMap hardwareMap;
    public MotifScanning(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    public int tagID;

    @Override
    public void initialize(){
        MotifScanning motifScanning = new MotifScanning(hardwareMap);
        motifScanning.initialize();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

    }

    public int motif(){
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