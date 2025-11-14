package org.firstinspires.ftc.teamcode.subsystems;



import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;



public class MotifScanning implements Subsystem{

    public static final MotifScanning INSTANCE = new MotifScanning();
    private MotifScanning(){
    }
    public static int tagID;
    private static Limelight3A limelight;
    static LLResult llResult;
    @Override
    public void initialize(){

        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.start();

    }

    public static int findMotif(){
        tagID = -1;
        llResult = limelight.getLatestResult();
        limelight.pipelineSwitch(1);
        try {
            Thread.sleep(333);
            } catch (InterruptedException e) {
            e.printStackTrace();
        }
        if (llResult != null && llResult.isValid()) {
            tagID = 21;
        }
        llResult = limelight.getLatestResult();
        limelight.pipelineSwitch(2);
        try {
            Thread.sleep(333);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        if (llResult != null && llResult.isValid()) {
            tagID = 23;
        }
        llResult = limelight.getLatestResult();
        limelight.pipelineSwitch(3);
        try {
            Thread.sleep(333);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        if (llResult != null && llResult.isValid()) {
            tagID = 22;
        }
        return tagID;

    }

    public void periodic(){
        llResult = limelight.getLatestResult();
    }

}