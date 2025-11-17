package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import dev.nextftc.ftc.NextFTCOpMode;

//Use to create the distance equation
@TeleOp
public class AprilTagDistanceEquation extends NextFTCOpMode {

    TestBench bench = new TestBench();

    private Limelight3A limelight;

    public double distance = 0;

    @Override
    public void onInit() {
        bench.init(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1); //use 0 for distance calculations
    }

    @Override
    public void onStartButtonPressed() {
        limelight.start();
    }

    @Override
    public void onUpdate () {
        YawPitchRollAngles orientation = bench.getOrientation();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botpose = llResult.getBotpose_MT2();
            telemetry.addData("Calculated Distance", distance);
            telemetry.addData("Target X", llResult.getTx());
            telemetry.addData("Target Area", llResult.getTa());
            telemetry.addData("Botpose", botpose.toString());
        }

    }

}
