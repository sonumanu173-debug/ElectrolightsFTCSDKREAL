package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.components.BulkReadComponent;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Main TeleOp")
public class TeleOp extends NextFTCOpMode {
    public TeleOp() {
        addComponents(
                new SubsystemComponent(Flywheel.INSTANCE, DriveTrain.INSTANCE, Intake.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE

        );
    }
    private Follower follower;

    private static final int APRILTAG_PIPELINE = 8;
    @Override
    public void onInit() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(25, -4, Math.toRadians(90)));
        follower.update();

    }

    @Override
    public void onUpdate() {
    }

    @Override
    public void onStartButtonPressed() {

    }
}