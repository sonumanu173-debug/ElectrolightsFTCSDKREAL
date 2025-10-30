package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import dev.nextftc.ftc.NextFTCOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous Test (NextFTC)", group = "Autonomous")
public class Autotest extends NextFTCOpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Paths paths;


    public void onInit() {
        telemetry.addLine("Initializing Follower...");
        telemetry.update();
        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        follower.setPose(new Pose(56.000, 8.000, Math.toRadians(90)));
        pathState = 0;
        telemetry.addLine("Follower + IMU + Odo Pods initialized successfully!");
        telemetry.addLine("Initialization complete!");
        telemetry.update();
    }


    public void onStart() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        follower.followPath(paths.Path1);
        telemetry.addLine("Started Path 1");
        telemetry.update();
    }


    public void onRun() {
        follower.update();
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    follower.followPath(paths.Path2);
                    telemetry.addLine("Started Path 2");
                    pathState++;
                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    telemetry.addLine("Autonomous Complete LOLLLL");
                    pathState++;
                }
                break;
        }
        telemetry.addData("State", pathState);
        telemetry.addData("Pose", follower.getPose());
        telemetry.addData("Path Timer", pathTimer.getElapsedTime());
        telemetry.update();
    }

    @Override
    public void onStop() {
        follower.breakFollowing();
        telemetry.addLine("Autonomous Stopped.");
        telemetry.update();
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56.000, 8.000),
                            new Pose(57.078, 32.451)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(57.078, 32.451),
                            new Pose(58.527, 41.433),
                            new Pose(14.487, 35.348)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }
}
