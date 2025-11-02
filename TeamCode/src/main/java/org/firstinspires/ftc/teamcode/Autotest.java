package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower; // tells robot about tge path it follows
import com.pedropathing.geometry.BezierLine; //2 points
import com.pedropathing.geometry.BezierCurve; //3 points
import com.pedropathing.geometry.Pose; // location
import com.pedropathing.paths.PathChain; // Continious paths composed into one smooth motion
import com.pedropathing.util.Timer; // timing for everything in pedro
import com.qualcomm.robotcore.eventloop.opmode.Autonomous; // Imports auto
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Autonomous Test", group = "Autonomous") // Defines autonomous to the ftc sdk

public class Autotest extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private Paths paths;

    @Override
    public void init() {
        telemetry.addLine("Initializing Follower...");
        telemetry.update();

        follower = Constants.createFollower(hardwareMap);

        telemetry.addLine("Follower and pinpoint IMU+odo pods have been initialied successfully!");
        telemetry.update();

        paths = new Paths(follower);

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        follower.setPose(new Pose(56.000, 8.000, Math.toRadians(90)));

        pathState = 0;
        telemetry.addLine("Fully finished initialization");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        follower.followPath(paths.Path1);
        telemetry.addLine("Path 1 has been succesfully built");
    }

    @Override
    public void loop() {
        follower.update();

        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    follower.followPath(paths.Path2);
                    telemetry.addLine("Path 2 has been succesfully built");
                    pathState++;
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    telemetry.addLine("✅ Auto complete!");
                    pathState++;
                }
                break;
        }

        telemetry.addData("State", pathState);
        telemetry.addData("Pose", follower.getPose());
        telemetry.addData("Path Timer", pathTimer.getElapsedTime());
        telemetry.update();
    }


    public static class Paths {
        public PathChain Path1;
        //path = create new path for robot to follow
        //stage = think of three stage rocket
        //state = variable containing stage
        //case = stage
        //! = not
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
