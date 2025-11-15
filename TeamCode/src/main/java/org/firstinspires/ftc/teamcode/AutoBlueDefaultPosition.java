

package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.subsystems.AutoSubsystem.servoPos;
import static org.firstinspires.ftc.teamcode.subsystems.AutoSubsystem.spin;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AutoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MotifScanning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;



@Autonomous(name = "Autonomous Test (NextFTC)", group = "Autonomous")
@Configurable
public class AutoBlueDefaultPosition extends NextFTCOpMode {
    public AutoBlueDefaultPosition(){
        addComponents(
                new SubsystemComponent(Flywheel.INSTANCE,  Intake.INSTANCE, MotifScanning.INSTANCE, AutoSubsystem.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE

        );
    }
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Paths paths;
    public Pose start = new Pose(28,120, Math.toRadians(110));

    public Pose PreloadLaunch = new Pose(54.871, 97.258, Math.toRadians(130));

    public Pose controlPoint1 = new Pose(50.263, 76.015);

    public Pose intake1 = new Pose(10.44396, 85.767033);

    public Pose ControlPose2 = new Pose(2.8484, 88.932);
    public Pose ControlPose3 = new Pose(3.48132, 113.934);
    public Pose Launch1 = new Pose(56.967033, 94);// THIS will be used for launching all 3 times lol

    public Pose intake2ControlPose = new Pose(84.5011, 58.5495);

    public Pose intake2 = new Pose(3.811, 59.499);

    public Pose Intake3ControlPoint = new Pose(85.1341, 25.635165);

    public Pose Intake3 = new Pose(-1.81099, 35.4462);

    public Pose ClassifierRampControl = new Pose(72.49061662198392,75.4745308310992);

    public Pose ClassifierRamp = new Pose(4.3,74);









    private MotorEx intakeMotor;

    private MotorEx spindexerMotor;

    public void onInit() {
        telemetry.addLine("Initializing Follower...");
        telemetry.update();
        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);
        intakeMotor = new MotorEx("intake");
        spindexerMotor = new MotorEx("spindexer");
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        follower.setStartingPose(start);

        pathState = 0;
        telemetry.addLine("Follower + IMU + Odo Pods initialized successfully!");
        telemetry.addLine("Initialization complete!");
        telemetry.update();
    }


    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        int tag=MotifScanning.INSTANCE.findMotif();
        follower.followPath(paths.Path1);
        Flywheel.shooter(1500);
        telemetry.addLine("The shooter has started btw");
        telemetry.addLine("Started Path 1");
        telemetry.update();
    }


    public void onUpdate() {
        follower.update();
        switch (pathState) {

            case 0:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    // WIhs follower.followPath(paths.Path2);

                    telemetry.addLine("Path 1 has been completed btw, that means it's going to launch in a couple of seconds");
                        // Im going to add spindexer logic here once its been done and spin the flywheel, we r so cooked
                    follower.turnToDegrees(75);

                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    Flywheel.shooter(0);
                    telemetry.addLine("The shooter has started btw");
                    intakeMotor.setPower(-1);
                    telemetry.addLine("The intake has started btw");
                    spin(2);
                    telemetry.addLine("The spindexer motor has started btw");
                    telemetry.update();
                    follower.followPath(paths.Path2);
                    telemetry.addLine("Started path 2 to intake the balls");
                    pathState++;
                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                    telemetry.addLine("Moving onto path 8 momentarily");
                    telemetry.update();
                    intakeMotor.setPower(0);

                    Flywheel.shooter(0);
                    follower.followPath(paths.Path8);
                    pathState++;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    try {
                        Thread.sleep(2000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    Flywheel.shooter(1500);
                    servoPos.setPosition(0.1);
                    try {
                        Thread.sleep(1500);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    servoPos.setPosition(0.0);
                    telemetry.addLine("UHH YES");
                    follower.followPath(paths.Path3);

                    pathState++;


                }
            case 3:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    intakeMotor.setPower(0);
                    try {
                        Thread.sleep(300);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    Flywheel.shooter(1500);
                    // Flywheel and spindexer logic here momentarily THIS IS WHERE WE HAVE TO SHOOT AFTER INTAKING BALLS
                    telemetry.addLine("UHH YES");
                    intakeMotor.setPower(-1);
                    follower.followPath(paths.Path4);
                    pathState++;


                }
            case 4:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    try {
                        Thread.sleep(300);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    intakeMotor.setPower(0);
                    Flywheel.shooter(1500);
                    // Flywheel and spindexer logic here momentarily
                    telemetry.addLine("UHH YES");
                    intakeMotor.setPower(0);
                    follower.followPath(paths.Path5);
                    pathState++;


                }
            case 5:
                if(!follower.isBusy()){
                    pathTimer.resetTimer();
                    intakeMotor.setPower(-1);
                    follower.followPath(paths.Path6);
                    pathState++;
                }
            case 6:
                if(!follower.isBusy()) {
                    pathTimer.resetTimer();
                    intakeMotor.setPower(0);
                    Flywheel.shooter(1500);
                    // Spindexer and sorting logic here
                    follower.followPath(paths.Path7);
                }
            case 7:
                if(!follower.isBusy()) {
                    pathTimer.resetTimer();
                    Flywheel.shooter(1500);
                    telemetry.addLine("Auto is finished!");
                    telemetry.update();
                }



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

    public class Paths {
        public PathChain Path1;
        public PathChain Path2;

        public PathChain Path3;

        public PathChain Path4;

        public PathChain Path5;

        public PathChain Path6;

        public PathChain Path7;

        public PathChain Path8;

        public Paths(Follower follower) {
            //Path1 = follower.pathBuilder()
            //.addPath(new BezierLine(
            // new Pose(56.000, 8.000),
            // new Pose(57.078, 32.451)
            //))
            //.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
            //.build();

            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            start,

                            PreloadLaunch
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(130))

                    .build();
            Path2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            PreloadLaunch,

                            controlPoint1,

                            intake1

                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(175), Math.toRadians(185))

                    .build();
            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            ClassifierRamp,
                            Launch1


                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(80))

                    .build();
            Path4 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            Launch1,

                            intake2ControlPose,

                            intake2
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(175), Math.toRadians(180))

                    .build();
            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            intake2,

                            Launch1
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(130))

                    .build();
            Path6 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            Launch1,

                            Intake3ControlPoint,

                            Intake3
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(175), Math.toRadians(180))

                    .build();
            Path7 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            Intake3,

                            Launch1
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(130))

                    .build();
            Path8 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            intake1,
                            ClassifierRampControl,
                            ClassifierRamp
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(175), Math.toRadians(180))

                    .build();

        }
    }
}