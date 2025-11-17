

package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AutoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ColorSense1;
import org.firstinspires.ftc.teamcode.subsystems.ColorSense2;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MotifScanning;
import org.firstinspires.ftc.teamcode.subsystems.realAutoSubsystemCommand;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;

import java.time.Duration;


@Autonomous(name = "Auto Blue Default", group = "Autonomous")
@Configurable
public class AutoBlueDefaultPosition extends NextFTCOpMode {
    public AutoBlueDefaultPosition(){
        addComponents(
                new SubsystemComponent(Flywheel.INSTANCE,  Intake.INSTANCE, MotifScanning.INSTANCE, realAutoSubsystemCommand.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE

        );
    }
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    public static final ServoEx servoPos = new ServoEx("servoPos");
    private Paths paths;
    public Pose start = new Pose(24.677419354838708,125.70967741935485, Math.toRadians(125));

    public Pose PreLoadLaunch1 = new Pose(57.6,101.27472527472527);

    public Pose ControlPoint1 = new Pose(50.225806451612904,80.41935483870968);

    public Pose Intake1 = new Pose(7.83871,84.48387096774194);

    public Pose ClassifierRampPoint = new Pose(69.38709677419355,69.67741935483872);

    public Pose ClassifierRamp = new Pose(9,74.61290322580643);

    public Pose Launch1 = new Pose(62.12903225806452, 96.67741935483872);

    public Pose ControlPoint2 = new Pose(63.87096774193548,53.70967741935483);

    public Pose Intake2 = new Pose(3.1935483870967745,59.516129032258064);

    public Pose ControlPoint3 = new Pose(72.87096774193549,25.258064516129032);

    public Pose Intake3 = new Pose(4.35483870967742,37.16129032258064);

    public Pose Teleop1 = new Pose(72.29032258064517,46.16129032258064);













    private MotorEx intakeMotor;

    private MotorEx spindexerMotor;

    private boolean path2Following= false;
    int ball1Color = 0;
    int ball2Color = 0;
    int ball3Color = 0;
    int tagId = 0;
    private ServoEx servo = new ServoEx("servoPos");

    public static double spindexvelocity;
    public static MotorEx spindex = new MotorEx("spindexer");

    ColorSense1 bench = new ColorSense1();
    Command pathCommand = new LambdaCommand()
            .setStart(() -> follower.followPath(paths.Intake1set))
            .setIsDone(() -> !follower.isBusy())
            .named("Path2 Command");
    Command pathCommand2 = new LambdaCommand()
            .setStart(() -> follower.followPath(paths.Intake2ndSet))
            .setIsDone(() -> !follower.isBusy())
            .named("Path4 Command");
    Command pathCommand3 = new LambdaCommand()
            .setStart(() -> follower.followPath(paths.Intake3rdSet))
            .setIsDone(() -> !follower.isBusy())
            .named("Path4 Command");

    public static MotorEx flywheel = new MotorEx("launchingmotor").reversed();
    private realAutoSubsystemCommand stopSpindexer = new realAutoSubsystemCommand();

    ColorSense2 bench2 = new ColorSense2();

    public static void velocityControlWithFeedforwardExample(KineticState currentstate, float configtps) {
        ControlSystem controller = ControlSystem.builder()
                .velPid(0.1, 0.01, 0.05) // Velocity PID with kP=0.1, kI=0.01, kD=0.05
                .basicFF(0.0067, 0.0, 0.01) // Basic feedforward with kV=0.02, kA=0.0, kS=0.01 //pid tuning
                .build();

        controller.setGoal(new KineticState(0.0, configtps, 0.0));

        double power = controller.calculate(currentstate);
        spindex.setPower(power);
    }
    public static void spin(float tps) {
        BindingManager.update();
        spindexvelocity = spindex.getVelocity();
        KineticState currentState = new KineticState(0, spindexvelocity, 0.0);
        velocityControlWithFeedforwardExample(currentState, tps);
    }



    public void onInit() {
        telemetry.addLine("Initializing Follower...");

        bench.init(ActiveOpMode.hardwareMap());
        bench2.init(ActiveOpMode.hardwareMap());
        telemetry.update();
        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);
        intakeMotor = new MotorEx("intake");
        spindexerMotor = new MotorEx("spindexer");
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        follower.setStartingPose(start);
        Flywheel.shooter(0);
        pathState = 0;
        telemetry.addLine("Follower + IMU + Odo Pods initialized successfully!");
        telemetry.addLine("Initialization complete!");
        telemetry.update();
    }


    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        int tag=MotifScanning.INSTANCE.findMotif();
        follower.followPath(paths.PreLoadLaunch);
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

                    follower.turnToDegrees(75);
                    tagId = MotifScanning.findMotif();

                    /*if(tagId == 23){
                        ball1Color = 2;
                        ball2Color = 2;
                        ball3Color = 1;

                    }
                    else if(tagId == 22){
                        ball1Color = 2;
                        ball2Color = 1;
                        ball3Color = 2;

                    }
                    else if(tagId == 21){
                        ball1Color = 1;
                        ball2Color = 2;
                        ball3Color = 2;

                    }*/
                    new Delay(1.5).schedule();

                    telemetry.addLine("Path 1 has been completed btw, that means it's going to launch in a couple of seconds");
                    // Im going to add spindexer logic here once its been done and spin the flywheel, we r so cooked
                    follower.turnToDegrees(130);
                    flywheel.setPower(0.3);

                    spindex.setPower(0.1);
                    servoPos.setPosition(0.2);

                    new Delay(3).schedule();
                    flywheel.setPower(0);

                    spindex.setPower(0);
                    servoPos.setPosition(0);

                    new ParallelGroup(
                            pathCommand,
                            realAutoSubsystemCommand.INSTANCE.stopSpinDexer()
                    ).schedule();

                    telemetry.addLine("Started path 2 to intake the balls");
                    pathState++;

                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    String ballColor1 = String.valueOf(ColorSense1.getDetectedColor(telemetry));
                    String ballColor2 = String.valueOf(ColorSense2.getDetectedColor(telemetry));
                    if (tagId == 23) {
                        if (ballColor1 != "isPurple" && ballColor2 != "isGreen") {
                            spindex.setPower(0.05);
                            new Delay(1).schedule();
                            spindex.setPower(0);


                        }

                    }
                    if (tagId == 22) {
                        if (ballColor1 != "isPurple" && ballColor2 != "isPurple") {
                            spindex.setPower(0.05);
                            new Delay(1).schedule();
                            spindex.setPower(0);




                        }


                    }
                    if (tagId == 21) {
                        if (ballColor1 != "isGreen" && ballColor2 != "isPurple") {
                            spindex.setPower(0.05);
                            new Delay(1).schedule();
                            spindex.setPower(0);




                        }


                    }
                    follower.followPath(paths.ClassifierRamp1);
                    pathState++;
                }
            case 2:
                if (!follower.isBusy()){
                    pathTimer.resetTimer();
                    follower.followPath(paths.Launch1Real);
                    pathState++;

                }
            case 3:
                if(!follower.isBusy()){
                    pathTimer.resetTimer();
                    servoPos.setPosition(0.1);
                    flywheel.setPower(0.4);
                    spindex.setPower(0.1);
                    new Delay(2.5).schedule();
                    servoPos.setPosition(0);
                    flywheel.setPower(0);
                    spindex.setPower(0);

                    new ParallelGroup(pathCommand2,realAutoSubsystemCommand.INSTANCE.stopSpinDexer());
                    pathState++;


                }
            case 4:
                if(!follower.isBusy()){
                    pathTimer.resetTimer();
                    String ballColor1 = String.valueOf(ColorSense1.getDetectedColor(telemetry));
                    String ballColor2 = String.valueOf(ColorSense2.getDetectedColor(telemetry));
                    if (tagId == 23) {
                        if (ballColor1 != "isPurple" && ballColor2 != "isGreen") {
                            spindex.setPower(0.05);
                            new Delay(1).schedule();
                            spindex.setPower(0);


                        }

                    }
                    if (tagId == 22) {
                        if (ballColor1 != "isPurple" && ballColor2 != "isPurple") {
                            spindex.setPower(0.05);
                            new Delay(1).schedule();
                            spindex.setPower(0);




                        }


                    }
                    if (tagId == 21) {
                        if (ballColor1 != "isGreen" && ballColor2 != "isPurple") {
                            spindex.setPower(0.05);
                            new Delay(1).schedule();
                            spindex.setPower(0);




                        }


                    }
                    follower.followPath(paths.Launch2);
                    pathState++;


                }
            case 5:
                if(!follower.isBusy()){
                    pathTimer.resetTimer();
                    servoPos.setPosition(0.1);
                    flywheel.setPower(0.4);
                    spindex.setPower(0.1);
                    new Delay(2.5).schedule();
                    servoPos.setPosition(0);
                    flywheel.setPower(0);
                    spindex.setPower(0);
                    new ParallelGroup(pathCommand3,realAutoSubsystemCommand.INSTANCE.stopSpinDexer());
                    pathState++;

                }
            case 6:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    String ballColor1 = String.valueOf(ColorSense1.getDetectedColor(telemetry));
                    String ballColor2 = String.valueOf(ColorSense2.getDetectedColor(telemetry));
                    if (tagId == 23) {
                        if (ballColor1 != "isPurple" && ballColor2 != "isGreen") {
                            spindex.setPower(0.05);
                            new Delay(1).schedule();
                            spindex.setPower(0);


                        }

                    }
                    if (tagId == 22) {
                        if (ballColor1 != "isPurple" && ballColor2 != "isPurple") {
                            spindex.setPower(0.05);
                            new Delay(1).schedule();
                            spindex.setPower(0);




                        }


                    }
                    if (tagId == 21) {
                        if (ballColor1 != "isGreen" && ballColor2 != "isPurple") {
                            spindex.setPower(0.05);
                            new Delay(1).schedule();
                            spindex.setPower(0);




                        }


                    }
                    follower.followPath(paths.Launch3);
                    pathState++;


                }
            case 7:
                if(!follower.isBusy()){
                    pathTimer.resetTimer();
                    servoPos.setPosition(0.1);
                    flywheel.setPower(0.4);
                    spindex.setPower(0.1);
                    new Delay(2.5).schedule();
                    servoPos.setPosition(0);
                    flywheel.setPower(0);
                    spindex.setPower(0);
                    follower.followPath(paths.teleOp);
                    telemetry.addLine("Auto is Finished!");
                    telemetry.update();

                }



               /* while(path2Following==true){
                    ColorSense1.detectedColor yes = bench.getDetectedColor(ActiveOpMode.telemetry());
                    ColorSense2.detectedColor ye = bench2.getDetectedColor(ActiveOpMode.telemetry());
                    if (yes != ColorSense1.detectedColor.ERROR && ye != ColorSense2.detectedColor.ERROR) {
                        spindex.setPower(0);
                        ActiveOpMode.telemetry().addLine("spinstopped for 2");

                        try {
                            Thread.sleep(1000);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        spindex.setPower(0.1);

                    }

                }*/



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
        public PathChain PreLoadLaunch;
        public PathChain Intake1set;

        public PathChain ClassifierRamp1;

        public PathChain Launch1Real;

        public PathChain Intake2ndSet;

        public PathChain Launch2;

        public PathChain Intake3rdSet;

        public PathChain Launch3;

        public PathChain teleOp;

        public Paths(Follower follower) {
            //Path1 = follower.pathBuilder()
            //.addPath(new BezierLine(
            // new Pose(56.000, 8.000),
            // new Pose(57.078, 32.451)
            //))
            //.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
            //.build();

            PreLoadLaunch = follower.pathBuilder()
                    .addPath(new BezierLine(
                            start,
                            PreLoadLaunch1
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(130))
                    //.setVelocityConstraint(50)
                    .build();
            Intake1set = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            PreLoadLaunch1,
                            ControlPoint1,
                            Intake1

                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(180))

                    .build();
            ClassifierRamp1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            Intake1,
                            ClassifierRampPoint,
                            ClassifierRamp


                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();
            Launch1Real = follower.pathBuilder()
                    .addPath(new BezierLine(
                            ClassifierRamp,
                            Launch1
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))

                    .build();
            Intake2ndSet = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            Launch1,
                            ControlPoint2,
                            Intake2

                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(180))

                    .build();
            Launch2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            Intake2,
                            Launch1
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))

                    .build();
            Intake3rdSet = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            Launch1,
                            ControlPoint3,
                            Intake3

                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(180))

                    .build();
            Launch3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            Intake3,
                            Launch1
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))

                    .build();
            teleOp = follower.pathBuilder()
                    .addPath(new BezierLine(
                            Launch1,
                            Teleop1

                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

        }
    }
}