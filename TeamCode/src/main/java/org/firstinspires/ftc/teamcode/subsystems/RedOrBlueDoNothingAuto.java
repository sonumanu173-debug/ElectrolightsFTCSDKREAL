package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

import org.firstinspires.ftc.teamcode.AutoBlueNewPosition;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MotifScanning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;



@Autonomous(name="AutoDoNothing", group="Auto")
public class RedOrBlueDoNothingAuto extends NextFTCOpMode {
    public RedOrBlueDoNothingAuto(){
        addComponents(
                new SubsystemComponent(Flywheel.INSTANCE,  Intake.INSTANCE, MotifScanning.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE

        );
    }
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private AutoBlueNewPosition.Paths paths;
    public Pose start = new Pose(56.018,5.6967033, Math.toRadians(90));

    public Pose PreloadLaunch = new Pose(54.871, 97.258, Math.toRadians(130));

    public Pose controlPoint1 = new Pose(50.263, 76.015);

    public Pose intake1 = new Pose(10.44396, 85.767033);

    public Pose ControlPose2 = new Pose(2.8484, 88.932);
    public Pose ControlPose3 = new Pose(3.48132, 113.934);
    public Pose Launch1 = new Pose(56.967033, 94);// THIS will be used for launching all 3 times lol

    public Pose intake2ControlPose = new Pose(84.5011, 58.5495);

    public Pose intake2 = new Pose(9.811, 59.499);

    public Pose Intake3ControlPoint = new Pose(85.1341, 25.635165);

    public Pose Intake3 = new Pose(9.81099, 35.4462);

    public Pose ClassifierRampControl = new Pose(35.1297,72.474725374);

    public Pose ClassifierRamp = new Pose(10.127473,69.94286);









    private MotorEx intakeMotor;
    private MotorEx spindexerMotor;
    private MotorEx frontLeft;
    private MotorEx frontRight;



    private MotorEx backLeft;
    private MotorEx backRight;



    @Override
    public void onInit(){
        backLeft = new MotorEx("backLeft");
        backRight = new MotorEx("backRight");
        frontLeft = new MotorEx("frontLeft");
        frontRight = new MotorEx("frontRight");




        telemetry.addData("It's been initialized and nothing will happen", "WARNING TO DRIVERS: CHANGE THIS TO ANOTHER AUTO IF YOU DON'T INTEND OR IT TO DO NOTHING");

    }
    public void onUpdate(){

        frontLeft.setPower(-0.5);
        frontRight.setPower(-0.5);

        new Delay(1);

        frontLeft.setPower(0);
        frontRight.setPower(0);



        telemetry.addLine("Bam its running nothing");
    }
}