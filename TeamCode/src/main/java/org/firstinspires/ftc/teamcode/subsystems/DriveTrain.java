package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;


@Configurable
public class DriveTrain implements Subsystem {

    public static final DriveTrain INSTANCE = new DriveTrain();
    private DriveTrain() { }

    private Limelight3A limelight;

    private Follower follower;
    public static boolean spinstop;
    private double tx;
    private boolean hasTag;

    private boolean autolock = false;

    private boolean slow = false;
    private static boolean intakeReverse = false;

    // === AprilTag/Limelight align tuning ===
    private static final int APRILTAG_PIPELINE = 8;   // <-- set to your AprilTag pipeline index
    private static final double YAW_KP = 0.050;      // deg -> yaw power (flip sign if turning wrong way)
    private static final double YAW_MAX = 0.7;        // yaw cap
    private static final double YAW_DEADBAND_DEG = 1.0;

    private double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public static boolean shooting = false;

    public static boolean indexing = true;

    public static boolean spinstop1 = false;

    public static void shootingtrue(){
        shooting = true;
    }

    public static void shootingfalse(){
        shooting = false;
    }

    public static void indextrue(){
        indexing = true;
    }

    public static void indexfalse(){
        indexing = false;
    }
    public static void spinstoptrue() {spinstop = true;}
    public static void spinstopfalse() {spinstop = false;}
    public static void spinstop1true() {spinstop1 = true;}
    public static void spinstop1false() {spinstop1 = false;}
    public static void intakeReverseTrue() {intakeReverse = true;}
    public static void intakeReverseFalse() {intakeReverse = false;}
    public static double spindexvelocity;
    public static MotorEx spindex = new MotorEx("spindexer");
    public static boolean spinr=false;

    public static void SpinReverse() {
        spindex.setPower(-0.2);
        spinr=true;

    }
    private MotorEx intakeMotor;


    public static float configvelocity = 1400; //far zone - ~1500. near zone - ~1200-1300

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

    public static final ServoEx servoPos = new ServoEx("servoPos");

    private double visionYawCommand(double txDeg) {
        if (Math.abs(txDeg) < YAW_DEADBAND_DEG) return 0.0;
        return 0.5*clip(YAW_KP * txDeg, -YAW_MAX, YAW_MAX);
    }

    private void autolocktrue(){
        autolock = true;
    }

    private void autolockfalse(){
        autolock = false;
    }

    private void slowtrue(){
        slow = true;
    }

    private void slowfalse(){
        slow = false;
    }
    public static final MotorEx fL = new MotorEx("frontLeft").brakeMode();
    public static final MotorEx fR = new MotorEx("frontRight").brakeMode();
    public static final MotorEx bL = new MotorEx("backLeft").brakeMode();
    public static final MotorEx bR = new MotorEx("backRight").brakeMode();

    public static double sensistivity = 1;

    private IMUEx imu;

    public Supplier<Double> yVCtx;

    ColorSense1 bench = new ColorSense1();
    ColorSense2 bench2 = new ColorSense2();


    @Override
    public Command getDefaultCommand() {
        if(shooting==true){
            if(indexing==true){
                if(ColorSense1.getDetectedColor(ActiveOpMode.telemetry())== ColorSense1.detectedColor.PURPLE && ColorSense2.getDetectedColor(ActiveOpMode.telemetry())==ColorSense2.detectedColor.GREEN){
                    //raise servo
                    servoPos.setPosition(0.1);
                    try {
                        Thread.sleep(1500);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    servoPos.setPosition(0.0);
                }
            }
            else{
                servoPos.setPosition(0.1);
                try {
                    Thread.sleep(1500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                servoPos.setPosition(0.0);
            }
        }
        Gamepads.gamepad1().triangle().whenBecomesTrue(() -> autolocktrue())
                .whenFalse(() -> autolockfalse());
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(() -> slowtrue())
                .whenFalse(() -> slowfalse());
        Gamepads.gamepad2().rightBumper().whenBecomesTrue(() -> spinstoptrue())
                .whenFalse(() -> spinstopfalse());
        Gamepads.gamepad2().leftTrigger().greaterThan(0).whenBecomesTrue(()-> intakeReverseTrue())
                .whenFalse(()-> intakeReverseFalse());
        Gamepads.gamepad2().rightTrigger().greaterThan(0).whenBecomesTrue(()-> SpinReverse())
                .whenFalse(()-> spinr=false);
        Gamepads.gamepad2().leftBumper().whenBecomesTrue(()-> spinstop1true())
                .whenFalse(()-> spinstop1false());

        //if(ColorSense1)
        if (intakeReverse == true) {
            intakeMotor.setPower(1);
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                e.printStackTrace();

            }
            intakeReverse = false;
            intakeMotor.setPower(0);
        }
        if (spinstop == true) {
            ColorSense1.detectedColor yes = bench.getDetectedColor(ActiveOpMode.telemetry());
            ColorSense2.detectedColor ye = bench2.getDetectedColor(ActiveOpMode.telemetry());
            if (yes!= ColorSense1.detectedColor.ERROR && ye!=ColorSense2.detectedColor.ERROR) {
                spindex.setPower(0);
            }
        }
        if (spinstop1 == true) {
            ColorSense1.detectedColor yes = bench.getDetectedColor(ActiveOpMode.telemetry());
            ColorSense2.detectedColor ye = bench2.getDetectedColor(ActiveOpMode.telemetry());
            if (yes!= ColorSense1.detectedColor.ERROR || ye!=ColorSense2.detectedColor.ERROR) {
                spindex.setPower(0);
            }
        }
        if (spinstop1 == false && spinstop == false && spinr==false) {
            spin(2);
        }
        if (autolock == true) {
            limelight.pipelineSwitch(APRILTAG_PIPELINE);
            LLResult result = limelight.getLatestResult();
            hasTag = (result != null) && result.isValid() && !result.getFiducialResults().isEmpty();

            if (hasTag) {
                tx = result.getTx(); // deg
                ActiveOpMode.telemetry().addData("Tx", tx);
            } else {
                tx = 0.0;
            }
            yVCtx = () -> visionYawCommand(tx);
            return new MecanumDriverControlled(
                    fL,
                    fR,
                    bL,
                    bR,
                    Gamepads.gamepad1().leftStickY().map(it -> -it),
                    Gamepads.gamepad1().leftStickX().map(it -> it),
                    yVCtx,
                    new FieldCentric(imu)
            );
        }
        else // IF AUTOLOCK IS NOT ON
        {
            if (slow == true) {
                return new MecanumDriverControlled(
                        fL,
                        fR,
                        bL,
                        bR,
                        Gamepads.gamepad1().leftStickY().map(it -> -it * 0.4),
                        Gamepads.gamepad1().leftStickX().map(it -> it *0.4),
                        Gamepads.gamepad1().rightStickX().map(it -> it * 0.4 * 0.75),
                        new FieldCentric(imu)
                );
            }
            else //IF SLOW IS OFF
            {
                //if doesnt work, remove else here
                return new MecanumDriverControlled(
                        fL,
                        fR,
                        bL,
                        bR,
                        Gamepads.gamepad1().leftStickY().map(it -> -it),
                        Gamepads.gamepad1().leftStickX().map(it -> it),
                        Gamepads.gamepad1().rightStickX().map(it -> it * 0.75),
                        new FieldCentric(imu)
                );
            }
        }
    }

    @Override
    public void initialize() {
        imu = new IMUEx("imu", Direction.LEFT, Direction.FORWARD).zeroed();
        spindex = new MotorEx("spindexer");
        intakeMotor = new MotorEx("intake");
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();
        //spin(2);
        follower = Constants.createFollower(ActiveOpMode.hardwareMap());
        follower.setStartingPose(new Pose(25, -4, Math.toRadians(90)));
        follower.update();
        servoPos.setPosition(0.0);
        bench.init(ActiveOpMode.hardwareMap());
        bench2.init(ActiveOpMode.hardwareMap());
    }

    @Override
    public void periodic() {

        LLResult result = limelight.getLatestResult();
        hasTag = (result != null) && result.isValid() && !result.getFiducialResults().isEmpty();

        if (hasTag) {
            tx = result.getTx();
        } else {
            tx = 0.0;
        }
        yVCtx = () -> visionYawCommand(tx);
        //ActiveOpMode.telemetry().update();

        follower.update();
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double distinch = Math.sqrt(Math.pow(x, 2)+Math.pow((y-144), 2));
        double dist = distinch / 39.37;
        ActiveOpMode.telemetry().addData("Distance", distinch);
        ActiveOpMode.telemetry().addData("X", x);
        ActiveOpMode.telemetry().addData("Y", y);
        if(autolock==true)
        {
            float tps = findTPS((float) dist);
            shooter(tps);
            limelight.pipelineSwitch(APRILTAG_PIPELINE);
        }
        ActiveOpMode.telemetry().update();
    }
}
