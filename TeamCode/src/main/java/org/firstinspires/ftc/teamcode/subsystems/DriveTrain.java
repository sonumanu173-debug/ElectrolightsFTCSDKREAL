package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;


@Configurable
public class DriveTrain implements Subsystem {

    public static final DriveTrain INSTANCE = new DriveTrain();
    private DriveTrain() { }

    private Limelight3A limelight;

    private double tx, ty, ta;
    private boolean hasTag;

    private boolean autolock;

    private boolean slow;

    // === AprilTag/Limelight align tuning ===
    private static final int APRILTAG_PIPELINE = 8;   // <-- set to your AprilTag pipeline index
    private static final double YAW_KP = 0.050;      // deg -> yaw power (flip sign if turning wrong way)
    private static final double YAW_MAX = 0.7;        // yaw cap
    private static final double YAW_DEADBAND_DEG = 1.0;

    private double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private double visionYawCommand(double txDeg) {
        if (Math.abs(txDeg) < YAW_DEADBAND_DEG) return 0.0;
        return -0.5*clip(YAW_KP * txDeg, -YAW_MAX, YAW_MAX);
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
    public static final MotorEx fR = new MotorEx("frontRight").brakeMode().reversed();
    public static final MotorEx bL = new MotorEx("backLeft").brakeMode();
    public static final MotorEx bR = new MotorEx("backRight").brakeMode().reversed();

    public static double sensistivity = 1;

    private IMUEx imu;

    public Supplier<Double> yVCtx;

    @Override
    public Command getDefaultCommand() {
        Gamepads.gamepad1().triangle().whenBecomesTrue(() -> autolocktrue())
                .whenFalse(() -> autolockfalse());
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(() -> slowtrue())
                .whenFalse(() -> slowfalse());
        if (autolock == true) {
            limelight.pipelineSwitch(APRILTAG_PIPELINE);
            LLResult result = limelight.getLatestResult();
            hasTag = (result != null) && result.isValid() && !result.getFiducialResults().isEmpty();

            if (hasTag) {
                tx = result.getTx(); // deg
                ty = result.getTy(); // deg (positive = tag above crosshair)
                ta = result.getTa(); // %
                ActiveOpMode.telemetry().addData("Tx", tx);
            } else {
                tx = ty = ta = 0.0;
            }
            yVCtx = () -> visionYawCommand(tx);
            return new MecanumDriverControlled(
                    fL,
                    fR,
                    bL,
                    bR,
                    Gamepads.gamepad1().leftStickY().map(it -> it),
                    Gamepads.gamepad1().leftStickX().map(it -> -it),
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
                        Gamepads.gamepad1().leftStickY().map(it -> it * 0.4),
                        Gamepads.gamepad1().leftStickX().map(it -> -it *0.4),
                        Gamepads.gamepad1().rightStickX().map(it -> -it * 0.4 * 0.75),
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
                        Gamepads.gamepad1().leftStickY().map(it -> it),
                        Gamepads.gamepad1().leftStickX().map(it -> -it),
                        Gamepads.gamepad1().rightStickX().map(it -> -it * 0.75),
                        new FieldCentric(imu)
                );
            }
        }
    }

    @Override
    public void initialize() {
        imu = new IMUEx("imu", Direction.LEFT, Direction.FORWARD).zeroed();
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();
        follower = Constants.createFollower(ActiveOpMode.hardwareMap());
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));
        follower.update();
    }

    @Override
    public void periodic() {

        LLResult result = limelight.getLatestResult();
        hasTag = (result != null) && result.isValid() && !result.getFiducialResults().isEmpty();

        if (hasTag) {
            tx = result.getTx(); // deg
            ActiveOpMode.telemetry().addData("Tx", tx);
        } else {
            tx = 0.0;
        }
        yVCtx = () -> visionYawCommand(tx);
        ActiveOpMode.telemetry().addData("yVCtx", yVCtx);
        ActiveOpMode.telemetry().update();

        follower.update();
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double distinch = Math.sqrt(Math.pow((x-0), 2)*Math.pow((y-144), 2));
        double dist = distinch / 39.37;
        ActiveOpMode.telemetry().addData("Distance", dist);
        ActiveOpMode.telemetry().update();
        if(autolock==true)
        {
            float tps = findTPS((float) dist);
            shooter(tps);
        }
    }
}
