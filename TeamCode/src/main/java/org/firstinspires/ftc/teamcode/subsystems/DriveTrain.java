package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.TeleOp.startingPose;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;

import static dev.nextftc.bindings.Bindings.button;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.units.Angle;
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
        return clip(YAW_KP * txDeg, -YAW_MAX, YAW_MAX);
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

    @Override
    public Command getDefaultCommand() {
        Button Slowmode = button(() -> gamepad1.left_bumper);
        Button Autoaim = button(() -> gamepad1.triangle);
        Autoaim.whenTrue(() -> autolocktrue())
                .whenFalse(() -> autolockfalse());
        Slowmode.whenTrue(() -> slowtrue())
                .whenFalse(() -> slowfalse());
        if(autolock==true){
            limelight.pipelineSwitch(APRILTAG_PIPELINE);
            LLResult result = limelight.getLatestResult();
            hasTag = (result != null) && result.isValid() && !result.getFiducialResults().isEmpty();

            if (hasTag)
            {
                tx = result.getTx(); // deg
                ty = result.getTy(); // deg (positive = tag above crosshair)
                ta = result.getTa(); // %
                gamepad1.rumbleBlips(1);
            }
            else
            {
                tx = ty = ta = 0.0;
            }
            Supplier<Double> yVCtx = () -> visionYawCommand(tx);

            // Get the double value from the supplier
            return new MecanumDriverControlled(
                    fL,
                    fR,
                    bL,
                    bR,
                    Gamepads.gamepad1().leftStickY().map(it -> it * sensistivity),
                    Gamepads.gamepad1().leftStickX().map(it -> -it * sensistivity),
                    yVCtx,//THIS IS FOR HEADING
                    new FieldCentric(imu)
            );
        }
        else {
            if(slow==true)
            {
                return new MecanumDriverControlled(
                        fL,
                        fR,
                        bL,
                        bR,
                        Gamepads.gamepad1().leftStickY().map(it -> it * 0.5),
                        Gamepads.gamepad1().leftStickX().map(it -> -it * 0.5),
                        Gamepads.gamepad1().rightStickX().map(it -> -it * 0.5),
                        new FieldCentric(imu)
                );
            }
            else
            {
                return new MecanumDriverControlled(
                    fL,
                    fR,
                    bL,
                    bR,
                    Gamepads.gamepad1().leftStickY().map(it -> it * sensistivity),
                    Gamepads.gamepad1().leftStickX().map(it -> -it * sensistivity),
                    Gamepads.gamepad1().rightStickX().map(it -> -it * sensistivity),
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
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        follower = Constants.createFollower(ActiveOpMode.hardwareMap());
    }

    @Override
    public void periodic() {
        follower.update();
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double distinch = Math.sqrt(Math.pow((x-0), 2)*Math.pow((y-144), 2));
        double dist = distinch / 39.37;
        if(autolock==true)
        {
            float tps = findTPS((float) dist);
            shooter(tps);
        }
    }
}
