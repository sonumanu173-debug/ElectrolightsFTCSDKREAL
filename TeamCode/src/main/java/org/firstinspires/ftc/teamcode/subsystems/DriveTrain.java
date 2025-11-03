package org.firstinspires.ftc.teamcode.subsystems;

/*import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.function.Supplier;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;

public class DriveTrain implements Subsystem {
    public static final DriveTrain INSTANCE = new DriveTrain();
    private DriveTrain() { }

    private Limelight3A limelight;

    private double tx, ty, ta;
    private boolean hasTag;

    private boolean autolock;

    // === AprilTag/Limelight align tuning ===
    private static final int APRILTAG_PIPELINE = 8;   // <-- set to your AprilTag pipeline index
    private static final double YAW_KP = 0.050;      // deg -> yaw power (flip sign if turning wrong way)
    private static final double YAW_MAX = 0.7;        // yaw cap
    private static final double YAW_DEADBAND_DEG = 1.0;

    // Optional crude distance hold using target area (leave false to disable)
    private static final boolean HOLD_DISTANCE = false;
    private static final double TARGET_TA = 2.0;
    private static final double FWD_KP = 0.02;
    private static final double FWD_MAX = 0.35;

    // Camera mounting pitch (positive = camera tilted UP relative to robot horizon)
    private static final double CAM_PITCH_DEG = 75.0;
    private MotorEx frontLeftMotor = new MotorEx("frontLeft").brakeMode().reversed();
    private MotorEx frontRightMotor = new MotorEx("frontRight").brakeMode();
    private MotorEx backLeftMotor = new MotorEx("backLeft").brakeMode().reversed();
    private MotorEx backRightMotor = new MotorEx("backRight").brakeMode();
    private IMUEx imu = new IMUEx("imu", Direction.FORWARD, Direction.UP).zeroed();

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

    @Override
    public void initialize() {
        //limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        imu = new IMUEx("imu", Direction.UP, Direction.FORWARD).zeroed();
    }
    @Override
    public Command getDefaultCommand() {
        Button Autoaim = button(() -> gamepad1.triangle);
        Autoaim.whenTrue(() -> autolocktrue())
                .whenFalse(() -> autolockfalse());
        if(autolock==true){
            limelight.pipelineSwitch(APRILTAG_PIPELINE);
            LLResult result = limelight.getLatestResult();
            hasTag = (result != null) && result.isValid() && !result.getFiducialResults().isEmpty();

            if (hasTag) {
                tx = result.getTx(); // deg
                ty = result.getTy(); // deg (positive = tag above crosshair)
                ta = result.getTa(); // %
                gamepad1.rumbleBlips(1);
            } else {
                tx = ty = ta = 0.0;
            }
            Supplier<Double> yVCtx = () -> visionYawCommand(tx);

            // Get the double value from the supplier
            return new MecanumDriverControlled(
                    frontLeftMotor,
                    frontRightMotor,
                    backLeftMotor,
                    backRightMotor,
                    Gamepads.gamepad1().leftStickY().negate(),
                    Gamepads.gamepad1().leftStickX(),
                    yVCtx,//THIS IS FOR HEADING
                    new FieldCentric(imu)
            );
        }
        //else
        //{
            return new MecanumDriverControlled(
                    frontLeftMotor,
                    frontRightMotor,
                    backLeftMotor,
                    backRightMotor,
                    Gamepads.gamepad1().leftStickY().negate(),
                    Gamepads.gamepad1().leftStickX(),
                    Gamepads.gamepad1().rightStickX(),//,//THIS IS FOR HEADING - PID OUTPUT GOES HERE
                    new FieldCentric(imu)
            );
        //}
    }
}*/

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.units.Angle;
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

    // === AprilTag/Limelight align tuning ===
    private static final int APRILTAG_PIPELINE = 8;   // <-- set to your AprilTag pipeline index
    private static final double YAW_KP = 0.050;      // deg -> yaw power (flip sign if turning wrong way)
    private static final double YAW_MAX = 0.7;        // yaw cap
    private static final double YAW_DEADBAND_DEG = 1.0;

    // Optional crude distance hold using target area (leave false to disable)
    private static final boolean HOLD_DISTANCE = false;
    private static final double TARGET_TA = 2.0;
    private static final double FWD_KP = 0.02;
    private static final double FWD_MAX = 0.35;

    // Camera mounting pitch (positive = camera tilted UP relative to robot horizon)
    private static final double CAM_PITCH_DEG = 75.0;

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
    public static final MotorEx fL = new MotorEx("frontLeft").brakeMode();
    public static final MotorEx fR = new MotorEx("frontRight").brakeMode().reversed();
    public static final MotorEx bL = new MotorEx("backLeft").brakeMode();
    public static final MotorEx bR = new MotorEx("backRight").brakeMode().reversed();

    public static double sensistivity = 1;

    private IMUEx imu;

    @Override
    public Command getDefaultCommand() {
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

    @Override
    public void initialize() {
        //follower.startTeleopDrive();
        imu = new IMUEx("imu", Direction.RIGHT, Direction.FORWARD).zeroed();
    }
/*
    @Override
    public void periodic() {
        follower.update();
    }*/
}
