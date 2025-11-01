package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.SerialNumber;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import static dev.nextftc.bindings.Bindings.*;

import java.util.function.Supplier;

public class Drivetrain implements Subsystem {
    public static final Drivetrain INSTANCE = new Drivetrain();
    private Drivetrain() { }

    SerialNumber LLSerialNumber = SerialNumber.fromString("ye");
    private Limelight3A limelight; /*=new Limelight3A(LLSerialNumber,"limelight");*/
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
    private IMUEx imu = new IMUEx("imu", Direction.UP, Direction.FORWARD).zeroed();

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
        else
        {
            return new MecanumDriverControlled(
                    frontLeftMotor,
                    frontRightMotor,
                    backLeftMotor,
                    backRightMotor,
                    Gamepads.gamepad1().leftStickY().negate(),
                    Gamepads.gamepad1().leftStickX(),
                    Gamepads.gamepad1().rightStickX(),//THIS IS FOR HEADING - PID OUTPUT GOES HERE
                    new FieldCentric(imu)
            );
        }
    }
}
