package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "MecanumAprilTag")
//@Disabled
public class MecanumAprilTag extends LinearOpMode {
    // --- Limelight & vision fields ---
    private Limelight3A limelight;
    private double tx, ty, ta;
    private boolean hasTag;

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
    private static final double CAM_PITCH_DEG = 75.0; // <-- set to your real mount angle

    // --- drive & mechanisms ---
    private DcMotor backRightMotor, backLeftMotor, frontRightMotor, frontLeftMotor;
    private double moveSpeed = 0.384;

    // --- toggles & state ---
    private final Map<String, Boolean> toggleStates = new HashMap<>();
    private final Map<String, Boolean> previousStates = new HashMap<>();
    int rumbleNum = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);

        // Drivetrain motors
        backRightMotor  = hardwareMap.dcMotor.get("backRight");  // Port 0
        backLeftMotor   = hardwareMap.dcMotor.get("backLeft");   // Port 1
        frontRightMotor = hardwareMap.dcMotor.get("frontRight"); // Port 2
        frontLeftMotor  = hardwareMap.dcMotor.get("frontLeft");  // Port 3


        // Reverse right side (matches your original)
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (isStopRequested()) return;

        limelight.start();

        double powerMultiplier = 1.0;

        while (opModeIsActive()) {
            // --- Read Limelight ---
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
            // else: keep last aimPos (or park to AIM_ZERO_POS if you prefer)

            // --- Driver 2 slow nudge controls ---

            // --- Driver 1 inputs ---
            double y  = -gamepad1.left_stick_y;       // forward/back
            double x  =  gamepad1.left_stick_x * 1.6; // strafe
            double rx =  gamepad1.right_stick_x;      // yaw

            // Vision yaw assist while holding RT
            if (gamepad1.right_trigger > 0.5 && hasTag) {
                rx = visionYawCommand(tx);
            }

            // Turn-in-place dampening
            if (y == 0 || x == 0) rx *= 0.6;

            // Speed toggle (RB)

            powerMultiplier = 1.0;


            // --- Mecanum mixing ---
            double denominator     = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double backLeftPower   = (y - x + rx) / denominator;
            double frontLeftPower  = (x + y + rx) / denominator;
            double backRightPower  = (x + y - rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;

            // Apply speed scaling
            frontLeftMotor.setPower(frontLeftPower * powerMultiplier);
            frontRightMotor.setPower(frontRightPower * powerMultiplier);
            backLeftMotor.setPower(backLeftPower * powerMultiplier);
            backRightMotor.setPower(backRightPower * powerMultiplier);

            // --- Telemetry ---
            telemetry.addData("Tag valid?", hasTag);
            telemetry.addData("tx (deg)", tx);
            telemetry.addData("ty (deg)", ty);
            telemetry.addData("ta (%)",  ta);
            telemetry.addData("Cam pitch (deg)", CAM_PITCH_DEG);
            telemetry.addData("Speed: ", powerMultiplier);
            telemetry.addData("\nFront Left Motor ", frontLeftPower * powerMultiplier * 100);
            telemetry.addData("Front Right Motor ", frontRightPower * powerMultiplier * 100);
            telemetry.addData("Back Left Motor ", backLeftPower * powerMultiplier * 100);
            telemetry.addData("Back Right Motor ", backRightPower * powerMultiplier * 100);
            telemetry.addData("\nG1 Left Stick X", x);
            telemetry.addData("G1 Left Stick Y", y);
            telemetry.addData("G1 Right Stick X", rx);
            telemetry.update();
        }

        limelight.stop();
    }

    // --- Helpers ---

    // Dpad nudge / utility drive
    private void moveRobot(double x, double y, double yaw) {
        x = -x; // preserve your original convention
        double leftFrontPower  = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower   = x + y - yaw;
        double rightBackPower  = x - y + yaw;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        frontLeftMotor.setPower(leftFrontPower);
        frontRightMotor.setPower(rightFrontPower);
        backLeftMotor.setPower(leftBackPower);
        backRightMotor.setPower(rightBackPower);
    }

    private double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private double clip01(double v) { return clip(v, 0.0, 1.0); }

    private double visionYawCommand(double txDeg) {
        if (Math.abs(txDeg) < YAW_DEADBAND_DEG) return 0.0;
        return clip(YAW_KP * txDeg, -YAW_MAX, YAW_MAX);
    }


    // Map an angle in degrees (relative to robot HORIZON) -> servo position [0..1]


}