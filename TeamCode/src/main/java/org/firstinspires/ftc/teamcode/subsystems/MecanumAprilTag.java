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

    // --- Simple AprilTag aimer (servo points up/down) ---
    private Servo aimServo;                           // add "aimServo" in RC config
    private static final double AIM_MIN_POS = 0.10;   // mechanical lower limit
    private static final double AIM_MAX_POS = 0.90;   // mechanical upper limit
    private static final double AIM_ZERO_POS = 0.50;  // servo when angle == 0°
    private static final double AIM_DEG_PER_POS = 60.0; // degrees per 1.0 servo position change
    private static final boolean AIM_REVERSED = false;   // flip if motion is inverted
    private static final double AIM_ALPHA = 0.25;        // smoothing (0..1); lower = smoother
    private double aimPos = AIM_ZERO_POS;

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

        // Servos
        //Servo claw  = hardwareMap.servo.get("clawServo");   // Port 0
        Servo wrist = hardwareMap.servo.get("wristServo");  // Port 1
        aimServo    = hardwareMap.servo.get("aimServo");    // NEW: aiming servo
        aimServo.setPosition(AIM_ZERO_POS);

        // Reverse right side (matches your original)
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (isStopRequested()) return;

        limelight.start();

        double wristPos = .5;
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

            // --- Aim servo update (accounts for camera UP pitch) ---
            if (hasTag) {
                // Effective vertical angle relative to robot horizon:
                // camera's fixed upward pitch + measured vertical offset
                double angleToTagDeg = CAM_PITCH_DEG + (AIM_REVERSED ? -ty : ty);
                double desiredPos    = aimFromAngle(angleToTagDeg);
                aimPos = AIM_ALPHA * desiredPos + (1.0 - AIM_ALPHA) * aimPos; // smoothing
                aimServo.setPosition(aimPos);
            }
            // else: keep last aimPos (or park to AIM_ZERO_POS if you prefer)

            // --- Driver 2 slow nudge controls ---
            if (gamepad2.dpad_down)  moveRobot(.5,  0,   0);
            if (gamepad2.dpad_up)    moveRobot(-.5, 0,   0);
            if (gamepad2.dpad_right) moveRobot(0,  -.5,  0);
            if (gamepad2.dpad_left)  moveRobot(0,   .5,  0);

            // --- Driver 1 inputs ---
            double y  = -gamepad1.left_stick_y;       // forward/back
            double x  =  gamepad1.left_stick_x * 1.6; // strafe
            double rx =  gamepad1.right_stick_x;      // yaw

            // Vision yaw assist while holding RT
            if (gamepad1.right_trigger > 0.5 && hasTag) {
                rx = visionYawCommand(tx);
                y += visionForwardCommand(ta); // optional distance hold
            }

            // Turn-in-place dampening
            if (y == 0 || x == 0) rx *= 0.6;

            // Speed toggle (RB)
            if (toggleButton(gamepad1.right_bumper, "g1RightBumper")) powerMultiplier = 0.6;
            else powerMultiplier = 1.0;

            // Claw toggle on driver 2 RB
//            if (toggleButton(gamepad2.right_bumper, "g2RightBumper")) claw.setPosition(0.4); // open
//            else claw.setPosition(0.17); // closed

            // Wrist quick positions
            if (gamepad2.x) wristPos = .25;
            if (gamepad2.y) wristPos = .5;
            if (gamepad2.b) wristPos = .75;
            wrist.setPosition(wristPos);

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
            telemetry.addData("Aim pos", aimPos);
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

    private double visionForwardCommand(double taNow) {
        if (!HOLD_DISTANCE) return 0.0;
        double err = TARGET_TA - taNow; // positive => we're too far
        return clip(FWD_KP * err, -FWD_MAX, FWD_MAX);
    }

    // Map an angle in degrees (relative to robot HORIZON) -> servo position [0..1]
    private double aimFromAngle(double angleDeg) {
        double slope = 1.0 / AIM_DEG_PER_POS;   // servo pos change per degree
        double pos   = AIM_ZERO_POS + slope * angleDeg;
        pos = Math.max(AIM_MIN_POS, Math.min(AIM_MAX_POS, pos));
        return clip01(pos);
    }

    // Toggle helper
    private boolean toggleButton(boolean buttonInput, String buttonName) {
        boolean previousState = previousStates.getOrDefault(buttonName, false);
        boolean toggleState   = toggleStates.getOrDefault(buttonName, false);

        if (buttonInput && !previousState) {
            toggleState = !toggleState;
            toggleStates.put(buttonName, toggleState);

            if (buttonName.equals("g1RightBumper")) {
                gamepad1.rumbleBlips(rumbleNum % 2 + 1);
                rumbleNum++;
            }
        }

        previousStates.put(buttonName, buttonInput);
        return toggleState;
    }
}