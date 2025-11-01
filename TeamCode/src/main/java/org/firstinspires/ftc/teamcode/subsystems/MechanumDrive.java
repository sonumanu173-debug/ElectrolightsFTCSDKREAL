package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Teleop yes", group = "Main")// Setting it to teleop gng
public class MechanumDrive extends OpMode { // Public class for opmode because thats what u gotta do

    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor; // Setting motor variables with dcmotor
    private IMU imu; // Setting IMU with variable
    private boolean fieldCentric = true; // Setting field centric to true at default but obv I made sure we can switch out of it by pressing Y if we need robot centric
    private double maxSpeed = 1.0; // Setting max speeed

    private boolean yPressedLast = false; // to prevent rapid toggling for Y so it doesnt keep switching

    @Override
    public void init() {// This is the initialization process, everything here initializes when ypou press it on the driver hub
        telemetry.addLine("Initializing the teleop...");

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft"); // Setting motors to hardwaremap and specific locations
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft"); //Bro i dont think ur slow so im not even going to explain this
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRight");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE); // Pretty straightforward imo, setting motor directions to reverse
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Makes it run using encoders so it tracks rotations, etc. AND PLS READ COMMENTS, I SPENT A WHILE WORKING ON IT ;)
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot( // Setting the orientation of the bobot for imu
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        imu.initialize(new IMU.Parameters(orientation));

        telemetry.addLine("Initialized"); // Ensuring we know whats working and what doesnt since this will project onto the driverhub
    }

    @Override
    public void loop() { // This is actually where we're going to do everything and it's in a loop to ensure once its run, it always keeps moving da bobot
        double forward = -gamepad1.left_stick_y; // Self explanatory imo but its using the y of the stick(joystick)
        double strafe = gamepad1.left_stick_x; // Strafing(like moving diagonal, etc)
        double rotate = gamepad1.right_stick_x; // Rotating the robot like in fortnite(deb and ayussi reference iykyk)

        // this toggles field centric to robot centric and robot centric to field centric, dependent on which it's already on
        if (gamepad1.y && !yPressedLast) {
            fieldCentric = !fieldCentric;
        }
        yPressedLast = gamepad1.y;

        if (fieldCentric) { // Checks if its field centric and if it is, it moves relative to the field yipeeee
            driveFieldRelative(forward, strafe, rotate);
        } else { // or else it js chills and moves robot centric(moving based on the robot)
            drive(forward, strafe, rotate);
        }

        telemetry.addData("Mode", fieldCentric ? "Field-Centric" : "Robot-Centric"); // Lets us know whether its field centric or robot centric
        telemetry.addData("Yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)); // its straightforward monke
        telemetry.update(); // Actually inputs all of this into the driverhub for the people to seee
    }

    public void drive(double forward, double strafe, double rotate) { // ALMOST DONE WITH COMMENTS BUT THIS ACTUALLY DRIVES THE ROBOT
        double frontLeftPower = forward + strafe + rotate; //im writing this with onion in my eyes but it sets the power to this based on the other values so front might have more or less depending on the inputs
        double backLeftPower = forward - strafe + rotate; //again input wise as said above, my eyes r burning ;(
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double max = Math.max(1.0, Math.max(Math.abs(frontLeftPower), // Calculates speed for back and front
                Math.max(Math.abs(backLeftPower),
                        Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)))));

        frontLeftMotor.setPower(maxSpeed * (frontLeftPower / max)); // Calculates power
        backLeftMotor.setPower(maxSpeed * (backLeftPower / max));
        frontRightMotor.setPower(maxSpeed * (frontRightPower / max));
        backRightMotor.setPower(maxSpeed * (backRightPower / max));
    }

    public void driveFieldRelative(double forward, double strafe, double rotate) { // its driving field relative THIS CONFUSES MOST PEOPLE INCLUDING ME but please try your best to understand
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); // Gets the current robot heading
        double rotX = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading); // Drives this relative to the field, I cant explain it, this is what most teams use to do field centric driving
        double rotY = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);
        drive(rotY, rotX, rotate); // This actually moves and drives the robot
    }
} // IM FINALLY DONE, THE ONION IS STILL IN MY EYES BTW
