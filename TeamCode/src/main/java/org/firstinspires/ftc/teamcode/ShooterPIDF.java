package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp
public class ShooterPIDF extends OpMode {

    AprilTagDistance tagDistance = new AprilTagDistance();

    DcMotor launcher;
    IMU imu;

    double distance;


    @Override
    public void init() {
        //map hardware
        launcher = hardwareMap.get(DcMotor.class, "launcher"); //"launcher" has to be name in control hub
        imu = hardwareMap.get(IMU.class, "imu"); //"imu" has to be name in control hub

        //set motor direction
        launcher.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        distance = tagDistance.distance;
    }

}
