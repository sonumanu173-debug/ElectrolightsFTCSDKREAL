package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.lang.Math;

public class ShooterCalculations extends OpMode {

    AprilTagDistance tagDistance = new AprilTagDistance();
    AprilTagDistance targetArea = new AprilTagDistance();
    double ta;
    double distance;
    double v0;
    double numerator;
    double denominator;
    double requiredRPM;


    @Override
    public void init() {

    }

    @Override
    public void loop() {
        ta = targetArea.
        distance = tagDistance.getDistanceFromTage();
        numerator = Math.sqrt(9.81 * Math.pow(distance, 2));
        denominator = (Math.pow(2 * Math.cos(63.2) , 2) * (distance * Math.tan(63.2) - 0.85125));
        v0 = numerator / denominator;
        requiredRPM = 10 * (v0)*(v0) + 10 * v0 + 10;
    }

}