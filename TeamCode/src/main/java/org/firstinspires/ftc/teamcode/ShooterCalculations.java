package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.lang.Math;

public class ShooterCalculations extends OpMode {


    //create a new class
    double distance;
    double initalVelocity;
    double numerator;
    double denominator;

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        distance = 10;
        numerator = Math.sqrt(9.81 * Math.pow(distance, 2));
        denominator = (Math.pow(2 * Math.cos(63.2) , 2) * (distance * Math.tan(63.2) - 0.85125));
        initalVelocity = numerator / denominator;
    }

}