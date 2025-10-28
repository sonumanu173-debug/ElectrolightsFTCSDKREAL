package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.flywheelpid.shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.lang.Math;

import dev.nextftc.ftc.NextFTCOpMode;

@Configurable
public class ShooterCalculations extends NextFTCOpMode {

    AprilTagDistance tagDistance = new AprilTagDistance();
    AprilTagDistance targetArea = new AprilTagDistance();
    double ta;
    double distance;
    double v0;
    double numerator;
    double denominator;

    public static double requiredRPM = 4500.00;
    public static double requiredTPS = (28*requiredRPM)/60;



    @Override
    public void onInit() {

    }

    @Override
    public void onUpdate() {
        distance = 10;
        numerator = Math.sqrt(9.81 * Math.pow(distance, 2));
        denominator = (Math.pow(2 * Math.cos(63.2) , 2) * (distance * Math.tan(63.2) - 0.85125));
        v0 = numerator / denominator;
        //yes = 10 * (v0)*(v0) + 10 * v0 + 10; - this is placeholder
        //requiredRPM = 0; //testing, idk why flywheel still rotates when this is 0
        requiredRPM = 4500.00;
        requiredTPS = (28*requiredRPM)/60;
        shooter((float) requiredTPS);
    }

}