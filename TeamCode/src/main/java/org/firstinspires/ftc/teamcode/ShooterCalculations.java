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

    public static double requiredRPM;
    public static double requiredTPS = (28*requiredRPM)/60;



    @Override
    public void onInit() {

    }

//    @Override
    public void onUpdate() {
        distance = 3.5;
        numerator = 9.81 * Math.pow(distance, 2);
        denominator = (2 * Math.pow(Math.cos(1.103048) , 2) * (distance * Math.tan(1.103048) - 0.85125));
        v0 = Math.sqrt(numerator / denominator);
        requiredRPM = 26.613*v0*v0 + 576.17*v0 - 1495.9;
        //requiredRPM = 4500.00;
        requiredTPS = (28*requiredRPM)/60;
        shooter((float) requiredTPS);
    }

}