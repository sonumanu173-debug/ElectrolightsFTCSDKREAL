package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AprilTagDistance;

import java.lang.Math;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.NextFTCOpMode;

@Configurable
public class Calculations implements Subsystem {

    AprilTagDistance tagDistance = new AprilTagDistance();
    AprilTagDistance targetArea = new AprilTagDistance();
    double ta;
    double distance;
    public static double v0;
    public static double numerator;
    public static double denominator;

    double dist;


    public static double requiredRPM;
    public static double requiredTPS = (28*requiredRPM)/60;

    public static float findTPS(double dist){
        numerator = 9.81 * Math.pow(dist, 2);
        denominator = (2 * Math.pow(Math.cos(1.103048) , 2) * (dist * Math.tan(1.103048) - 0.85125));
        v0 = Math.sqrt(numerator / denominator);
        requiredRPM = -39.357*v0*v0*v0 + 730.79*v0*v0 - 3915.4*v0 + 8565.9;
        requiredTPS = (28*requiredRPM)/60;
        return (float) requiredTPS;
    }


}