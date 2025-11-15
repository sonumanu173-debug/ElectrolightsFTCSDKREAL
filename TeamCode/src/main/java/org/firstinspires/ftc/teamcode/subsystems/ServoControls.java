package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.launchertestservo;

import dev.nextftc.ftc.NextFTCOpMode;



@TeleOp(name = "Servo Test", group = "TeleOp")
public class ServoControls extends NextFTCOpMode{
    launchertestservo bench = new launchertestservo();
    @Override
    public void onInit() {
        bench.init(hardwareMap);
        bench.setServoPos(0.8);
    }

    @Override
    public void onUpdate() {
        if (gamepad1.circle) {

            double S = 0.1;

           bench.setServoPos(S);


        }
        else{
            bench.setServoPos(0);
        }













    }



}
