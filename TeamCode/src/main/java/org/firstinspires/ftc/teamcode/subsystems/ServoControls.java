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
    }

    @Override
    public void onUpdate() {
        if (gamepad1.right_trigger > 0) {

            double S = 0;
           while (S < 0.05) {
               bench.setServoPos(S);

               S = S + 0.005;

           }
           bench.setServoPos(0.0);


        }














    }



}
