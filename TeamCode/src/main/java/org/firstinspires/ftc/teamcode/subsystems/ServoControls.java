package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.launchertestservo;

@TeleOp(name = "Servo Test", group = "TeleOp")
public class ServoControls extends OpMode{
    launchertestservo bench = new launchertestservo();
    @Override
    public void init() {
        bench.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            bench.setServoPos(-1.0);
        }

        else if (gamepad1.b) {
            bench.setServoPos(1.0);
         }
        else {
            bench.setServoPos(0.0);
        }











    }



}
