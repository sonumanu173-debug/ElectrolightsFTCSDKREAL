package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.launchertestservo;


public class ServoControls extends OpMode{
    launchertestservo bench = new launchertestservo();
    @Override
    public void init() {
        bench.init(hardwareMap);
    }



    @Override
    public void loop() {
        bench.setServoPos(-1.0);


    }



}
