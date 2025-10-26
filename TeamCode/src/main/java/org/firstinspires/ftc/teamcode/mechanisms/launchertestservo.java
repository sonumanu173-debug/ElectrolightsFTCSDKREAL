package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;




public class launchertestservo {

    private Servo servoPos;

    public void init(HardwareMap hwMap) {
        servoPos = hwMap.get(Servo.class,"servo_positional");



    }

    public void setPos(double angle){
        servoPos.setPosition(angle);

            











    }

}


