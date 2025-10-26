package org.firstinspires.ftc.teamcode.mechanisms;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class launchertestservo {
    private Servo servoPos;

    public void init(HardwareMap hwMap) {

        servoPos = hwMap.get(Servo.class, "servoPos");

    }

    public void setServoPos(double angle) {
        servoPos.setPosition(angle);
    }
}
