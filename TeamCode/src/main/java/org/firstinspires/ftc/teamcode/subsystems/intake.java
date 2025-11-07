package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.impl.MotorEx;


public class intake implements Subsystem{
    public static final Subsystem INSTANCE = new intake();
    private MotorEx intakeMotor;


    @Override
    public void initialize(){
        intakeMotor = new MotorEx("intake");

    }
    public void periodic(){
        intakeMotor.setPower(1);
    }
}



