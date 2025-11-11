package org.firstinspires.ftc.teamcode.subsystems;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.impl.MotorEx;


public class Intake implements Subsystem{
    public static final Subsystem INSTANCE = new Intake();

    private Intake() { }
    private MotorEx intakeMotor;


    @Override
    public void initialize(){
        intakeMotor = new MotorEx("intake");

    }
    public void periodic(){

        //follower.update();
    }
}



