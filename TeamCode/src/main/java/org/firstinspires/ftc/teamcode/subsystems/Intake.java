package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;


public class Intake implements Subsystem{
    public static final Subsystem INSTANCE = new Intake();

    private Intake() { }
    private MotorEx intakeMotor;
    public Command intakeOff = new SetPower(intakeMotor, 0).requires(this);
    public Command intakeOn = new SetPower(intakeMotor, 1).requires(this);

    @Override
    public void initialize(){
        intakeMotor = new MotorEx("intake");

    }
    public void periodic(){

    }
}



