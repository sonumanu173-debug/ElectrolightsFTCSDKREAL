package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.impl.MotorEx;


public class Intake implements Subsystem{
    public static final Subsystem INSTANCE = new Intake();

    private Intake() { }
    private MotorEx intakeMotor;

    public Command getDefaultCommand() {
        Gamepads.gamepad1().triangle().whenBecomesTrue(() -> intakeMotor.setPower(1))
                .whenFalse(() -> intakeMotor.setPower(0));
        return null;
    }
    @Override
    public void initialize(){
        intakeMotor = new MotorEx("intake");

    }
    public void periodic(){

    }
}



