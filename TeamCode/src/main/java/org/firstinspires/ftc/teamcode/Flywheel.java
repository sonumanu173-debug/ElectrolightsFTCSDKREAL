package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class  Flywheel implements Subsystem {
    public static final Flywheel INSTANCE = new Flywheel();
    private Flywheel() { }

    public static double kP=0.01;
    public static double kI=0;
    public static double kD=0;

    public static double kV=0.01;
    public static double kA=0.02;
    public static double kS=0.03;

    public static double RPM=5000;

    private final MotorEx motor = new MotorEx("launchingmotor");

    private final ControlSystem controller = ControlSystem.builder()
            .velPid(0.01, 0, 0)
            .basicFF(0.01, 0.02, 0.03)
            .build();

    public final Command enable = new RunToVelocity(controller, RPM).requires(this).named("FlywheelOff");
    public final Command disable = new RunToVelocity(controller, 0.0).requires(this).named("FlywheelOn");

    @Override
    public void periodic() {
        motor.setPower(controller.calculate(motor.getState()));
    }
}
