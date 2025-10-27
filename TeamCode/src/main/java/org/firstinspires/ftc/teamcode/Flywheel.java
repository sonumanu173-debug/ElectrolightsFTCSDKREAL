package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class  Flywheel implements Subsystem {
    public static final Flywheel INSTANCE = new Flywheel();
    private Flywheel() { }

    public PIDCoefficients PIDCoeff;
    public BasicFeedforwardParameters FF;
    private final MotorEx motor = new MotorEx("launchingmotor");

    private final ControlSystem controller = ControlSystem.builder()
            .velPid(0.01, 0, 0)
            .basicFF(0.01, 0.01, 0.01)
            .build();

    public final Command enable = new RunToVelocity(controller, ShooterCalculations.requiredTPS).requires(this).named("FlywheelOn");
    public final Command disable = new RunToVelocity(controller, 0.0).requires(this).named("FlywheelOff");
    @Override
    public void periodic() {
        motor.setPower(controller.calculate(motor.getState()));

    }
}
