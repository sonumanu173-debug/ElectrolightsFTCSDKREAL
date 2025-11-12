package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import dev.nextftc.bindings.*;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.ControlSystem;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

public class Flywheel implements Subsystem {
    public Flywheel() {

    }

    public static final Flywheel INSTANCE = new Flywheel();
    public static double flywheelvelocity;

    public static MotorEx flywheel = new MotorEx("launchingmotor");

    public static float configvelocity = 1400; //far zone - ~1500. near zone - ~1200-1300

    public static void velocityControlWithFeedforwardExample(KineticState currentstate, float configtps) {
        // Create a velocity controller with PID and feedforward
        ControlSystem controller = ControlSystem.builder()
                .velPid(0.18, 0.01, 0.05) // Velocity PID with kP=0.1, kI=0.01, kD=0.05
                .basicFF(0.0067, 0.0, 0.01) // Basic feedforward with kV=0.02, kA=0.0, kS=0.01 //pid tuning
                .build();

        controller.setGoal(new KineticState(0.0, configtps, 0.0));

        // In a loop (simulated here), you would:
        // Create a KineticState with current position and velocity

        double power = controller.calculate(currentstate);
        flywheel.setPower(power);
    }
    public static void shooter(float tps) {
        BindingManager.update();
        flywheelvelocity = flywheel.getVelocity();
        KineticState currentState = new KineticState(0, flywheelvelocity, 0.0);
        //if(tps-(-1*flywheelvelocity)<7 && tps-(-1*flywheelvelocity)>-7){
            velocityControlWithFeedforwardExample(currentState, tps);
        double rpm = (flywheelvelocity / 28) * 60.0;
        ActiveOpMode.telemetry().addData("Flywheel RPM", rpm);

        //} // if this doesnt work remove if statement
        //motor.setPower(kP * error + kV * targetVelocity)
        //where error is (targetVelocity - motor.getVelocity())
        //tune kP until error is small enough (graph error)
    }
    @Override public void initialize() {
        flywheel.setPower(0.1);
    }
    /*@Override public void onUpdate() {
        configvelocity=findTPS(4.03);
        shooter(configvelocity);
        double ticksPerSecond = flywheel.getVelocity();

        double rpm = (ticksPerSecond / 28) * 60.0;
        double goal = ShooterCalculations.requiredRPM;
        double goal2 = ShooterCalculations.requiredTPS;
        PanelsTelemetry.INSTANCE.getTelemetry().addData("Motor RPM", rpm);
        PanelsTelemetry.INSTANCE.getTelemetry().addData("Required RPM", goal);
        PanelsTelemetry.INSTANCE.getTelemetry().addData("Required TPS", goal2);
        PanelsTelemetry.INSTANCE.getTelemetry().update(telemetry);*/
}

