package org.firstinspires.ftc.teamcode;

import static dev.nextftc.bindings.Bindings.button;

import android.health.connect.datatypes.units.Power;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


import dev.nextftc.bindings.*;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.ControlSystem;

import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@TeleOp(name = "Flywheel PID")
public class flywheelpid extends NextFTCOpMode {
    public flywheelpid() {
        addComponents(

        );




    }


    public static double flywheelvelocity;


    public static MotorEx flywheel = new MotorEx("launchingmotor");

    public static float configvelocity = (float) ShooterCalculations.requiredTPS; //far zone - ~1500. near zone - ~1200-1300


    public static void velocityControlWithFeedforwardExample(KineticState currentstate) {
        // Create a velocity controller with PID and feedforward
        ControlSystem controller = ControlSystem.builder()
                .velPid(0.18, 0.01, 0.05) // Velocity PID with kP=0.1, kI=0.01, kD=0.05
                .basicFF(0.0067, 0.0, 0.01) // Basic feedforward with kV=0.02, kA=0.0, kS=0.01 //pid tuning
                .build();

        // Set the goal velocity to 500 units per second
        controller.setGoal(new KineticState(0.0, configvelocity, 0.0));

        // In a loop (simulated here), you would:
        // Create a KineticState with current position and velocity
        // In a loop (simulated here), you would:
        // Create a KineticState with current position and velocity

        double power = controller.calculate(currentstate);
        flywheel.setPower(power);

        // Apply power to your motor
        System.out.println("Power to apply: " + power);
    }

    @Override public void onInit() { }
    @Override public void onWaitForStart() { }
    @Override public void onStartButtonPressed() {

    }

    public static void shooter() {
        BindingManager.update();
        flywheelvelocity = flywheel.getVelocity();
        KineticState currentState = new KineticState(0, flywheelvelocity, 0.0); //figure out velocity (is it in ticks?!?)
        velocityControlWithFeedforwardExample(currentState);


//motor.setPower(kP * error + kV * targetVelocity)
//where error is (targetVelocity - motor.getVelocity())
//tune kP until error is small enough (graph error)


    }
    @Override public void onUpdate() {
        shooter();
        double ticksPerSecond = flywheel.getVelocity();

        double rpm = (ticksPerSecond / 28) * -60.0;
        double goal = ShooterCalculations.requiredRPM;
        PanelsTelemetry.INSTANCE.getTelemetry().addData("Motor RPM", rpm);
        PanelsTelemetry.INSTANCE.getTelemetry().addData("Required RPM", goal);
        PanelsTelemetry.INSTANCE.getTelemetry().update(telemetry);
    }

}