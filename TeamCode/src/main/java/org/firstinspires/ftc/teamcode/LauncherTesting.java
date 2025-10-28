package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.flywheelpid.shooter;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.components.BulkReadComponent;


@TeleOp(name = "Launcher Testing")
public class LauncherTesting extends NextFTCOpMode {
    public LauncherTesting() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    public DcMotorEx motor;

    private static final double TICKS_PER_REV = 28;

    @Override
    public void onInit() {
        motor = hardwareMap.get(DcMotorEx.class, "launchingmotor");
    }

    @Override
    public void onUpdate() {
        double ticksPerSecond = motor.getVelocity();
        shooter(1500);
        double rpm = (ticksPerSecond / TICKS_PER_REV) * -60.0;
        double goal = ShooterCalculations.requiredRPM;
        PanelsTelemetry.INSTANCE.getTelemetry().addData("Motor RPM", rpm);
        PanelsTelemetry.INSTANCE.getTelemetry().addData("Required RPM", goal);
        PanelsTelemetry.INSTANCE.getTelemetry().update(telemetry);

    }

    @Override
    public void onStartButtonPressed() {


    }
}z