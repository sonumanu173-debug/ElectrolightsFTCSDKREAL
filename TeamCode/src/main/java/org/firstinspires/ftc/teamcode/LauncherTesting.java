package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.Flywheel;

import dev.nextftc.core.commands.Command;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.components.BulkReadComponent;


@TeleOp(name = "Launcher Testing")
public class LauncherTesting extends NextFTCOpMode {
    public LauncherTesting() {
        addComponents(
                new SubsystemComponent(Flywheel.INSTANCE),
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

        double rpm = (ticksPerSecond / TICKS_PER_REV) * -60.0;
        double goal = Flywheel.RPM;
        telemetry.addData("Motor RPM", rpm);
        telemetry.update();
        PanelsTelemetry.INSTANCE.getTelemetry().addData("Motor RPM", rpm);
        PanelsTelemetry.INSTANCE.getTelemetry().addData("Required RPM", goal);
        PanelsTelemetry.INSTANCE.getTelemetry().update(telemetry);

    }

    @Override
    public void onStartButtonPressed() {

        Flywheel.INSTANCE.enable.schedule();


    }
}