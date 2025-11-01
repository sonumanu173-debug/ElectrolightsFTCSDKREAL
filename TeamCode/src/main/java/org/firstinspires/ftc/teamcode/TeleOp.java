package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Calculations;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.components.BulkReadComponent;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends NextFTCOpMode {
    public TeleOp() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Flywheel.INSTANCE, Drivetrain.INSTANCE)
        );
    }

    /*public DcMotorEx motor;

    private static final double TICKS_PER_REV = 28;*/
    private static final int APRILTAG_PIPELINE = 8;
    @Override
    public void onInit() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();
        //motor = hardwareMap.get(DcMotorEx.class, "launchingmotor");
    }

    @Override
    public void onUpdate() {
        /*double ticksPerSecond = motor.getVelocity();
        shooter(1500);
        double rpm = (ticksPerSecond / TICKS_PER_REV) * -60.0;
        double goal = Calculations.requiredRPM;
        PanelsTelemetry.INSTANCE.getTelemetry().addData("Motor RPM", rpm);
        PanelsTelemetry.INSTANCE.getTelemetry().addData("Required RPM", goal);
        PanelsTelemetry.INSTANCE.getTelemetry().update(telemetry);*/

    }

    @Override
    public void onStartButtonPressed() {


    }
}