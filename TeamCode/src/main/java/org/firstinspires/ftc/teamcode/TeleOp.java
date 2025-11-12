package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.subsystems.ColorSense1;
import org.firstinspires.ftc.teamcode.subsystems.ColorSense2;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Main TeleOp")
public class TeleOp extends NextFTCOpMode {

    public MotorEx intakeMotor;
    public TeleOp() {
        addComponents(
                new SubsystemComponent(Flywheel.INSTANCE, DriveTrain.INSTANCE/*, Intake.INSTANCE, Spindexer.INSTANCE*/, ColorSense1.INSTANCE, ColorSense2.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE

        );
    }

    private static final int APRILTAG_PIPELINE = 8;
    @Override
    public void onInit() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();
        intakeMotor = new MotorEx("intake").reversed();
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> intakeMotor.setPower(1))
                .whenFalse(() -> intakeMotor.setPower(0));

    }

    @Override
    public void onUpdate() {
    }

    @Override
    public void onStartButtonPressed() {
        Gamepads.gamepad1().rightTrigger().greaterThan(0.2).whenTrue(() -> DriveTrain.shootingtrue())
                .whenFalse(() -> DriveTrain.shootingfalse());
        Gamepads.gamepad1().leftTrigger().greaterThan(0.2).whenTrue(() -> DriveTrain.indextrue())
                .whenFalse(() -> DriveTrain.indexfalse());
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> intakeMotor.setPower(1))
                .whenFalse(() -> intakeMotor.setPower(0));
    }
}