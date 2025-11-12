package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;

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


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Test TeleOp")
public class TeleOpTest extends NextFTCOpMode {

    public MotorEx intakeMotor;
    public TeleOpTest() {
        addComponents(
                new SubsystemComponent(Flywheel.INSTANCE/*, Intake.INSTANCE, Spindexer.INSTANCE*/, ColorSense1.INSTANCE, ColorSense2.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE

        );
    }

    private static final int APRILTAG_PIPELINE = 8;
    @Override
    public void onInit() {
        shooter(1000);

    }

    @Override
    public void onUpdate() {
    }

    @Override
    public void onStartButtonPressed() {

    }
}