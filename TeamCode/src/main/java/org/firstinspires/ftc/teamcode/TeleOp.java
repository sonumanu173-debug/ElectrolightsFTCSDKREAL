package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.subsystems.ColorSense1;
import org.firstinspires.ftc.teamcode.subsystems.ColorSense2;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.MotifScanning;

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
                new SubsystemComponent(Flywheel.INSTANCE, DriveTrain.INSTANCE, MotifScanning.INSTANCE/*, Intake.INSTANCE, Spindexer.INSTANCE*/, ColorSense1.INSTANCE, ColorSense2.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE

        );
    }

    public static int tagID;
    public static boolean findMotif = false;
    public static int ball1Color = 0; //green = 1, purple = 2
    public static int ball2Color = 0;
    public static int ball3Color = 0;

    public static int getBall1Color() {
        return ball1Color;
    }

    public static int getBall2Color() {
        return ball2Color;
    }
    public static int getBall3Color() {
        return ball3Color;
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
        Flywheel.shooter(0);

    }

    @Override
    public void onUpdate() {
        if (findMotif) {
            tagID = MotifScanning.INSTANCE.findMotif();
            if (tagID == 21) {
                ball1Color = 1; //green
                ball2Color = 2; //purple
                ball3Color = 2;
            } else if (tagID == 22) {
                ball1Color = 2;
                ball2Color = 1;
                ball3Color = 2;
            } else if (tagID == 23) {
                ball1Color = 2;
                ball2Color = 2;
                ball3Color = 1;
            }
            findMotif = false;
        }
    }

    @Override
    public void onStartButtonPressed() {
        Gamepads.gamepad1().rightTrigger().greaterThan(0.2).whenBecomesTrue(() -> DriveTrain.shootingtrue())
                .whenBecomesFalse(() -> DriveTrain.shootingfalse());
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> intakeMotor.setPower(1))
                .whenFalse(() -> intakeMotor.setPower(0));
        Gamepads.gamepad2().triangle().whenBecomesTrue(() -> findMotif = true);
    }
}