package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.driving.HolonomicMode;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;


@TeleOp(name = "NextFTC TeleOp Program Java")
public class MechanumFieldOrientatedOpMode extends NextFTCOpMode {

    public MechanumFieldOrientatedOpMode() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private final MotorEx frontLeftMotor = new MotorEx("front_left").reversed();
    private final MotorEx frontRightMotor = new MotorEx("front_right");
    private final MotorEx backLeftMotor = new MotorEx("back_left").reversed();
    private final MotorEx backRightMotor = new MotorEx("back_right");

    private IMUEx imu;
    private boolean fieldCentric = true;
    private boolean yPressedLast = false;
    private Command driverControlled;

    @Override
    public void onInit() {
        imu = new IMUEx("imu", Direction.UP, Direction.FORWARD).zeroed();
        // i'll change ts based on the config of the bobot lmao
    }

    @Override
    public void onStartButtonPressed() {
        startDriveCommand(fieldCentric);
    }

    private void startDriveCommand(boolean useFieldCentric) {
        if (driverControlled != null) driverControlled.cancel();

        if (useFieldCentric) {
            driverControlled = new MecanumDriverControlled(
                    frontLeftMotor,
                    frontRightMotor,
                    backLeftMotor,
                    backRightMotor,
                    Gamepads.gamepad1().leftStickY().negate(),
                    Gamepads.gamepad1().leftStickX(),
                    Gamepads.gamepad1().rightStickX(),
                    new FieldCentric(imu)
            );
            driverControlled.schedule();
        } else {
            driverControlled = new MecanumDriverControlled(
                    frontLeftMotor,
                    frontRightMotor,
                    backLeftMotor,
                    backRightMotor,
                    Gamepads.gamepad1().leftStickY().negate(),
                    Gamepads.gamepad1().leftStickX(),
                    Gamepads.gamepad1().rightStickX()
            );
        }

        driverControlled.schedule();
    }

    @Override
    public void onUpdate() {
        boolean yPressed = Gamepads.gamepad1().y().get();

        if (yPressed && !yPressedLast) {
            fieldCentric = !fieldCentric;
            startDriveCommand(fieldCentric);
        }

        yPressedLast = yPressed;

        telemetry.addData("Drive Mode", fieldCentric ? "Field Centric" : "Robot Centric");
    }
}
