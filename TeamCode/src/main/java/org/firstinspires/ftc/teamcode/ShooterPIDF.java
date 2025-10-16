package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Configurable
public class ShooterPIDF extends LinearOpMode {

    DcMotorEx motor;

    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "motor"); //motor has to be name in control hub
        waitForStart();
        while (opModeIsActive()) {

        }
    }

    public double PIDControl(double reference, double state) {

    }

}
