package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp
public class MotifScanningTest extends NextFTCOpMode {

    int tagID;
    public MotifScanningTest() {
        addComponents(
                new SubsystemComponent(MotifScanning.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {

    }

    @Override
    public void onUpdate() {
        tagID = MotifScanning.INSTANCE.findMotif();

        telemetry.addData("Tag ID", tagID);
        telemetry.update();
    }

}
