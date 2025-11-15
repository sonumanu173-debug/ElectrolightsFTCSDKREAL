package org.firstinspires.ftc.teamcode.subsystems;
import java.util.concurrent.TimeUnit;


import androidx.annotation.NonNull;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.bindings.*;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import org.firstinspires.ftc.teamcode.subsystems.ColorSense1;
import org.firstinspires.ftc.teamcode.subsystems.ColorSense2;
import org.firstinspires.ftc.teamcode.subsystems.MotifScanning;


public class realAutoSubsystemCommand implements Subsystem {
    static ColorSense1 bench = new ColorSense1();
    static ColorSense2 bench2 = new ColorSense2();

    static MotorEx spindex = new MotorEx("spindexer");
    public static final realAutoSubsystemCommand INSTANCE = new realAutoSubsystemCommand();

    @Override

    public void initialize(){
        bench.init(ActiveOpMode.hardwareMap());
        bench2.init(ActiveOpMode.hardwareMap());



    }

    public Command stopSpinDexer() {
        return new LambdaCommand()
                .setStart(() -> {
                    bench.init(ActiveOpMode.hardwareMap());
                    bench2.init(ActiveOpMode.hardwareMap());
                })
                .setUpdate(() -> {
                    // Runs on update
                    ColorSense1.detectedColor yes = bench.getDetectedColor(ActiveOpMode.telemetry());
                    ColorSense2.detectedColor ye = bench2.getDetectedColor(ActiveOpMode.telemetry());
                    if (yes != ColorSense1.detectedColor.ERROR && ye != ColorSense2.detectedColor.ERROR) {
                        spindex.setPower(0);
                        ActiveOpMode.telemetry().addLine("spinstopped for 2");

                        new Delay(1);
                        spindex.setPower(0.1);
                    }
                })
                .setStop(interrupted -> { /* Runs on stop */ })
                .setIsDone(() -> true)
                .requires(this)
                .setInterruptible(true)
                .named("Stop Spindexer");
    }



    @Override
    public void periodic(){

    }
}
