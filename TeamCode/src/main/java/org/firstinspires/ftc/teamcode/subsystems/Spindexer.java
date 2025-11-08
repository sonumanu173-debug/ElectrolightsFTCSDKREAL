package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

import org.firstinspires.ftc.teamcode.subsystems.ColorSense1;
import org.firstinspires.ftc.teamcode.subsystems.ColorSense2;


@Configurable
public class Spindexer implements Subsystem {

    public static final Spindexer INSTANCE = new Spindexer();
    private Spindexer() { }

    public static boolean shooting = false;

    public static boolean indexing = true;

    public static void shootingtrue(){
        shooting = true;
    }

    public static void shootingfalse(){
        shooting = false;
    }

    public static void indextrue(){
        indexing = true;
    }

    public static void indexfalse(){
        indexing = false;
    }

    public static double spindexvelocity;
    public static final MotorEx spindex = new MotorEx("spindexer");


    public static float configvelocity = 1400; //far zone - ~1500. near zone - ~1200-1300

    public static void velocityControlWithFeedforwardExample(KineticState currentstate, float configtps) {
        ControlSystem controller = ControlSystem.builder()
                .velPid(0.1, 0.01, 0.05) // Velocity PID with kP=0.1, kI=0.01, kD=0.05
                .basicFF(0.0067, 0.0, 0.01) // Basic feedforward with kV=0.02, kA=0.0, kS=0.01 //pid tuning
                .build();

        controller.setGoal(new KineticState(0.0, configtps, 0.0));

        double power = controller.calculate(currentstate);
        spindex.setPower(power);
    }
    public static void spin(float tps) {
        BindingManager.update();
        spindexvelocity = spindex.getVelocity();
        KineticState currentState = new KineticState(0, spindexvelocity, 0.0);
        velocityControlWithFeedforwardExample(currentState, tps);
    }

    public static final ServoEx servoPos = new ServoEx("servoPos");

    @Override
    public Command getDefaultCommand() {
        if(shooting==true){
            if(indexing==true){
                if(ColorSense1.getDetectedColor(ActiveOpMode.telemetry())== ColorSense1.detectedColor.PURPLE && ColorSense2.getDetectedColor(ActiveOpMode.telemetry())==ColorSense2.detectedColor.GREEN){
                    //raise servo
                    servoPos.setPosition(0.1);
                }
            }
            else{
                servoPos.setPosition(0.1);
            }
        }
        return null;
    }

    @Override
    public void initialize() {
        spin(17);



    }

    @Override
    public void periodic() {


    }
}
