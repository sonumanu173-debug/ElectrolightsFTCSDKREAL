package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


public class TestBench {
    private IMU imu;
    public TestBench(){

    }
    public void init(HardwareMap hardwareMap){
        imu = hardwareMap.get(IMU.class, "imu");

    }
    public YawPitchRollAngles getOrientation(){
        if(imu != null){
            return imu.getRobotYawPitchRollAngles();
        } else {
            return new YawPitchRollAngles();
        }
    }
}
