package org.firstinspires.ftc.teamcode.CommandBase;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.Robot;

public class robotContainer extends Robot {
    private HardwareMap hardwareMap;
    private static VoltageSensor voltageSensor;
    public robotContainer (HardwareMap hmap){
        hardwareMap = hmap;

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        setBulkReading(hardwareMap, LynxModule.BulkCachingMode.AUTO);
    }
    public static double getVoltage(){return voltageSensor.getVoltage();}
}
