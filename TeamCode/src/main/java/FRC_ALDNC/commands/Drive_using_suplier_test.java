package FRC_ALDNC.commands;

import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.p_rotation;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.ff_rotation;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.seuilDriveShooter;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.tolerence_rotation;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import FRC_ALDNC.SubSystem.Camera_subsystem;
import FRC_ALDNC.SubSystem.Drive_Train;

public class Drive_using_suplier_test extends CommandBase {
    private final Drive_Train chassis;
    private final Camera_subsystem cam;
    private final DoubleSupplier forward;
    private final DoubleSupplier turn;
    private final BooleanSupplier is_shooting;
    private double targetPosition;
    private double erreurPos;
    private double powerTurn;
    public Drive_using_suplier_test (Drive_Train chassiS,Camera_subsystem camera, DoubleSupplier forward, DoubleSupplier turn, BooleanSupplier is_shooting){
        chassis = chassiS;
        cam = camera;
        this.forward = forward;
        this.turn = turn;
        this.is_shooting = is_shooting;
        addRequirements(chassis);
    }

    @Override
    public void execute() {
        cam.updtade();

        targetPosition = cam.getBearing()*ff_rotation;
        targetPosition = targetPosition + chassis.Get_right_current();
        erreurPos = targetPosition-chassis.Get_right_current();
        if(erreurPos>-tolerence_rotation && erreurPos<tolerence_rotation){
            erreurPos = 0;
        }
        powerTurn = erreurPos*p_rotation;
        powerTurn = Math.max(-0.3, Math.min(0.3, powerTurn));
        if(is_shooting.getAsBoolean()){
            chassis.drive(0, powerTurn);
            if(chassis.Get_right_power() > seuilDriveShooter || chassis.Get_right_power() < -seuilDriveShooter || chassis.Get_left_power() > seuilDriveShooter || chassis.Get_left_power() < -seuilDriveShooter){
                chassis.drive(forward.getAsDouble()/1.3, turn.getAsDouble()/1.3);
            }
        }else{
            chassis.drive(forward.getAsDouble(), turn.getAsDouble());
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
