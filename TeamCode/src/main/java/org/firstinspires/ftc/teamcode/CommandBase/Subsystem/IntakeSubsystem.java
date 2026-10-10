package org.firstinspires.ftc.teamcode.CommandBase.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.CommandBase.robotContainer;
import org.firstinspires.ftc.teamcode.Lib.utils;

import java.util.function.Supplier;

public class IntakeSubsystem extends SubsystemBase {
    private DcMotor intakeMotor;
    private robotContainer robot;
    private Supplier<Float> intakeSupplier;
    private Supplier<Float> ejectSupplier;

    public final static double INTAKE_VOLTAGE_SETPOINT = 10; //TUNEME



    public enum IntakeWanteState { //Souviens toi il n'y a que la base qui a un enum mode les autres c WantedState et SystemState
        IDLE,
        INTAKE,
        EJECT
    }
    public enum IntakeControlMode {
        DISABLE,
        MANUAL_VOLTAGE,
        AUTO_ON_OFF
    }
    public enum IntakeSystemState {
        STAND_BY,
        INTAKING,
        EJECTING
    }
    private IntakeWanteState intakeWanteState = IntakeWanteState.IDLE;
    private IntakeSystemState intakeSystemState = IntakeSystemState.STAND_BY;
    private IntakeControlMode intakeControlMode = IntakeControlMode.DISABLE;

    public void setIntakeControlMode(IntakeControlMode controlMode){intakeControlMode = controlMode;}
    public void setIntakeWanteState(IntakeWanteState intakeWanteState) {this.intakeWanteState = intakeWanteState;}

    public IntakeControlMode getIntakeControlMode() {
        return intakeControlMode;
    }
    public IntakeSystemState getIntakeSystemState() {
        return intakeSystemState;
    }

    public IntakeSubsystem(HardwareMap hmap, robotContainer robot){

        this.robot = robot;

        intakeMotor = hmap.get(DcMotor.class, "Intake");

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (intakeSupplier == null)
            intakeSupplier = ()-> 0f;

        if (ejectSupplier == null)
            ejectSupplier = ()-> 0f;

    }
    public void setSuppliers(Supplier<Float> intakeSupplier, Supplier<Float> ejectSupplier){
        this.intakeSupplier = intakeSupplier;
        this.ejectSupplier = ejectSupplier;
    }

    @Override
    public void periodic(){

        switch (intakeControlMode) {
            case DISABLE:
                break;
            case MANUAL_VOLTAGE:
                intakeMotor.setPower(
                        utils.getVoltageCompensated(
                                intakeSupplier.get() - ejectSupplier.get(),
                                robot.getVoltage(),
                                INTAKE_VOLTAGE_SETPOINT
                        )
                );
                break;

            case AUTO_ON_OFF:

                RunStateMachine();

                switch (intakeSystemState) {
                case STAND_BY:
                    intakeMotor.setPower(0.0);
                    break;
                case INTAKING:
                    intakeMotor.setPower(utils.getVoltageCompensated(1.0, robot.getVoltage(), INTAKE_VOLTAGE_SETPOINT));
                    break;
                case EJECTING:
                    intakeMotor.setPower(utils.getVoltageCompensated(-1.0, robot.getVoltage(), INTAKE_VOLTAGE_SETPOINT));
                    break;
            }
            break;
        }
    }
    private void RunStateMachine(){
        switch (intakeWanteState){
            case IDLE:
                intakeSystemState = IntakeSystemState.STAND_BY;
                break;
            case INTAKE:
                intakeSystemState = IntakeSystemState.INTAKING;
                break;
            case EJECT:
                intakeSystemState = IntakeSystemState.EJECTING;
                break;
        }

        switch (intakeSystemState){
            case STAND_BY:
            case INTAKING:
            case EJECTING:
                break;
        }
    }
}
