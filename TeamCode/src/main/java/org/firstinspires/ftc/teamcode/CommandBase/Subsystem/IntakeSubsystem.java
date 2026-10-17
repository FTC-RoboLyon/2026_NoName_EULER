package org.firstinspires.ftc.teamcode.CommandBase.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.CommandBase.robotContainer;
import org.firstinspires.ftc.teamcode.Lib.LyonLib.control.ControlMode;
import org.firstinspires.ftc.teamcode.Lib.utils;

import java.util.function.Supplier;

public class IntakeSubsystem extends SubsystemBase {
    private DcMotor intakeMotor;
    private robotContainer robot;
    private Supplier<Float> intakeSupplier = ()-> 0f;
    private Supplier<Float> ejectSupplier = ()-> 0f;

    public final static double INTAKE_VOLTAGE_SETPOINT = 10; //TUNEME

    public enum WanteState {
        STAND_BY,
        INTAKE,
        EJECT
    }

    public enum SystemState {
        IDLE,
        INTAKING,
        EJECTING
    }
    private WanteState intakeWanteState = WanteState.STAND_BY;
    private SystemState intakeSystemState = SystemState.IDLE;
    private ControlMode intakeControlMode = ControlMode.DISABLED;

    public void setControlMode(ControlMode controlMode){intakeControlMode = controlMode;}
    public void setWantedState(WanteState intakeWanteState) {this.intakeWanteState = intakeWanteState;}

    public ControlMode getIntakeControlMode() {
        return intakeControlMode;
    }
    public SystemState getIntakeSystemState() {
        return intakeSystemState;
    }

    public IntakeSubsystem(HardwareMap hmap, robotContainer robot, String intakeName){

        this.robot = robot;

        intakeMotor = hmap.get(DcMotor.class, intakeName);

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void setSuppliers(Supplier<Float> intakeSupplier, Supplier<Float> ejectSupplier){
        this.intakeSupplier = intakeSupplier;
        this.ejectSupplier = ejectSupplier;
    }

    @Override
    public void periodic(){

        switch (intakeControlMode)
        {
            case DISABLED:
                break;
            case MANUAL_DUTY_CYCLE:
                intakeMotor.setPower(
                        utils.getVoltageCompensated(
                                intakeSupplier.get() - ejectSupplier.get(),
                                robot.getVoltage(),
                                INTAKE_VOLTAGE_SETPOINT
                        )
                );
                break;

            case DUTY_CYCLE:

                RunStateMachine();

                switch (intakeSystemState) {
                case IDLE:
                    intakeMotor.setPower(0.0);
                    break;
                case INTAKING:
                    intakeMotor.setPower(utils.getVoltageCompensated(1.0, robot.getVoltage(), INTAKE_VOLTAGE_SETPOINT));
                    break;
                case EJECTING:
                    intakeMotor.setPower(utils.getVoltageCompensated(-1.0, robot.getVoltage(), INTAKE_VOLTAGE_SETPOINT));
                    break;
                default:
                    robot.getTelemetry().addLine("Intake system state isn't valid");
                }
                break;
            default:
                robot.getTelemetry().addLine("Please enter a valid Intake Control Mode");
        }
    }
    private void RunStateMachine(){
        switch (intakeWanteState){
            case STAND_BY:
                intakeSystemState = SystemState.IDLE;
                break;
            case INTAKE:
                intakeSystemState = SystemState.INTAKING;
                break;
            case EJECT:
                intakeSystemState = SystemState.EJECTING;
                break;
            default:
                robot.getTelemetry().addLine("Please enter a valid Intake Wanted state");
        }

        switch (intakeSystemState){
            case IDLE:
            case INTAKING:
            case EJECTING:
                break;
            default:
                robot.getTelemetry().addLine("Intake system state isn't valid");
        }
    }
}
