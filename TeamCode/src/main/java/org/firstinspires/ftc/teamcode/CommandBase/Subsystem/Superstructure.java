package org.firstinspires.ftc.teamcode.CommandBase.Subsystem;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.Camera.Camera;
import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.CommandBase.robotContainer;

public class Superstructure extends SubsystemBase {

    private Camera camera;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private robotContainer robot;

    public enum WantedSuperState{
        STAND_BY,
        //EMERGENCY_STATE
        STOP_SHOOTER,
        STOP_INTAKE,

        //Basics states
        INTAKE,
        EJECT,
        SHOOT_NEAR,
        SHOOT_MID,
        SHOOT_FAR,
        SHOOT_AUTO,

        //Combined states
        INTAKE_AND_SHOOT_NEAR,
        INTAKE_AND_SHOOT_MID,
        INTAKE_AND_SHOOT_FAR,
        INTAKE_AND_SHOOT_AUTO,


    }

    public enum SystemSuperState{
        IDLE,

        INTAKING,
        EJECTING,

        PREPARING_SHOOT_NEAR,
        PREPARING_SHOOT_MID,
        PREPARING_SHOOT_FAR,
        PREPARING_SHOOT_AUTO,

        READY_TO_SHOOT_NEAR,
        READY_TO_SHOOT_MID,
        READY_TO_SHOOT_FAR,
        READY_TO_SHOOT_AUTO,

        INTAKING_AND_PREPARING_SHOOT_NEAR,
        INTAKING_AND_PREPARING_SHOOT_MID,
        INTAKING_AND_PREPARING_SHOOT_FAR,
        INTAKING_AND_PREPARING_SHOOT_AUTO,

        INTAKING_AND_READY_TO_SHOOT_NEAR,
        INTAKING_AND_READY_TO_SHOOT_MID,
        INTAKING_AND_READY_TO_SHOOT_FAR,
        INTAKING_AND_READY_TO_SHOOT_AUTO,

    }

    private WantedSuperState wantedState = WantedSuperState.STAND_BY;
    private SystemSuperState systemState = SystemSuperState.IDLE;

    public void setWantedState(WantedSuperState wantedState){
        this.wantedState = wantedState;
    }

    public SystemSuperState getSystemState(){
        return systemState;
    }

    public Superstructure(Camera camera,
                          IntakeSubsystem intake,
                          ShooterSubsystem shooter,
                          robotContainer robot){

        this.camera = camera;
        this.intake = intake;
        this.shooter = shooter;
        this.robot = robot;



    }

    @Override
    public void periodic(){
        RunStateMachine();

        switch (systemState){
            case IDLE:
                break;
            case INTAKING:
                intake.setWantedState(IntakeSubsystem.WanteState.INTAKE);
                break;
            case EJECTING:
                intake.setWantedState(IntakeSubsystem.WanteState.EJECT);
                break;
            case PREPARING_SHOOT_NEAR:
                shooter.setWantedState(ShooterSubsystem.WantedState.SHOOT_NEAR);
                break;
            case PREPARING_SHOOT_MID:
                shooter.setWantedState(ShooterSubsystem.WantedState.SHOOT_MID);
                break;
            case PREPARING_SHOOT_FAR:
                shooter.setWantedState(ShooterSubsystem.WantedState.SHOOT_FAR);
                break;
            case PREPARING_SHOOT_AUTO:
                shooter.setWantedState(ShooterSubsystem.WantedState.SHOOT_AUTO);
                break;
            case READY_TO_SHOOT_NEAR:
            case READY_TO_SHOOT_MID:
            case READY_TO_SHOOT_FAR:
            case READY_TO_SHOOT_AUTO:
                break;
            case INTAKING_AND_PREPARING_SHOOT_NEAR:
            case INTAKING_AND_PREPARING_SHOOT_MID:
                break;
            case INTAKING_AND_PREPARING_SHOOT_FAR:
                break;
            case INTAKING_AND_PREPARING_SHOOT_AUTO:
                break;
            case INTAKING_AND_READY_TO_SHOOT_NEAR:
                break;
            case INTAKING_AND_READY_TO_SHOOT_MID:
                break;
            case INTAKING_AND_READY_TO_SHOOT_FAR:
                break;
            case INTAKING_AND_READY_TO_SHOOT_AUTO:
                break;
            default :
                robot.getTelemetry().addLine("Superstructure system state is not valid");
        }

    }

    private void RunStateMachine(){
        switch (wantedState)
        {

            case STAND_BY:
                break;

            case STOP_SHOOTER:
                switch (systemState)
                {
                    case PREPARING_SHOOT_NEAR:
                    case PREPARING_SHOOT_MID:
                    case PREPARING_SHOOT_FAR:
                    case PREPARING_SHOOT_AUTO:
                    case READY_TO_SHOOT_NEAR:
                    case READY_TO_SHOOT_MID:
                    case READY_TO_SHOOT_FAR:
                    case READY_TO_SHOOT_AUTO:
                        wantedState = WantedSuperState.STAND_BY;
                        break;
                    case INTAKING_AND_PREPARING_SHOOT_NEAR:
                    case INTAKING_AND_PREPARING_SHOOT_MID:
                    case INTAKING_AND_PREPARING_SHOOT_FAR:
                    case INTAKING_AND_PREPARING_SHOOT_AUTO:
                    case INTAKING_AND_READY_TO_SHOOT_NEAR:
                    case INTAKING_AND_READY_TO_SHOOT_MID:
                    case INTAKING_AND_READY_TO_SHOOT_FAR:
                    case INTAKING_AND_READY_TO_SHOOT_AUTO:
                        wantedState = WantedSuperState.INTAKE;
                        break;
                    default:
                        break;
                }
                break;

            case STOP_INTAKE:
                switch (systemState)
                {
                    case IDLE:
                        break;
                    case INTAKING:
                        wantedState = WantedSuperState.STAND_BY;
                        break;
                    case INTAKING_AND_PREPARING_SHOOT_NEAR:
                    case INTAKING_AND_READY_TO_SHOOT_NEAR:
                        wantedState = WantedSuperState.SHOOT_NEAR;
                        break;
                    case INTAKING_AND_PREPARING_SHOOT_MID:
                    case INTAKING_AND_READY_TO_SHOOT_MID:
                        wantedState = WantedSuperState.SHOOT_MID;
                        break;
                    case INTAKING_AND_PREPARING_SHOOT_FAR:
                    case INTAKING_AND_READY_TO_SHOOT_FAR:
                        wantedState = WantedSuperState.SHOOT_FAR;
                        break;
                    case INTAKING_AND_PREPARING_SHOOT_AUTO:
                    case INTAKING_AND_READY_TO_SHOOT_AUTO:
                        wantedState = WantedSuperState.SHOOT_AUTO;
                        break;
                    default:
                        break;
                }


            case INTAKE:
                switch (systemState){
                    case PREPARING_SHOOT_NEAR:
                        wantedState = WantedSuperState.INTAKE_AND_SHOOT_NEAR;
                        break;
                    case PREPARING_SHOOT_MID:
                        wantedState = WantedSuperState.INTAKE_AND_SHOOT_MID;
                        break;
                    case PREPARING_SHOOT_FAR:
                        wantedState = WantedSuperState.INTAKE_AND_SHOOT_FAR;
                        break;
                    case PREPARING_SHOOT_AUTO:
                        wantedState = WantedSuperState.INTAKE_AND_SHOOT_AUTO;
                        break;
                    default:
                        wantedState = WantedSuperState.INTAKE;
                        break;
                }
                break;

            case EJECT:
                break;

            case SHOOT_NEAR:
                break;

            case SHOOT_MID:
                break;

            case SHOOT_FAR:
                break;

            case SHOOT_AUTO:
                break;

            default :
                robot.getTelemetry().addLine("Please enter a valid superstructure wanted state");
        }
        switch (systemState){


            case IDLE:
                break;
            case INTAKING:
                break;
            case EJECTING:
                break;
            case PREPARING_SHOOT_NEAR:
                break;
            case PREPARING_SHOOT_MID:
                break;
            case PREPARING_SHOOT_FAR:
                break;
            case PREPARING_SHOOT_AUTO:
                break;
            case READY_TO_SHOOT_NEAR:
                break;
            case READY_TO_SHOOT_MID:
                break;
            case READY_TO_SHOOT_FAR:
                break;
            case READY_TO_SHOOT_AUTO:
                break;
            case INTAKING_AND_PREPARING_SHOOT_NEAR:
                break;
            case INTAKING_AND_PREPARING_SHOOT_MID:
                break;
            case INTAKING_AND_PREPARING_SHOOT_FAR:
                break;
            case INTAKING_AND_PREPARING_SHOOT_AUTO:
                break;
            case INTAKING_AND_READY_TO_SHOOT_NEAR:
                break;
            case INTAKING_AND_READY_TO_SHOOT_MID:
                break;
            case INTAKING_AND_READY_TO_SHOOT_FAR:
                break;
            case INTAKING_AND_READY_TO_SHOOT_AUTO:
                break;
            default :
                robot.getTelemetry().addLine("Superstructure system state is not valid");
        }
    }
}
