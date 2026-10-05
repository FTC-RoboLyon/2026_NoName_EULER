package org.firstinspires.ftc.teamcode.CommandBase.Command;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.ShooterSubsystem;

import java.util.function.BooleanSupplier;

public class ShootManually extends CommandBase {

    ShooterSubsystem shooterSubsystem;
    BooleanSupplier buttonVelo;
    BooleanSupplier buttonHoodPos;

    int veloTarget;
    double hoodPos;

    int veloIncrementation;
    double hoodPosIncrmentation;
    public ShootManually (ShooterSubsystem shooter,
                          BooleanSupplier buttonVelo, BooleanSupplier buttonHoodPos,
                          int veloTarget, double hoodPos,
                          int veloIncrementation, double hoodPosIncrmentation){
        shooterSubsystem = shooter;
        this.buttonVelo = buttonVelo;
        this.buttonHoodPos = buttonHoodPos;

        this.veloTarget = veloTarget;
        this.hoodPos = hoodPos;

        this.veloIncrementation = veloIncrementation;
        this.hoodPosIncrmentation = hoodPosIncrmentation;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        shooterSubsystem.setWantedState(ShooterSubsystem.WantedState.MANUAL);
    }

    @Override
    public void execute(){
        if (buttonVelo.getAsBoolean())
            veloTarget += veloIncrementation;

        if (buttonHoodPos.getAsBoolean())
            hoodPos += hoodPosIncrmentation;
        shooterSubsystem.setTargets(veloTarget, hoodPos);
        //Et pourquoi le shooter irait pas chercher cette info tout seul prc que la dcp il est required par une command qui peut en plus le mettre en Stand_By a la fin
    }

    @Override
    public void end(boolean interrupted){
        if (interrupted)
            shooterSubsystem.setWantedState(ShooterSubsystem.WantedState.STAND_BY);
        //est tu sur de bien comprendre a quoi correspondent la fonction end et son parametre interrupted
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
