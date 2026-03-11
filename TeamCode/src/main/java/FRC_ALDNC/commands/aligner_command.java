package FRC_ALDNC.commands;
import com.arcrobotics.ftclib.command.CommandBase;

import FRC_ALDNC.SubSystem.Drive_Train;

public class aligner_command extends CommandBase {
    Drive_Train jambeDroite;
    Drive_Train jambeGauche;
    public aligner_command(Drive_Train jambeDroite, Drive_Train jambegauche){
        this.jambeDroite = jambeDroite;
        this.jambeGauche = jambegauche;
        addRequirements(jambeDroite, jambegauche);
    }
    @Override
    public void initialize() {}

    @Override
    public boolean isFinished() {
        return  true;
    }
}
