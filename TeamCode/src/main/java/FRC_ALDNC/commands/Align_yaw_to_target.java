package FRC_ALDNC.commands;

import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.ff_rotation;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.p_rotation;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.tolerence_rotation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

import FRC_ALDNC.SubSystem.Camera_subsystem;
import FRC_ALDNC.SubSystem.Drive_Train;
import FRC_ALDNC.SubSystem.Shooter_Subsystem;

public class Align_yaw_to_target extends CommandBase {
    /*ATTENTION EN UTILISANT CETTE COMMANDE
    * Elle est destiné a être utilisé pour s'aligner parallèlement a un apriltag
    * Cependant, il y a de forte chance pour que la position parallèle a l'april tag ne détécte plus celui ci
    * Il est donc préférable de s'assurer que le robot détèctera toujours l'april tag en étant parallèle a celuii ci avant d'utiliser cette commmande
    * */


    private final Drive_Train drive;
    private final Camera_subsystem camera;
    private final Shooter_Subsystem shooter;
    private boolean inTeleop;
    double erreurPos;
    private DoubleSupplier forward;
    FtcDashboard dash = FtcDashboard.getInstance();


    public Align_yaw_to_target(Drive_Train drive,
                         Camera_subsystem camera,
                         Shooter_Subsystem shooter,
                         boolean InTeleop) {
        this(drive, camera, shooter, InTeleop, ()-> 0);
    }
    public Align_yaw_to_target(Drive_Train drive,
                         Camera_subsystem camera,
                         Shooter_Subsystem shooter,
                         boolean InTeleop, DoubleSupplier Forward){
        inTeleop = InTeleop;
        this.drive = drive;
        this.camera = camera;
        this.shooter = shooter;
        forward = Forward;

        addRequirements(drive); // IMPORTANT
    }

    @Override
    public void execute() {

        erreurPos = camera.getActual_detection().ftcPose.yaw * ff_rotation;

        if (Math.abs(erreurPos) < tolerence_rotation) {
            erreurPos = 0;
        }

        double turn = -Math.max(-0.3, Math.min(0.3, erreurPos * p_rotation));

        drive.drive(forward.getAsDouble()/1.3, turn);
        TelemetryPacket pack = new TelemetryPacket();
        pack.put("erreur alignement : ", erreurPos);
        dash.sendTelemetryPacket(pack);
    }

    @Override
    public boolean isFinished() {
        if (inTeleop)
            return !shooter.Is_Shooting(); // s'arrête quand on ne tire plus
        return erreurPos == 0;
    }

    @Override
    public void end(boolean interrupted) {
        TelemetryPacket pack = new TelemetryPacket();
        pack.addLine("I'm done");
        dash.sendTelemetryPacket(pack);
    }

}
