package FRC_ALDNC.commands;

import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.ff_rotation;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.p_rotation;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.tolerence_rotation;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

import FRC_ALDNC.ALDNC_container;
import FRC_ALDNC.SubSystem.Camera_subsystem;
import FRC_ALDNC.SubSystem.Drive_Train;
import FRC_ALDNC.SubSystem.Shooter_Subsystem;
import FRC_ALDNC.CONSTAAANT_CESTMOILEBON;
import FRC_ALDNC.SubSystem.joystick_subsystem;

public class AlignToTarget extends CommandBase {
    double targetAngle;

    private final Drive_Train drive;
    private final Camera_subsystem camera;
    private final Shooter_Subsystem shooter;
    private boolean inTeleop;
    double erreurPos = 2;
    private DoubleSupplier forward;
    private DoubleSupplier turn;
    //FtcDashboard dash = FtcDashboard.getInstance();


    public AlignToTarget(Drive_Train drive,
                         Camera_subsystem camera,
                         Shooter_Subsystem shooter,
                         boolean InTeleop) {
        this(drive, camera, shooter, InTeleop, ()-> 0, ()->0);
    }
    public AlignToTarget(Drive_Train drive,
                         Camera_subsystem camera,
                         Shooter_Subsystem shooter,
                         boolean InTeleop, DoubleSupplier Forward,
                         DoubleSupplier tUrn){
        turn = tUrn;
        inTeleop = InTeleop;
        this.drive = drive;
        this.camera = camera;
        this.shooter = shooter;
        forward = Forward;

        addRequirements(drive); // IMPORTANT
    }

    @Override
    public void execute() {

        targetAngle = camera.getBearing() == 100000? targetAngle : drive.getHeadingRadians()+camera.getBearing();
        erreurPos = targetAngle - drive.getHeadingRadians();
        if (erreurPos > 180) erreurPos -= 2*Math.PI;
        if (erreurPos < -180) erreurPos += 2*Math.PI;

        if (Math.abs(erreurPos) < tolerence_rotation) {
            erreurPos = 2;
        }

        double turn = -Math.max(-0.3, Math.min(0.3, erreurPos * ff_rotation + erreurPos * p_rotation));

        drive.drive(forward.getAsDouble()/1.3, turn + this.turn.getAsDouble()/1.3);
        //TelemetryPacket pack = new TelemetryPacket();
        //pack.put("erreur alignement : ", erreurPos);
        //dash.sendTelemetryPacket(pack);<xpklkjo+

        //caa
    }

    @Override
    public boolean isFinished() {
        if (inTeleop)
            return !shooter.Is_Shooting(); // s'arrête quand on ne tire plus
        return erreurPos == 0;
    }

    @Override
    public void end(boolean interrupted) {
        //TelemetryPacket pack = new TelemetryPacket();
        //pack.addLine("I'm done");
        //dash.sendTelemetryPacket(pack);
    }
}
