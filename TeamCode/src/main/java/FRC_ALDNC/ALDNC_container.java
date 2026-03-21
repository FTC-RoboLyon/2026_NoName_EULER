package FRC_ALDNC;



//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.arcrobotics.ftclib.command.button.Button;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import FRC_ALDNC.SubSystem.Camera_subsystem;
import FRC_ALDNC.SubSystem.Drive_Train;
import FRC_ALDNC.SubSystem.Feeder_subsystem;
import FRC_ALDNC.SubSystem.Intake_subsystem;
import FRC_ALDNC.SubSystem.NavXSubsystem;
import FRC_ALDNC.SubSystem.Shooter_Subsystem;
import FRC_ALDNC.SubSystem.joystick_subsystem;
import FRC_ALDNC.commands.AlignToTarget;
import FRC_ALDNC.commands.Let_a_ball_pass;
import FRC_ALDNC.commands.Shoot_a_ball_command;
import FRC_ALDNC.commands.Collect_command;
import FRC_ALDNC.commands.Configure_shooter;
import FRC_ALDNC.commands.Stop_shooter;


public class ALDNC_container{
    Drive_Train chassis_subsystem;
    Shooter_Subsystem shooter_subsystem;
    Intake_subsystem intake;
    Feeder_subsystem feeder;
    joystick_subsystem left_joystick;
    joystick_subsystem right_joystick;
    Telemetry telemetry;
    Camera_subsystem apriljoke;
    DoubleSupplier forward;
    DoubleSupplier turn;
    public enum RobotMode
    {
        AUTO_BLUE,
        AUTO_RED,
        TELEOP_RED,
        TELEOP_BLUE
    }
    public enum Artefact_order
    {
        PPG,
        PGP,
        GPP
    }

    public boolean is_inTeleop;

    RobotMode team_and_mode;
    Artefact_order actual_artefact_order = Artefact_order.PPG;
    public double m_voltageSensorValue;
    VoltageSensor voltageSensor;
    public BooleanSupplier is_shooting;

    public double x,y;
    public NavXSubsystem navx;

    public DoubleSupplier diStance_to_goal;

    public ALDNC_container (HardwareMap hmap, RobotMode wich_programme, GamepadEx gamepad, Telemetry telemetry){
        team_and_mode = wich_programme;
        is_inTeleop = team_and_mode == RobotMode.TELEOP_RED || team_and_mode == RobotMode.TELEOP_BLUE;
        apriljoke = new Camera_subsystem(hmap, wich_programme == RobotMode.TELEOP_RED || wich_programme == RobotMode.AUTO_RED ? 24 : 20, telemetry);
        diStance_to_goal = new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                if (apriljoke.getActual_detection() != null)
                    return apriljoke.getActual_detection().ftcPose.y;
                return 0;
            }
        };
        BooleanSupplier is_intaking = ()-> intake.getM_systemState() == Intake_subsystem.SystemState.INTAKING;

        chassis_subsystem = new Drive_Train(hmap, telemetry, x, y, this, is_intaking);


        shooter_subsystem = new Shooter_Subsystem(hmap, telemetry, this, !is_inTeleop, diStance_to_goal, is_intaking);




        intake = new Intake_subsystem(hmap);

        feeder = new Feeder_subsystem(hmap, this);
        navx = new NavXSubsystem(hmap);


        left_joystick = new joystick_subsystem(gamepad, joystick_subsystem.Witch_stick.left);
        right_joystick = new joystick_subsystem(gamepad, joystick_subsystem.Witch_stick.right);
        forward = () -> left_joystick.getX();
        turn = () -> right_joystick.getY();







        voltageSensor = hmap.get(VoltageSensor.class, "Control Hub");

        team_and_mode = wich_programme;

        //chassis_subsystem.setDefaultCommand(new Drive_command(chassis_subsystem, left_joystick, right_joystick, telemetry));
        // shooter_subsystem.setDefaultCommand(new Tuning_postir_command(shooter_subsystem, apriljoke));
        if (is_inTeleop) {
            //chassis_subsystem.setDefaultCommand(new Drive_using_suplier_test(chassis_subsystem,apriljoke, forward, turn, is_shooting));
            chassis_subsystem.setDefaultCommand(new RunCommand(
                    () -> chassis_subsystem.drive(
                            forward.getAsDouble(),
                            turn.getAsDouble()
                    ),
                    chassis_subsystem));
            new Trigger(() -> shooter_subsystem.Is_Shooting())
                    .whileActiveContinuous(new AlignToTarget(chassis_subsystem, apriljoke, shooter_subsystem, is_inTeleop, forward, turn, LED));
        } else {
            //new Trigger(() -> shooter_subsystem.Is_Shooting())
            //        .whileActiveContinuous(new AlignToTarget(chassis_subsystem, apriljoke, shooter_subsystem, true));
        }
        this.telemetry = telemetry;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());


        LED = hmap.get(RevBlinkinLedDriver.class, "led");
        LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);
//
    }


    public final void Determinate_order(Artefact_order order){
        actual_artefact_order = order;
    }
    public Artefact_order Get_actual_artefact_order(){
        return actual_artefact_order;
    }
    public void Configure_Binding(
            Button feeder_button,
            Button shoot_bank_button,
            Button shoot_mid_button,
            Button shoot_far_butto,
            Button aspirer_button,
            Button intake_button,
            Trigger eject_button,
            Button alignageButton,
            Button reglage_shooter,
            Trigger eject,
            Button plus_viseur,
            Trigger splus_viseur,
            Button minus_viseur,
            Trigger sminus_viseur,
            Button plus_velo,
            Button minus_velo,
            Button splus_velo,
            Button sminus_velo){

        feeder_button.whenPressed(new Shoot_a_ball_command(feeder, shooter_subsystem, LED));
        feeder_button.whenReleased(new Let_a_ball_pass(feeder));

        intake_button.whenPressed(new Collect_command(intake, Intake_subsystem.WantedState.COLLECT, LED));
        intake_button.whenReleased(new Collect_command(intake, Intake_subsystem.WantedState.STAND_BY, LED));

        shoot_bank_button.toggleWhenPressed(new Configure_shooter(shooter_subsystem,Shooter_Subsystem.WantedState.SHOOT_BANK, true));
        //shoot_bank_button.whenReleased(new Configure_shooter(shooter_subsystem, Shooter_Subsystem.WantedState.WAIT));

        shoot_mid_button.toggleWhenPressed(new Configure_shooter(shooter_subsystem,Shooter_Subsystem.WantedState.SHOOT_MID, true));
        //shoot_mid_button.whenReleased(new Configure_shooter(shooter_subsystem, Shooter_Subsystem.WantedState.WAIT));

        shoot_far_butto.toggleWhenPressed(new Configure_shooter(shooter_subsystem, Shooter_Subsystem.WantedState.SHOOT_FAR, true));
        //shoot_far_butto.whenReleased(new Configure_shooter(shooter_subsystem, Shooter_Subsystem.WantedState.WAIT));

        aspirer_button.toggleWhenPressed(new Stop_shooter(shooter_subsystem));
        //aspirer_button.whenReleased(new Configure_shooter_with_toggle(shooter_subsystem,  apriljoke,Shooter_Subsystem.WantedState.WAIT));

        eject_button.whenActive(new Collect_command(intake, Intake_subsystem.WantedState.EJECT, LED));
        eject_button.whenInactive(new Collect_command(intake, Intake_subsystem.WantedState.STAND_BY, LED));

        aspirer_button.whenActive(new Configure_shooter(shooter_subsystem,Shooter_Subsystem.WantedState.SHOOT_BANK, false));
        aspirer_button.whenInactive(new Stop_shooter(shooter_subsystem));

        plus_velo.whenActive(new InstantCommand(()-> shooter_subsystem.change_velo(10)));
        splus_velo.whenActive(new InstantCommand(()-> shooter_subsystem.change_velo(50)));

        minus_velo.whenActive(new InstantCommand(()-> shooter_subsystem.change_velo(-10)));
        sminus_velo.whenActive(new InstantCommand(()-> shooter_subsystem.change_velo(-50)));


        plus_viseur.whenActive(new InstantCommand(()-> shooter_subsystem.change_visage(0.01)));
        splus_viseur.whenActive(new InstantCommand(()-> shooter_subsystem.change_visage(0.05)));

        minus_viseur.whenActive(new InstantCommand(()-> shooter_subsystem.change_visage(-0.01)));
        sminus_viseur.whenActive(new InstantCommand(()-> shooter_subsystem.change_visage(-0.05)));


        reglage_shooter.toggleWhenPressed(new Configure_shooter(shooter_subsystem, Shooter_Subsystem.WantedState.AUTO, true));



        //alignageButton.whenPressed(new aligner_command(chassis_subsystem, chassis_subsystem));
    }
    public void telemetry (){
        telemetry.addData("Vrai vélocité shooter", shooter_subsystem.shooter.getVelocity());
        telemetry.addData("vélocité programmé shooter", shooter_subsystem.getVeloShooter());
        telemetry.addData("angle viseur", shooter_subsystem.viseur.getPosition());
        telemetry.addData("joystick_gauche", left_joystick.getX());
        telemetry.addData("joystick_droit", right_joystick.getY());
        apriljoke.telemetry();
        telemetry.update();
    }
    public boolean is_In_teleop(){return is_inTeleop;}
    public void Send_telemetry (String key, Object vallue){
        telemetry.addData(key, vallue);
        telemetry.update();
    }
    public double Get_goal_distance (){
        return apriljoke.getActual_detection().ftcPose.y;
    }
    public double get_forward (){
        return left_joystick.getX();
    }
    public double get_forget_turn (){
        return right_joystick.getY();
    }

    //Super utile pour que chaque subsystem ait accé a chaque autre subsystem
    //Il suffit de passer l'instantce actuelle de cette classe en attribut dans le constructeur du subsystem
    //Je suis un génie d'avoir pensé a ca aaaaaah c'est tlm pratique
    public Shooter_Subsystem Shooter (){return shooter_subsystem;}
    public Intake_subsystem Intake (){return intake;}
    public Drive_Train Chassis (){return chassis_subsystem;}
    public Camera_subsystem Camera (){return apriljoke;}
    public Feeder_subsystem Feeder (){return feeder;}
    public joystick_subsystem Left_joystick (){return left_joystick;}
    public joystick_subsystem Right_joystick (){return right_joystick;}
    public RobotMode which_programm (){return team_and_mode;}

    public RevBlinkinLedDriver LED;

    public void ActualiseVoltageSensorValue()
    {
        m_voltageSensorValue = voltageSensor.getVoltage();
    }

    public double GetVoltageSensorValue()
    {
        return m_voltageSensorValue;
    }

}
