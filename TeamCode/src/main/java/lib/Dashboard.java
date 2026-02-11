package lib;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

public class Dashboard {
    static FtcDashboard m_dashboard;

    private static boolean DashboardActivated = true;
    //Change to false to disable sending information to the Dashboard. Ex : in competition
    //Caution : It wouldn't disable the FTCDashboard itself, to do that you must run the OpMode Enable/Disable Dashboard in the DS.

    public static void InitDashboard() {
        if (DashboardActivated)
        {
            m_dashboard = FtcDashboard.getInstance();
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        }
    }

    public static void Telemetry_with_Text(String key, String text) {
        telemetry.addData(key, text);
    }

    public static void Telemetry_with_number(String key, double number) {
        telemetry.addData(key, number);
    }

    public static void TelemetryUpdate() {
        telemetry.update();
    }
}
