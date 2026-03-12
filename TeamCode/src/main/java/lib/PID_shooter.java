package lib;

import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.seuil_volt_shooter;

public class PID_shooter {
    private double kp;
    private double ki;
    private double kd;
    private double feedforward;

    private double outputMin = -1.0;
    private double outputMax = 1.0;
    private double inputMin;
    private double inputMax;

    private double previousError = 0.0;
    private double integrative = 0.0;
    private double setpoint = 0.0;
    private double currentError = 0.0;

    private double output = 0.0;
    private double tolerance = 0.0;

    private double lastTimestamp = 0.0;
    private double dt = 20;

    public PID_shooter(double kp, double ki, double kd, double ff) {
        SetGains(kp, ki, kd, ff);
        Reset();
    }
    public void SetTolerance(double tolerance) {
        this.tolerance = tolerance;
    }
    public void SetGains(double kp, double ki, double kd, double ff) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.feedforward = ff;
    }
    public void Reset() {
        setpoint = setpoint - currentError; // hold current position
        previousError = 0.0;
        currentError = 0.0;
        output = 0.0;
    }
    public void set_dt(double timestamp){this.dt=timestamp;}
    public double GetKP() { return kp; }
    public double GetKI() { return ki; }
    public double GetKD() { return kd; }
    public double GetFF() { return feedforward; }
    public double GetError() { return currentError; }
    public double GetSetpoint() { return setpoint; }
    public double Getdt() { return dt; }
    public String GetState() {
        return String.format("PID[kp=%.3f, ki=%.3f, kd=%.3f, ff=%.3f, setpoint=%.3f, error=%.3f, output=%.3f]",
                kp, ki, kd, feedforward, setpoint, currentError, output);
    }
    public double Calculate_Power(double setpoint, double measurement) {
        this.setpoint = setpoint;
        return calculateInternal(measurement);
    }
    public double Calculate_simple (double setpoint, double measurement){
        currentError = setpoint-measurement;
        integrative += currentError * dt;
        double derivative = (currentError - previousError) / dt;
        return 0;
    }

    public double calculateInternal(double measurement) {
        currentError = setpoint - measurement;
        if((currentError * kp) < seuil_volt_shooter && (currentError * kp) > -seuil_volt_shooter)
        {
            integrative += currentError * dt;
        }
        else
        {
            integrative = 0.0;
        }

        // If error is above tolerance, calculate full PID output
        if(Math.abs(currentError) >= tolerance)
        {
            output =  kp * currentError +
                    ki * integrative +
                    kd * ((currentError - previousError) / dt) +
                    feedforward*setpoint;
        }
        else
        {
            // Near target: skip proportional term to reduce overshoot
            output = ki * integrative +
                    kd * ((currentError - previousError) / dt) +
                    feedforward*setpoint;
        }
        previousError = currentError;
        return clamp(output, -seuil_volt_shooter, seuil_volt_shooter);
    }
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
