package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDVelocityController {
    private double Kp, Ki, Kd, Kv;
    private double targetVelocity;
    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();

    public PIDVelocityController(double Kp, double Ki, double Kd, double Kv, double targetVelocity) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kv = Kv;
        this.targetVelocity = targetVelocity;
        timer.reset();
    }

    public double update(double currentVelocity) {
        double error = targetVelocity - currentVelocity;
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0) dt = 1e-6;
        integralSum += error * dt;
        double derivative = (error - lastError) / dt;
        double pidOutput = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
        lastError = error;
        double output = pidOutput + Kv * targetVelocity;
        return Math.max(-1.0, Math.min(1.0, output));
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }

    public void setTargetVelocity(double newTarget) {
        this.targetVelocity = newTarget;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public void setGains(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public void setFeedforward(double Kv) {
        this.Kv = Kv;
    }
}
