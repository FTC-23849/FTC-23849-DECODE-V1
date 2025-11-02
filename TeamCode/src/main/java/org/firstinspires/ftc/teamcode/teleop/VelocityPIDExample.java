package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.PIDVelocityController;

@Config
@TeleOp(name = "Velocity PID Example")
public class VelocityPIDExample extends LinearOpMode {
    private DcMotorEx topShooterMotor;
    private DcMotorEx bottomShooterMotor;
    private PIDVelocityController velocityPID;
    public static double power;
    public static double TargetVelocity;
    public static double currentVelocity;
    public static double Kp = 0.00107;
    public static double Ki = 1.7;
    public static double Kd = 0.000007;
    public static double Kv = 0.000627;


    @Override
    public void runOpMode() {
        ElapsedTime timer = new ElapsedTime();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        topShooterMotor = hardwareMap.get(DcMotorEx.class,"topShooterMotor");
        bottomShooterMotor = hardwareMap.get(DcMotorEx.class,"bottomShooterMotor");
        topShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        TargetVelocity = 900;
        while (opModeIsActive()) {
            timer.reset();
            if (Math.abs(TargetVelocity-currentVelocity) > 20) {
                 if ((TargetVelocity - currentVelocity) < 0 ){
                     velocityPID.tuneFeedForward(1,Kv);
                 }else {
                     velocityPID.tuneFeedForward(-1,Kv);
                 }
            }
            if (Math.abs(TargetVelocity-currentVelocity) > 20) {
                if ((TargetVelocity - currentVelocity) < 0 ){
                    velocityPID.tuneFeedForward(1,Kv);
                }else {
                    velocityPID.tuneFeedForward(-1,Kv);
                }
            }
            velocityPID = new PIDVelocityController(Kp, Ki, Kd,Kv, TargetVelocity);
            velocityPID.reset();
            currentVelocity = topShooterMotor.getVelocity();

            power = velocityPID.update(currentVelocity);
            topShooterMotor.setPower(power);
            double steadyStateError = Math.abs(velocityPID.getTargetVelocity() - topShooterMotor.getVelocity());
            telemetry.addData("Steady-State Error", steadyStateError);
            telemetry.addData("Target Veocity", TargetVelocity);
            telemetry.addData("Current Velocity", currentVelocity);
            telemetry.addData("Motor Power", power);
            telemetry.update();
        }
    }
}
