package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp
public class VelocityPIDTuner extends OpMode {

    DcMotorEx topShooterMotor;
    DcMotorEx bottomShooterMotor;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 12;
    PIDFCoefficients SHOOTER_VELOCITY_PID = new PIDFCoefficients(kP, kI, kD, kF);

    private static double MOTOR_GEAR_RATIO = 1;
    private static double MOTOR_TICKS_PER_REVOLUTION = 28;
    private static double MOTOR_MAX_RPM = 5800;

    public static double targetVelocity = 0;

    @Override
    public void init() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        topShooterMotor = hardwareMap.get(DcMotorEx.class,"topShooterMotor");
        bottomShooterMotor = hardwareMap.get(DcMotorEx.class,"bottomShooterMotor");

        topShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topShooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }

    @Override
    public void loop() {

        setPIDFCoefficients(topShooterMotor, bottomShooterMotor, SHOOTER_VELOCITY_PID);
        topShooterMotor.setVelocity(rpmToTicksPerSecond(targetVelocity));
        bottomShooterMotor.setVelocity(rpmToTicksPerSecond(targetVelocity));

        printVelocity(topShooterMotor, bottomShooterMotor, targetVelocity);
        telemetry.update();

    }

    private void setPIDFCoefficients(DcMotorEx motor1, DcMotorEx motor2, PIDFCoefficients coefficients) {
        motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f));

        motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f));
    }

//    public static double getMotorVelocityF() {
//        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
//        return 32767 * 60.0 / (MOTOR_MAX_RPM * MOTOR_TICKS_PER_REVOLUTION);
//    }

    public static double rpmToTicksPerSecond(double rpm) {
        return (rpm * MOTOR_TICKS_PER_REVOLUTION) / 60;
    }

    private void printVelocity(DcMotorEx topshootermotor, DcMotorEx bottomshootermotor, double target) {
        telemetry.addData("targetVelocity", rpmToTicksPerSecond(target));

        double topshootermotorVelo = topshootermotor.getVelocity();
        telemetry.addData("topTrueVel", topshootermotorVelo);
        telemetry.addData("toperror", rpmToTicksPerSecond(target) - topshootermotorVelo);

        double bottomshootermotorVelo = bottomshootermotor.getVelocity();
        telemetry.addData("bottomTrueVel", bottomshootermotorVelo);
        telemetry.addData("bottomerror", rpmToTicksPerSecond(target) - bottomshootermotorVelo);
    }

}
