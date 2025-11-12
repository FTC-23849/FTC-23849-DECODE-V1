package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.auto.PIDVelocityController;
import org.firstinspires.ftc.teamcode.v1;

@Config
@TeleOp(name = "Velocity PID Example")
public class VelocityPIDExample extends LinearOpMode {
    CRServo transferServoLeft;
    CRServo transferServoRight;
    DcMotorEx intakeMotor;
    DcMotorEx transferMotor;
    private DcMotorEx topShooterMotor;
    private DcMotorEx bottomShooterMotor;
    private PIDVelocityController velocityPID;

    public static double TargetVelocity = 900;
    public static double Kp = 0.00107;
    public static double Ki = 0;
    public static double Kd = 0.000007;
    public static double Kv = 0.000627;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        topShooterMotor = hardwareMap.get(DcMotorEx.class, "topShooterMotor");
        bottomShooterMotor = hardwareMap.get(DcMotorEx.class, "bottomShooterMotor");
        transferServoLeft = hardwareMap.get(CRServo.class, "transferServoLeft");
        transferServoRight = hardwareMap.get(CRServo.class, "transferServoRight");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        topShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        topShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        topShooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
        bottomShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bottomShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        velocityPID = new PIDVelocityController(Kp, Ki, Kd, Kv, TargetVelocity);

        waitForStart();

        while (opModeIsActive()) {
            transferServoLeft.setPower(-0.8);
            transferServoRight.setPower(0.8);
            transferMotor.setPower(0.8);
            intakeMotor.setPower(0.8);
            double currentVelocity = topShooterMotor.getVelocity();
            velocityPID.setTargetVelocity(TargetVelocity);
            velocityPID.setGains(Kp, Ki, Kd);
            velocityPID.setFeedforward(Kv);

            double power = velocityPID.update(currentVelocity);
            topShooterMotor.setPower(power);
            bottomShooterMotor.setPower(power);

            telemetry.addData("Target Velocity", TargetVelocity);
            telemetry.addData("Current Velocity", currentVelocity);
            telemetry.addData("Motor Power", power);
            telemetry.addData("Kv (Live)", velocityPID.getFeedforward());
            telemetry.addData("Kp", Kp);
            telemetry.addData("Ki", Ki);
            telemetry.addData("Kd", Kd);
            telemetry.update();
        }
    }
}
