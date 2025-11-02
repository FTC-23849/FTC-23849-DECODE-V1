package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.openftc.easyopencv.OpenCvCamera;

@TeleOp(name = "OpenCV Intake Blob", group = "stuff")

public class IntakeBlob extends OpMode {

    private OpenCvCamera GroundCam;
    private Limelight3A limelight;
    private IMU imu;

    DcMotorEx leftFrontMotor;
    DcMotorEx rightFrontMotor;
    DcMotorEx leftBackMotor;
    DcMotorEx rightBackMotor;

    double turnSpeed = 0.5;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry,dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(GroundCam,30);
        new BlobDetection();
        leftFrontMotor  = hardwareMap.get(DcMotorEx.class,"LF");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "RF");
        leftBackMotor = hardwareMap.get(DcMotorEx.class,"LB");
        rightBackMotor = hardwareMap.get(DcMotorEx.class,"RB");
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("running","");
        telemetry.update();
    }

    @Override
    public void start() {


    }



    @Override
    public void loop() {
        double offset = Math.abs(BlobDetection.getOffsetX());
        telemetry.addData("Offset",offset);
        telemetry.update();
        if (Math.abs(offset) <= 20) {
            leftFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightBackMotor.setPower(0);
            telemetry.addData("Status:","reached");
            telemetry.update();
        } else {
            double rotationalSpeed = (offset / 320 < 0.3 ? 0.1  : offset / 640 );
            double denominator = (offset < 0 ? -1  : 1);
            double leftFrontPower = (rotationalSpeed) / denominator;
            double leftBackPower = (rotationalSpeed) / denominator;
            double rightFrontPower = (rotationalSpeed) / denominator;
            double rightBackPower = (rotationalSpeed) / denominator;

            leftFrontMotor.setPower(leftFrontPower);
            leftBackMotor.setPower(leftBackPower);
            rightFrontMotor.setPower(rightFrontPower);
            rightBackMotor.setPower(rightBackPower);
            telemetry.addData("Status: ", "searching");
            telemetry.addData("speed", rotationalSpeed);
            telemetry.update();
        }


    }
}

