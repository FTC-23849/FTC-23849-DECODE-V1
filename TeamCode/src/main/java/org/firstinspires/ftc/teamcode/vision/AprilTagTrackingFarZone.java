package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.IMU;
//left front and left back reversed to maek it compatible with teleop


public class AprilTagTrackingFarZone {
    public boolean AprilTagTracker(DcMotorEx leftFrontMotor, DcMotorEx leftBackMotor, DcMotorEx rightFrontMotor, DcMotorEx rightBackMotor,IMU imu, Limelight3A limelight) {
        double Offset = -5.5;
        double ErrorMargin = 0.5;
        LLResult result = limelight.getLatestResult();
        if (result.isValid()& result != null) {
            leftFrontMotor.setDirection(DcMotorEx.Direction.FORWARD);
            leftBackMotor.setDirection(DcMotorEx.Direction.FORWARD);
            result = limelight.getLatestResult();
            leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            double turnSpeed = 0.5;
            double yval = result.getTy();
            double xval = result.getTx();
            double TagSize = result.getTa();

            leftFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightBackMotor.setPower(0);

            boolean Check = false;
            if (Math.abs(yval-Offset) >= ErrorMargin) {
                double FOV = 17;
                ///if(Math.abs(yval - Offset) < 1){
                ///    turnSpeed = 0.1;
                ///}
                double rotationalInput = (yval < Offset ? -1 : 1) * turnSpeed;
                double denominator = Math.max(Math.abs(rotationalInput), 1);
                double leftFrontPower = (rotationalInput) / denominator;
                double leftBackPower = (rotationalInput) / denominator;
                double rightFrontPower = (rotationalInput) / denominator;
                double rightBackPower = (rotationalInput) / denominator;

                leftFrontMotor.setPower(leftFrontPower);
                leftBackMotor.setPower(leftBackPower);
                rightFrontMotor.setPower(rightFrontPower);
                rightBackMotor.setPower(rightBackPower);
                if (Math.abs(yval-Offset) <= ErrorMargin ) {
                Check = true;
                }

            }
        return Check;
        }
    leftFrontMotor.setPower(0);
    leftBackMotor.setPower(0);
    rightFrontMotor.setPower(0);
    rightBackMotor.setPower(0);
    return false;
    }
}