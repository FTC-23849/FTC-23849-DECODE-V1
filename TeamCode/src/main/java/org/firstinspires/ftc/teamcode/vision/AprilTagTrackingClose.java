package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.IMU;
//left front and left back reversed to maek it compatible with teleop


public class AprilTagTrackingClose {
    public boolean AprilTagTracker(DcMotorEx leftFrontMotor, DcMotorEx leftBackMotor, DcMotorEx rightFrontMotor, DcMotorEx rightBackMotor, Limelight3A limelight, IMU imu) {
        double Offset = -7.11;
        LLResult result = limelight.getLatestResult();
        if (result.isValid()& result != null) {
            leftFrontMotor.setDirection(DcMotorEx.Direction.FORWARD);
            leftBackMotor.setDirection(DcMotorEx.Direction.FORWARD);
            result = limelight.getLatestResult();
            leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            double turnSpeed = 0.3;
            double ErrorMarginTurning = 2;
            double ErrorMarginTagSize = 0.2;
            double yval = result.getTy();
            double xval = result.getTx();
            double TagSize = result.getTa();
            double TargetSize = 2.1;
            leftFrontMotor.setPower(0);
            leftBackMotor.setPower(0);

            rightFrontMotor.setPower(0);
            rightBackMotor.setPower(0);

            if (Math.abs(TargetSize - TagSize) > ErrorMarginTagSize) {
                double distance = (Math.abs(TargetSize - TagSize) < 0.3 ? 0.3 : Math.abs(TargetSize - TagSize) );
                double denominator = 1;
                if (TagSize < TargetSize) {
                    denominator = 1;
                } else {
                    denominator = -1;
                }

                double leftFrontPower = (distance) / denominator;
                double leftBackPower = (distance) / denominator;
                double rightFrontPower = (distance) / denominator;
                double rightBackPower = (distance) / denominator;
                leftFrontMotor.setPower(leftFrontPower);
                leftBackMotor.setPower(leftBackPower);
                rightFrontMotor.setPower(-rightFrontPower);
                rightBackMotor.setPower(-rightBackPower);
            } else {
                leftFrontMotor.setPower(0);
                leftBackMotor.setPower(0);
                rightFrontMotor.setPower(0);
                rightBackMotor.setPower(0);
            }
            boolean Check = false;
            if (Math.abs(yval-Offset) >= ErrorMarginTurning) {
                double FOV = 17;
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

                if (Math.abs(yval-Offset) <= ErrorMarginTurning & Math.abs(TargetSize - TagSize) < ErrorMarginTagSize) {
                    Check = true;
                }
            }

            return Check;
        }
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        return true;
    }
}