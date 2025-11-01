package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
//left front and left back reversed to maek it compatible with teleop
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class AprilTagTrackingTeleopQuick {
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
            double yval = result.getTy();
            double xval = result.getTx();
            double TagSize = result.getTa();
            double CorrectOrientation;
            double TargetSize = 2.1;
            leftFrontMotor.setPower(0);
            leftBackMotor.setPower(0);

            rightFrontMotor.setPower(0);
            rightBackMotor.setPower(0);

            if (Math.abs(TargetSize - TagSize) > 0.2) {
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
            double LimelightOffsetCorrection = -Offset;
            if (Math.abs(yval+Offset) >= 2) {
                double FOV = 17;
                double rotationalInput = (yval < LimelightOffsetCorrection ? -1 : 1) * turnSpeed;
                turnSpeed = ((Math.abs(yval+Offset) / 17) < 0.2 ? 0.04 : yval / 17);
                double denominator = Math.max(Math.abs(rotationalInput), 1);
                double leftFrontPower = (rotationalInput) / denominator;
                double leftBackPower = (rotationalInput) / denominator;
                double rightFrontPower = (rotationalInput) / denominator;
                double rightBackPower = (rotationalInput) / denominator;

                leftFrontMotor.setPower(leftFrontPower);
                leftBackMotor.setPower(leftBackPower);
                rightFrontMotor.setPower(rightFrontPower);
                rightBackMotor.setPower(rightBackPower);

                if (Math.abs(yval) >= 2 & Math.abs(TargetSize - TagSize) < 0.2) {
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