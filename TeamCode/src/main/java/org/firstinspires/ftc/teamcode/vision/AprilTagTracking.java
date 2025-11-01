package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@TeleOp(name = "AprilTagTracking")
public class AprilTagTracking extends OpMode {
    private Limelight3A limelight;
    private IMU imu;

    DcMotorEx leftFrontMotor;
    DcMotorEx rightFrontMotor;
    DcMotorEx leftBackMotor;
    DcMotorEx rightBackMotor;

    double turnSpeed = 0.5;

    @Override
    public void init() {

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(8);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));


        leftFrontMotor  = hardwareMap.get(DcMotorEx.class,"LF");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "RF");
        leftBackMotor = hardwareMap.get(DcMotorEx.class,"LB");
        rightBackMotor = hardwareMap.get(DcMotorEx.class,"RB");
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
    }

    @Override
    public void start() {
        limelight.start();
    }



    @Override
    public void loop() {

        LLResult llResult = limelight.getLatestResult();


        double yval = llResult.getTy();
        double TagSize = llResult.getTa();
        double CorrectOrientation;
        if (Math.abs(yval) <= 1.3) {
            leftFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightBackMotor.setPower(0);
            if (Math.abs(2.4-TagSize)>0.1) {
                double distance = (Math.abs(2 - TagSize))/2;
                double denominator = 1;
                if (TagSize < 2) {
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
            }
        } else {
            double rotationalInput = (yval > 0 ? -1 : 1) * turnSpeed;
            turnSpeed =  (yval / 32 < 0.3 ? 0.1  : yval / 48 );
            double denominator = Math.max( Math.abs(rotationalInput), 1);
            double leftFrontPower = (rotationalInput) / denominator;
            double leftBackPower = ( rotationalInput) / denominator;
            double rightFrontPower = ( rotationalInput) / denominator;
            double rightBackPower = ( rotationalInput) / denominator;

            leftFrontMotor.setPower(leftFrontPower);
            leftBackMotor.setPower(leftBackPower);
            rightFrontMotor.setPower(rightFrontPower);
            rightBackMotor.setPower(rightBackPower);
        }


    }

}