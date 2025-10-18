package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.v1;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;



@TeleOp(name = "v1teleop", group = "stuff")

public class v1teleop extends OpMode {
    DcMotorEx leftFrontMotor;
    DcMotorEx rightFrontMotor;
    DcMotorEx leftBackMotor;
    DcMotorEx rightBackMotor;
    DcMotorEx intakeMotor;
    DcMotorEx transferMotor;
    DcMotorEx topShooterMotor;
    DcMotorEx bottomShooterMotor;
    CRServo transferServoLeft;
    CRServo transferServoRight;
    //GoBildaPinpointDriver pinpoint;
    Servo light;


    double rx;
    boolean shooting;

    double closeShootingPower = v1.defaultCloseZonePower;
    double farShootingPower = v1.defaultFarZonePower;
    boolean intaking = false;
    ElapsedTime transferReverse = new ElapsedTime();

    @Override
    public void init() {
        leftFrontMotor  = hardwareMap.get(DcMotorEx.class,"LF");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "RF");
        leftBackMotor = hardwareMap.get(DcMotorEx.class,"LB");
        rightBackMotor = hardwareMap.get(DcMotorEx.class,"RB");
        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);

        topShooterMotor = hardwareMap.get(DcMotorEx.class,"topShooterMotor");
        bottomShooterMotor = hardwareMap.get(DcMotorEx.class,"bottomShooterMotor");
        bottomShooterMotor.setDirection(DcMotorEx.Direction.REVERSE);

        topShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");

        transferServoLeft = hardwareMap.get(CRServo.class, "transferServoLeft");
        transferServoRight = hardwareMap.get(CRServo.class, "transferServoRight");
        /*pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(1,1,DistanceUnit.MM);
        pinpoint.setEncoderResolution(1, DistanceUnit.MM);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES,0));
        */
        light = hardwareMap.get(Servo.class,"light");
    }

    @Override
    public void loop() {
        //Pose2D pinpointPosition = pinpoint.getPosition();
        telemetry.addData("rx",rx);
        telemetry.addData("shooting",shooting);
        telemetry.addData("close shoot power",closeShootingPower);
        telemetry.addData("farShootingPower",farShootingPower);
        telemetry.addData("intaking",intaking);
        telemetry.addData("transferreferse timer",transferReverse.milliseconds());
        //telemetry.addData("heading",pinpointPosition.getHeading(AngleUnit.DEGREES));


        //drive
        double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
        double x = gamepad1.left_stick_x;
        if(shooting == true){
            rx = gamepad1.right_stick_x * v1.shootingturnspeed;
        }
        if(shooting == false){
            rx = gamepad1.right_stick_x;
        }

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (y + x + rx) / denominator;
        double leftBackPower = (y - x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double rightBackPower = (y + x - rx) / denominator;

        leftFrontMotor.setPower(leftFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightFrontMotor.setPower(rightFrontPower);
        rightBackMotor.setPower(rightBackPower);
        //intake
        if(gamepad1.right_trigger > 0.1){
            intaking = true;
            intakeMotor.setPower(1);
            transferMotor.setPower(v1.transferIntake);
            transferServoLeft.setPower(v1.transferServoIntake);
            transferServoRight.setPower(v1.transferServoIntake * -1);
            transferReverse.reset();
        }
        //reverse intake
        else if(gamepad1.a){
            transferServoLeft.setPower(v1.transferServoReverse);
            transferServoRight.setPower(v1.transferServoReverse * -1);
            transferMotor.setPower(v1.transferMotorReverse);
            intakeMotor.setPower(-1);
        }
        //reverse intake once finished so ball isnt stuck in shooter
        else if(!intaking && transferReverse.milliseconds() < 250){
            transferServoLeft.setPower(v1.transferServoReverse * -1);
            transferServoRight.setPower(v1.transferServoReverse);
            transferMotor.setPower(v1.transferMotorReverse);
            intakeMotor.setPower(0);
            topShooterMotor.setPower(-1);
            bottomShooterMotor.setPower(-1);
        }
        //shooting
        else if(gamepad1.left_trigger > 0.1){
            transferServoLeft.setPower(v1.transferServoShoot*-1);
            transferServoRight.setPower(v1.transferServoShoot);
            transferMotor.setPower(v1.transferMotorShoot);
            intakeMotor.setPower(v1.intakeShoot);

        }
        else{
            intakeMotor.setPower(0);
            transferServoLeft.setPower(0);
            transferServoRight.setPower(0);
            transferMotor.setPower(0);
            intaking = false;
        }
        //shoot from close zone
        if(gamepad1.left_bumper){
            topShooterMotor.setPower(closeShootingPower);
            bottomShooterMotor.setPower(closeShootingPower);
            shooting = true;
            /*if(pinpointPosition.getHeading(AngleUnit.DEGREES) > (v1.blueCloseAngle - 2) && pinpointPosition.getHeading(AngleUnit.DEGREES)<(v1.blueCloseAngle+2) || pinpointPosition.getHeading(AngleUnit.DEGREES) > (v1.redCloseAngle - 2) && pinpointPosition.getHeading(AngleUnit.DEGREES)<(v1.redCloseAngle+2)){
                light.setPosition(1);
            }*/
        }
        // shoot from far zone
        else if(gamepad1.right_bumper){
            topShooterMotor.setPower(farShootingPower);
            bottomShooterMotor.setPower(farShootingPower);
            shooting = true;
            /*if(pinpointPosition.getHeading(AngleUnit.DEGREES) > (v1.blueFarAngle - 2) && pinpointPosition.getHeading(AngleUnit.DEGREES)<(v1.blueFarAngle+2)|| pinpointPosition.getHeading(AngleUnit.DEGREES) > (v1.redFarAngle - 2) && pinpointPosition.getHeading(AngleUnit.DEGREES)<(v1.redFarAngle+2)){
                light.setPosition(1);
            }*/
        }
        else{
            topShooterMotor.setPower(0);
            bottomShooterMotor.setPower(0);
            light.setPosition(0);
            shooting = false;
        }
        if(gamepad1.b){
            //pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES,0));
        }




        // adjusting shooter values if needed in match
        if(!gamepad1.dpadUpWasPressed()){
            farShootingPower = farShootingPower - 0.01;

        }
        if(!gamepad1.dpadDownWasPressed()){
            farShootingPower = farShootingPower + 0.01;

        }
        if(!gamepad1.dpadLeftWasPressed()){
            closeShootingPower = closeShootingPower + 0.01;

        }
        if(!gamepad1.dpadRightWasPressed()){
            closeShootingPower = closeShootingPower - 0.01;

        }
        if(gamepad1.right_bumper&&gamepad1.left_bumper){
            closeShootingPower = v1.defaultCloseZonePower;
            farShootingPower = v1.defaultFarZonePower;
        }

    }

}

