package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.RRFiles.MecanumDrive;
import org.firstinspires.ftc.teamcode.v1;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class blueFarAuto extends LinearOpMode {

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

    ElapsedTime timer = new ElapsedTime();

    double minVel = 90;
    double minAccel = -90;
    double maxAccel = 90;

    @Override
    public void runOpMode() {

        // Create Roadrunner Trajectories

        Pose2d startPose = new Pose2d(63.5, -13.6, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        //map motors and servos
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

        light = hardwareMap.get(Servo.class,"light");


        waitForStart();

        if (isStopRequested()) return;

        sleep(4);

        //score preload
        TrajectoryActionBuilder scorePreloads = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(52, -15), Math.toRadians(20));

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        scorePreloads.build(),
                        new setShooter(topShooterMotor, bottomShooterMotor, v1.autoDefaultFarZonePower)
                ),
                new SleepAction(1),
                new feedBalls(transferMotor, intakeMotor, transferServoLeft, transferServoRight),
                new SleepAction(3),
                new stopFeed(transferMotor, intakeMotor, transferServoLeft, transferServoRight)
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();

        //Collect HP
        TrajectoryActionBuilder goToIntakeHP = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(42, -64), Math.toRadians(0));

        Actions.runBlocking(new ParallelAction(
                goToIntakeHP.build(),
                new stopFeed(transferMotor, intakeMotor, transferServoLeft, transferServoRight),
                new setIntake(intakeMotor, transferMotor, 1.0),
                new setShooter(topShooterMotor, bottomShooterMotor, 0.0)
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();



        TrajectoryActionBuilder intakeHP = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(62, -64), Math.toRadians(0), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-50, 50));

        Actions.runBlocking(new SequentialAction(
                intakeHP.build(),
                new setIntake(intakeMotor, transferMotor, 0.0)
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();



        //shoot 3rd spike mark
        TrajectoryActionBuilder scoreHP = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(52, -15), Math.toRadians(20));

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        scoreHP.build(),
                        new setShooter(topShooterMotor, bottomShooterMotor, v1.autoDefaultFarZonePower)
                ),
                new SleepAction(1),
                new feedBalls(transferMotor, intakeMotor, transferServoLeft, transferServoRight),
                new SleepAction(1.8),
                new stopFeed(transferMotor, intakeMotor, transferServoLeft, transferServoRight)
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();

        //park
        TrajectoryActionBuilder park = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(57, -30), Math.toRadians(90));

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        park.build(),
                        new setShooter(topShooterMotor, bottomShooterMotor, 0.0),
                        new stopFeed(transferMotor, intakeMotor, transferServoLeft, transferServoRight)
                )
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();

        sleep(1000);

    }

    public class setShooter implements Action {

        DcMotorEx topShooterMotor;
        DcMotorEx bottomShooterMotor;

        double power;

        public setShooter(DcMotorEx topShooterMotor, DcMotorEx bottomShooterMotor, double power){
            this.topShooterMotor = topShooterMotor;
            this.bottomShooterMotor = bottomShooterMotor;
            this.power = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            topShooterMotor.setPower(power);
            bottomShooterMotor.setPower(power);

            return false;
        }
    }

    public class feedBalls implements Action {

        DcMotorEx transferMotor;
        DcMotorEx intakeMotor;
        CRServo transferServoLeft;
        CRServo transferServoRight;

        public feedBalls(DcMotorEx transferMotor, DcMotorEx intakeMotor, CRServo transferServoLeft, CRServo transferServoRight){
            this.transferMotor = transferMotor;
            this.intakeMotor = intakeMotor;
            this.transferServoLeft = transferServoLeft;
            this.transferServoRight = transferServoRight;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            transferMotor.setPower(v1.transferMotorShoot);
            intakeMotor.setPower(v1.intakeShoot);
            transferServoLeft.setPower(-v1.transferServoShoot);
            transferServoRight.setPower(v1.transferServoShoot);

            return false;
        }
    }

    public class stopFeed implements Action {

        DcMotorEx transferMotor;
        DcMotorEx intakeMotor;
        CRServo transferServoLeft;
        CRServo transferServoRight;

        public stopFeed(DcMotorEx transferMotor, DcMotorEx intakeMotor, CRServo transferServoLeft, CRServo transferServoRight){
            this.transferMotor = transferMotor;
            this.intakeMotor = intakeMotor;
            this.transferServoLeft = transferServoLeft;
            this.transferServoRight = transferServoRight;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            transferMotor.setPower(0.0);
            intakeMotor.setPower(0.0);
            transferServoLeft.setPower(0.0);
            transferServoRight.setPower(0.0);

            return false;
        }
    }

    public class setIntake implements Action {

        DcMotorEx intakeMotor;
        DcMotorEx transferMotor;

        double power;

        public setIntake(DcMotorEx intakeMotor, DcMotorEx transferMotor, double power){
            this.intakeMotor = intakeMotor;
            this.transferMotor = transferMotor;
            this.power = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            intakeMotor.setPower(power);
            //transferMotor.setPower(v1.transferIntake);

            return false;
        }
    }

}