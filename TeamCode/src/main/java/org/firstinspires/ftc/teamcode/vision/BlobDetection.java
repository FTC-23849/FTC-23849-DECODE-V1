package org.firstinspires.ftc.teamcode.vision;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "OpenCV Blob Correction")
public class BlobDetection extends LinearOpMode {
    double cX = 0;
    double cY = 0;
    double offsetX = 0;
    DcMotorEx leftFront;
    DcMotorEx rightFront;
    DcMotorEx leftBack;
    DcMotorEx rightBack;

    private OpenCvCamera controlHubCam;
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;

    private static final double TARGET_OFFSET_PX = 30;
    private static final double ERROR_MARGIN = 10;
    private static final double BASE_TURN_SPEED = 0.05;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotorEx.class, "LF");
        rightFront = hardwareMap.get(DcMotorEx.class, "RF");
        leftBack = hardwareMap.get(DcMotorEx.class, "LB");
        rightBack = hardwareMap.get(DcMotorEx.class, "RB");
        leftFront.setDirection(REVERSE);
        leftBack.setDirection(REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initOpenCV();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        waitForStart();

        while (opModeIsActive()) {
            double centerX = CAMERA_WIDTH / 2.0;
            offsetX = cX - centerX;

            telemetry.addData("Center (x,y)", "(" + (int) cX + ", " + (int) cY + ")");
            telemetry.addData("Offset X (px)", String.format("%.0f", offsetX));

            double error = offsetX - TARGET_OFFSET_PX;

            if (Math.abs(error) > ERROR_MARGIN) {
                double turnSpeed = BASE_TURN_SPEED + 0.25 * (Math.abs(error) / 200.0);
                turnSpeed = Math.min(turnSpeed, 0.4);
                double direction = (error < 0) ? -1 : 1;
                double leftPower = direction * turnSpeed;
                double rightPower = -direction * turnSpeed;

                leftFront.setPower(leftPower);
                leftBack.setPower(leftPower);
                rightFront.setPower(rightPower);
                rightBack.setPower(rightPower);

                telemetry.addData("Turning", direction > 0 ? "Right" : "Left");
            } else {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightFront.setPower(0);
                rightBack.setPower(0);
                telemetry.addData("Status", "Aligned");
            }

            telemetry.update();
        }

        controlHubCam.stopStreaming();
    }

    private void initOpenCV() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "GroundCam"), cameraMonitorViewId);
        controlHubCam.setPipeline(new BallDetectionPipeline());
        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }

    class BallDetectionPipeline extends OpenCvPipeline {
        private static final double MIN_AREA_THRESHOLD = 500.0;
        private static final double MERGE_DISTANCE_PX = 100.0;

        @Override
        public Mat processFrame(Mat input) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(input, hsvFrame, Imgproc.COLOR_RGB2HSV);

            Scalar lowerPurple = new Scalar(117, 3, 64);
            Scalar upperPurple = new Scalar(155, 231, 220);
            Scalar lowerGreen = new Scalar(68, 82, 0);
            Scalar upperGreen = new Scalar(84, 255, 220);

            Mat maskPurple = new Mat();
            Mat maskGreen = new Mat();
            Core.inRange(hsvFrame, lowerPurple, upperPurple, maskPurple);
            Core.inRange(hsvFrame, lowerGreen, upperGreen, maskGreen);

            Mat combinedMask = new Mat();
            Core.bitwise_or(maskPurple, maskGreen, combinedMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(combinedMask, combinedMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(combinedMask, combinedMask, Imgproc.MORPH_CLOSE, kernel);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(combinedMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            List<Rect> validRects = new ArrayList<>();
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > MIN_AREA_THRESHOLD) {
                    Rect rect = Imgproc.boundingRect(contour);
                    validRects.add(rect);
                }
            }

            List<Rect> mergedRects = mergeCloseRects(validRects, MERGE_DISTANCE_PX);

            for (Rect rect : mergedRects) {
                Imgproc.rectangle(input, rect, new Scalar(0, 255, 0), 2);
            }

            if (!mergedRects.isEmpty()) {
                Rect largest = mergedRects.get(0);
                for (Rect r : mergedRects) {
                    if (r.area() > largest.area()) largest = r;
                }
                cX = largest.x + largest.width / 2.0;
                cY = largest.y + largest.height / 2.0;
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(255, 0, 0), -1);
            }

            return input;
        }

        private List<Rect> mergeCloseRects(List<Rect> rects, double maxDistance) {
            boolean[] merged = new boolean[rects.size()];
            List<Rect> results = new ArrayList<>();
            for (int i = 0; i < rects.size(); i++) {
                if (merged[i]) continue;
                Rect r1 = rects.get(i);
                Rect mergedRect = new Rect(r1.x, r1.y, r1.width, r1.height);
                for (int j = i + 1; j < rects.size(); j++) {
                    if (merged[j]) continue;
                    Rect r2 = rects.get(j);
                    if (rectsAreClose(mergedRect, r2, maxDistance)) {
                        mergedRect = unionRect(mergedRect, r2);
                        merged[j] = true;
                    }
                }
                results.add(mergedRect);
            }
            return results;
        }

        private boolean rectsAreClose(Rect r1, Rect r2, double maxDistance) {
            Point c1 = new Point(r1.x + r1.width / 2.0, r1.y + r1.height / 2.0);
            Point c2 = new Point(r2.x + r2.width / 2.0, r2.y + r2.height / 2.0);
            double dx = c1.x - c2.x;
            double dy = c1.y - c2.y;
            double distance = Math.sqrt(dx * dx + dy * dy);
            return distance < maxDistance;
        }

        private Rect unionRect(Rect r1, Rect r2) {
            int x = Math.min(r1.x, r2.x);
            int y = Math.min(r1.y, r2.y);
            int x2 = Math.max(r1.x + r1.width, r2.x + r2.width);
            int y2 = Math.max(r1.y + r1.height, r2.y + r2.height);
            return new Rect(x, y, x2 - x, y2 - y);
        }
    }
}
