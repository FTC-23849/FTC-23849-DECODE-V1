package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.*;

import java.util.ArrayList;
import java.util.List;
//wrong file
@TeleOp(name = "OpenCV X Offset Detection")
public class IntakeBlob extends LinearOpMode {
    double cX = 0;
    double cY = 0;
    double offsetX = 0;

    private OpenCvCamera controlHubCam;
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;

    @Override
    public void runOpMode() {
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
        @Override
        public Mat processFrame(Mat input) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(input, hsvFrame, Imgproc.COLOR_BGR2HSV);

            // HSV ranges for purple and green
            Scalar lowerPurple = new Scalar(122, 0, 90);
            Scalar upperPurple = new Scalar(166, 229, 226);
            Scalar lowerGreen = new Scalar(67, 50, 0);
            Scalar upperGreen = new Scalar(91, 255, 255);

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

            MatOfPoint largestContour = findLargestContour(contours);
            if (largestContour != null) {
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);

                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                double offsetX = cX - (CAMERA_WIDTH / 2.0);
                String label = "OffsetX: " + String.format("%.0f", offsetX) + " px";
                Imgproc.putText(input, label, new Point(cX + 10, cY + 20),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);
            }
            return input;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }
            return largestContour;
        }
    }
}
