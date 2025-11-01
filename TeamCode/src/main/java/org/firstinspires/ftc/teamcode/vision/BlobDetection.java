package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;

public class BlobDetection extends OpenCvPipeline {
    private static final int CAMERA_WIDTH = 1920;
    private static final int CAMERA_CENTER_X = CAMERA_WIDTH / 2;

    private static volatile double offsetX = 0;

    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);

        Scalar lowerPurple = new Scalar(122, 0, 90);
        Scalar upperPurple = new Scalar(166, 229, 226);
        Scalar lowerGreen = new Scalar(67, 50, 0);
        Scalar upperGreen = new Scalar(91, 255, 255);

        Mat purpleMask = new Mat();
        Mat greenMask = new Mat();
        Core.inRange(hsv, lowerPurple, upperPurple, purpleMask);
        Core.inRange(hsv, lowerGreen, upperGreen, greenMask);

        Mat combinedMask = new Mat();
        Core.bitwise_or(purpleMask, greenMask, combinedMask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(15, 15));
        Imgproc.morphologyEx(combinedMask, combinedMask, Imgproc.MORPH_CLOSE, kernel);
        Imgproc.morphologyEx(combinedMask, combinedMask, Imgproc.MORPH_OPEN, kernel);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(combinedMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double bestArea = 0;
        Rect bestRect = null;

        for (MatOfPoint contour : contours) {
            Rect rect = Imgproc.boundingRect(contour);
            if (rect.area() > 1000 && rect.area() > bestArea) {
                bestArea = rect.area();
                bestRect = rect;
            }
        }

        if (bestRect != null) {
            Imgproc.rectangle(input, bestRect, new Scalar(0, 255, 0), 3);
            double blobCenterX = bestRect.x + bestRect.width / 2.0;
            offsetX = blobCenterX - CAMERA_CENTER_X;
            Imgproc.circle(input, new Point(blobCenterX, bestRect.y + bestRect.height / 2.0),
                    6, new Scalar(255, 0, 0), -1);
            Imgproc.putText(input, "Offset: " + (int) offsetX + " px",
                    new Point(bestRect.x, bestRect.y - 10),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, new Scalar(255, 255, 255), 2);
        } else {
            offsetX = 0;
        }

        hsv.release();
        purpleMask.release();
        greenMask.release();
        combinedMask.release();
        hierarchy.release();

        return input;
    }

    public static double getOffsetX() {
        return offsetX;
    }
}
