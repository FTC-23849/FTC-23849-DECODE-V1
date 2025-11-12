package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name = "Ramp Detection")
public class RampDetection extends OpMode {
    Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        telemetry.addLine("Ramp Detection Initialized");
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.getPythonOutput() != null) {
            double[] pythonOutputs = result.getPythonOutput();

            if (pythonOutputs.length > 0) {
                StringBuilder pattern = new StringBuilder();
                int ballCount = 0;

                for (double output : pythonOutputs) {
                    int v = (int) output;
                    if (v == 1) {
                        pattern.append("G ");
                        ballCount++;
                    } else if (v == 2) {
                        pattern.append("P ");
                        ballCount++;
                    }
                }

                telemetry.addData("Balls Detected", ballCount);
                telemetry.addData("Pattern", pattern.toString().trim());
            } else {
                telemetry.addData("Balls Detected", 0);
                telemetry.addData("Pattern", "None");
            }
        } else {
            telemetry.addData("Balls Detected", "N/A");
            telemetry.addData("Pattern", "Null");
        }
        telemetry.update();
    }
}
