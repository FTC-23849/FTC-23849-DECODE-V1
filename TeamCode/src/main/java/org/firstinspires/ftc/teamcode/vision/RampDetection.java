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
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        telemetry.addLine("Ramp Detection Initialized");
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.getPythonOutput() != null) {
            double[] pythonOutputs = result.getPythonOutput();
            if (pythonOutputs.length >= 9) {
                StringBuilder pattern = new StringBuilder();
                for (int i = 0; i < pythonOutputs.length; i++) {
                    int v = (int) pythonOutputs[i];
                    switch (v) {
                        case 1: pattern.append("G "); break;
                        case 2: pattern.append("P "); break;
                        default: pattern.append("_ "); break;
                    }
                }
                telemetry.addData("Ramp Pattern",  pattern);
            }
        } else {
            telemetry.addData("Ramp Pattern", "Null");
        }
        telemetry.update();
    }
}
