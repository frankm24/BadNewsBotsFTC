package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.imgcodecs.Imgcodecs;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

import badnewsbots.hardware.GamepadEx;
import badnewsbots.pipelines.SignalSleevePipeline;
import badnewsbots.robots.PowerPlayCompBot;

@Autonomous
public class PowerPlayAutonomousTest extends LinearOpMode {

    private PowerPlayCompBot robot;
    private GamepadEx smartGamepad;
    private FtcDashboard ftcDashboard;
    private SignalSleevePipeline pipeline;
    private SignalSleevePipeline.ConeOrientation coneOrientation;
    private double[] colorFilterAverages;

    private void initOpenCV(OpenCvPipeline pipeline) {
        robot.camera.setPipeline(pipeline);
        robot.camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        robot.camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        robot.camera.showFpsMeterOnViewport(true);
        robot.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                ftcDashboard.getTelemetry().addLine("Camera stream initialized");
                telemetry.update();
                robot.camera.startStreaming(640, 480);
                ftcDashboard.startCameraStream(robot.camera, 30);
            }
            @Override
            public void onError(int errorCode) {
                throw new OpenCvCameraException("Could not open camera device. Error code: " + errorCode) ;
            }
        });
    }

    @Override
    public void runOpMode() {
        robot = new PowerPlayCompBot(this);
        smartGamepad = new GamepadEx(gamepad1);
        ftcDashboard = FtcDashboard.getInstance();

        pipeline = new SignalSleevePipeline();
        initOpenCV(pipeline);

        while (!isStarted() && !isStopRequested()) {
            coneOrientation = pipeline.getConeOrientation();
            colorFilterAverages = pipeline.getFilterAverages();
            telemetry.addData("Status: ", "Initialized");
            telemetry.addData("Cone filter averages: (G, M, O)", Arrays.toString(colorFilterAverages));
            telemetry.addData("Cone orientation: ", coneOrientation);
            telemetry.addData("FPS: ", robot.camera.getFps());
            telemetry.update();
            idle();
        }

        if (coneOrientation == SignalSleevePipeline.ConeOrientation.ONE) {

        }
        if (coneOrientation == SignalSleevePipeline.ConeOrientation.TWO) {

        }
        if (coneOrientation == SignalSleevePipeline.ConeOrientation.THREE) {

        }
    }
}
