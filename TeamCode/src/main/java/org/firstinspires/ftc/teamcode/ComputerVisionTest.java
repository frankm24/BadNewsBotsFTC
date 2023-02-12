package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;

import badnewsbots.pipelines.SignalSleevePipeline;

@Autonomous
public class ComputerVisionTest extends LinearOpMode {

    OpenCvWebcam camera;

    @Override
    public void runOpMode() {

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        SignalSleevePipeline pipeline = new SignalSleevePipeline();
        initOpenCV(pipeline);

        while (!isStarted() && !isStopRequested()) {
            SignalSleevePipeline.ConeOrientation coneOrientation = pipeline.getConeOrientation();
            double[] colorFilterAverages = pipeline.getFilterAverages();
            telemetry.addData("Status",  "Initialized");
            telemetry.addData("Filter averages: ", Arrays.toString(colorFilterAverages));
            telemetry.addData("Cone orientation", coneOrientation);
            telemetry.addData("FPS", camera.getFps());
            telemetry.update();
            idle();

        }
    }

    private void initOpenCV(OpenCvPipeline pipeline) {
        camera.setPipeline(pipeline);
        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 360);
            }

            @Override
            public void onError(int errorCode) {
                throw new OpenCvCameraException("oof" + errorCode);
            }
        });
    }
}
