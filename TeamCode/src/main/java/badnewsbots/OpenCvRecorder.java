package badnewsbots;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import badnewsbots.pipelines.RecordingPipeline;

public class OpenCvRecorder {
    private final OpenCvWebcam camera;
    private final RecordingPipeline pipeline;
    private final Telemetry telemetry;

    public OpenCvRecorder(OpenCvWebcam camera, Telemetry telemetry, boolean record) {
        this.camera = camera;
        this.telemetry = telemetry;
        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        pipeline = new RecordingPipeline(camera, record);
        camera.setPipeline(pipeline);
        camera.setMillisecondsPermissionTimeout(3000); // Give plenty of time for the internal code to ready to avoid errors
    }

   public void openCameraAsync() {
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
                /*
                OpenCV Camera resolution FPS Test :
                    1280x720: 7.5 FPS (Theoretical max 55)
                    640x360: 30 FPS (Theoretical max 200)
                    480x360:
                */
                telemetry.addLine("Camera stream initialized");
                FtcDashboard.getInstance().startCameraStream(camera, 30);
                telemetry.update();
            }
            @Override
            public void onError(int errorCode) {
                throw new OpenCvCameraException("Could not open camera device. Error code: " + errorCode) ;
                // This will be called if the camera could not be opened
            }
        });
    }
    public boolean isRecording() {return pipeline.record;}
}
