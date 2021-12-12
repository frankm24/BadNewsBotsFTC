package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvViewport;
import org.opencv.core.Core;
@TeleOp(group = "Testing")
public class CameraTest extends LinearOpMode {

    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        camera.showFpsMeterOnViewport(true);
        TestPipeline pipeline = new TestPipeline();
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                telemetry.addLine("Camera stream initialized");
                telemetry.update();
            }
            @Override
            public void onError(int errorCode)
            {
                throw new OpenCvCameraException("Could not open camera device.");
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Element Position", pipeline.getElementPosition());
            telemetry.update();
            sleep(50);
        }
    }

    public static class TestPipeline extends OpenCvPipeline {

        public enum ElementPosition {
            LEFT,
            CENTER,
            RIGHT,
            NONE
        }
        //Configure the points for the rectangle areas for detecting saturation values
        Point Left1 = new Point(100, 100);
        Point Left2 = new Point(250, 250);
        Rect LeftRect = new Rect(Left1, Left2);

        Point Center1 = new Point(300, 300);
        Point Center2 = new Point(450, 450);
        Rect CenterRect = new Rect(Center1, Center2);

        Point Right1 = new Point(450, 450);
        Point Right2 = new Point(600, 600);
        Rect RightRect = new Rect(Right1, Right2);


        Scalar draw_color = new Scalar(255, 0, 0);

        Scalar min = new Scalar(0, 0, 0);
        Scalar max = new Scalar(255, 255, 255);

        // Notice this is declared as an instance variable (and re-used), not a local variable
        Mat LeftMat;
        Mat CenterMat;
        Mat RightMat;

        int leftAvg;
        int centerAvg;
        int rightAvg;

        volatile ElementPosition position = ElementPosition.NONE;

        @Override
        public void init(Mat firstFrame) {
            LeftMat = firstFrame.submat(LeftRect);
            CenterMat = firstFrame.submat(CenterRect);
            RightMat = firstFrame.submat(RightRect);
            //Imgcodecs.imwrite("/root/image", firstFrame);
        }

        @Override
        public Mat processFrame(Mat input) {
            leftAvg = (int) Core.mean(LeftMat).val[1];
            centerAvg = (int) Core.mean(CenterMat).val[1];
            rightAvg = (int) Core.mean(RightMat).val[1];

            int avg = Math.max(leftAvg, Math.max(centerAvg, rightAvg));

            if (avg == leftAvg) {
               position = ElementPosition.LEFT;
            } else if (avg == centerAvg) {
               position = ElementPosition.CENTER;
            } else {
               position = ElementPosition.RIGHT;
            }

            //both methods need to be tried, range is probably better
            //Imgproc.adaptiveThreshold(input, processed, Imgproc.ADAPTIVE_THRESH_MEAN_C,);
            Imgproc.rectangle(input, LeftRect, draw_color, 10);
            Imgproc.rectangle(input, CenterRect, draw_color, 10);
            Imgproc.rectangle(input, RightRect, draw_color, 10);
            return input;
        }
        public ElementPosition getElementPosition() {
            return position;
        }
    }
}

