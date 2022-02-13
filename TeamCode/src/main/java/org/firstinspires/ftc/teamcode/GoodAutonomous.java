package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.SmartTerrain;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraBase;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvViewport;
import org.opencv.core.Core;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;

@TeleOp(group = "Testing")
public class GoodAutonomous extends LinearOpMode {
    private DcMotor back_left;
    private DcMotor front_left;
    private DcMotor back_right;
    private DcMotor front_right;
    private DcMotor[] motors = {back_left, front_left, front_right, back_right};

    private Servo arm;
    private Servo door;

    private DcMotorEx intake;

    private BNO055IMU imu;
    private Rev2mDistanceSensor front_tof;
    private Servo pusher;

    // settings
    final int FlywheelTargetSpeed = 1500; // ticks per second

    SampleMecanumDrive drive;
    TrajectorySequence part1;
    TrajectorySequence part2;

    @Override
    public void runOpMode() {
        // hardwareMap.get stuff
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        try {
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            front_tof = hardwareMap.get(Rev2mDistanceSensor.class, "front_tof");
        } catch (IllegalArgumentException e) {
            telemetry.addLine("expansion hub not working");
            telemetry.update();
        }
        arm = hardwareMap.get(Servo.class, "arm");
        door = hardwareMap.get(Servo.class, "door");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        // Reverse the motors that runs backwards (LEFT SIDE)
        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu.initialize(parameters);

        // Code for the autonomous driving task
        drive = new SampleMecanumDrive(hardwareMap);

        part1 = drive.trajectorySequenceBuilder(new Pose2d(0,0,0))
                .strafeRight(23.75)
                .back(22)
                .build();
        part2 = drive.trajectorySequenceBuilder(new Pose2d(0,0,0))
                .forward(27)
                .turn(Math.toRadians(53.1))
                .strafeRight(1)
                .forward(47)
                .build();

        // OpenCV begins here
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        camera.showFpsMeterOnViewport(true);
        TestPipeline pipeline = new TestPipeline();
        camera.setPipeline(pipeline);
        camera.setMillisecondsPermissionTimeout(3000); // Give plenty of time for the internal code to ready to avoid errors
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
                throw new OpenCvCameraException("Could not open camera device. Error code: " + errorCode) ;
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        while (!isStarted() && !isStopRequested()) {
            int[] values = pipeline.getValues();
            telemetry.addData("Element Position", pipeline.getElementPosition());
            telemetry.addData("Values:", Arrays.toString(values));
            telemetry.addData("FPS: ", camera.getFps());
            telemetry.update();
            sleep(100);
        }
        //  POST-Init
        camera.stopStreaming();

        TestPipeline.ElementPosition element_position = pipeline.getElementPosition();

        telemetry.addData("Final Determined Element Position:", element_position);
        telemetry.update();

        switch (element_position)
        {
            case LEFT:
            {
                // down
                break;
            }

            case CENTER:
            {
                // middle
                break;
            }

            case RIGHT:
            {
                // up
                drive.setPoseEstimate(new Pose2d(0, 0, 0));
                drive.followTrajectorySequence(part1);
                arm.setPosition(0.5);
                sleep(3000);
                arm.setPosition(-0.35);
                break;
            }
        }
    }
    // Contains the code for processing the camera feed
    public static class TestPipeline extends OpenCvPipeline {

        enum ElementPosition {
            LEFT,
            CENTER,
            RIGHT,
            NONE
        }
        //Configure the points for the rectangle areas for detecting saturation values
        Point Left1 = new Point(100, 400);
        Point Left2 = new Point(300, 600);
        Rect LeftRect = new Rect(Left1, Left2);

        Point Center1 = new Point(450, 400);
        Point Center2 = new Point(650, 600);
        Rect CenterRect = new Rect(Center1, Center2);

        Point Right1 = new Point(1000, 400);
        Point Right2 = new Point(1200, 600);
        Rect RightRect = new Rect(Right1, Right2);

        Scalar draw_color = new Scalar(255, 0, 0);

        boolean returnResult = true;

        Scalar min = new Scalar(45, 30, 50);
        Scalar max = new Scalar(70, 255, 255);

        // Notice this is declared as an instance variable (and re-used), not a local variable
        Mat hsvImage;
        Mat filteredImage;
        Mat LeftMat;
        Mat CenterMat;
        Mat RightMat;

        int leftAvg;
        int centerAvg;
        int rightAvg;

        volatile ElementPosition position = ElementPosition.NONE;

        @Override
        public void init(Mat firstFrame) {
            hsvImage = new Mat();
            filteredImage = new Mat();
            Imgproc.cvtColor(firstFrame, hsvImage, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsvImage, min, max, filteredImage);
            LeftMat = filteredImage.submat(LeftRect);
            CenterMat = filteredImage.submat(CenterRect);
            RightMat = filteredImage.submat(RightRect);
            String fileName = Environment.getExternalStorageDirectory() + "/Pictures/image.png";
            Imgcodecs.imwrite(fileName, firstFrame);
        }
        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsvImage, min, max, filteredImage);
            leftAvg = (int) Core.mean(LeftMat).val[0];
            centerAvg = (int) Core.mean(CenterMat).val[0];
            rightAvg = (int) Core.mean(RightMat).val[0];

            //int avg = Math.max(leftAvg, Math.max(centerAvg, rightAvg));

            int mostGreen = Math.max(leftAvg, Math.max(centerAvg, rightAvg));

            if (mostGreen == leftAvg) {
               position = ElementPosition.LEFT;
            } else if (mostGreen == centerAvg) {
               position = ElementPosition.CENTER;
            } else {
               position = ElementPosition.RIGHT;
            }
            //both methods need to be tried, range is probably better
            //Imgproc.adaptiveThreshold(input, processed, Imgproc.ADAPTIVE_THRESH_MEAN_C,);
            Imgproc.rectangle(filteredImage, LeftRect, draw_color, 10);
            Imgproc.rectangle(filteredImage, CenterRect, draw_color, 10);
            Imgproc.rectangle(filteredImage, RightRect, draw_color, 10);
            if (returnResult == true) {
                return filteredImage;
            } else {
                return input;
            }
        }
        public ElementPosition getElementPosition() {
            return position;
        }
        public int[] getValues() {
            return new int[] {leftAvg, centerAvg, rightAvg};
        }
    }

}

