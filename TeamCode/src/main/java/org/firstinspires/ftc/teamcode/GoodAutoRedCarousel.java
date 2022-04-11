package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

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
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;

@Disabled
@Autonomous
public class GoodAutoRedCarousel extends LinearOpMode {
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
    TrajectorySequence highGoal;
    TrajectorySequence highGoal2;
    TrajectorySequence midGoal;
    TrajectorySequence midGoal2;
    TrajectorySequence lowGoal;
    TrajectorySequence lowGoal2;

    OpenCvWebcam camera;

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

        Pose2d redStartPoseCarousel = new Pose2d(-35, -62.25, Math.toRadians(90));

        highGoal = drive.trajectorySequenceBuilder(redStartPoseCarousel)
                .strafeLeft(23.75)
                .forward(21)
                .build();
        highGoal2 = drive.trajectorySequenceBuilder(new Pose2d(-11.75,-41.25, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(12, -54, Math.toRadians(180)))
                .back(48)
                .build();
        midGoal = drive.trajectorySequenceBuilder(redStartPoseCarousel)
                .strafeLeft(23.75)
                .forward(17)
                .build();
        midGoal2 = drive.trajectorySequenceBuilder(new Pose2d(-11.75,-46.25, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(12, -52, Math.toRadians(180)))
                .back(48)
                .build();
        lowGoal = drive.trajectorySequenceBuilder(redStartPoseCarousel)
                .strafeLeft(23.75)
                .forward(14.5)
                .addTemporalMarker(0, () -> {
                    door.setPosition(0);
                })
                .addTemporalMarker(0.1, () -> {
                    arm.setPosition(0.8);
                })
                .addTemporalMarker(2.5, () -> {
                    door.setPosition(0.65);
                })
                .waitSeconds(1)
                .addTemporalMarker(4, () -> {
                    arm.setPosition(0.2);
                })
                .lineToSplineHeading(new Pose2d(-58, -58, Math.toRadians(180)))
                .waitSeconds(3)
                .strafeRight(24)
                .build();
        //lowGoal2 = drive.trajectorySequenceBuilder(new Pose2d(-11.75, -47.75, Math.toRadians(90)))

        // OpenCV begins here
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        camera.showFpsMeterOnViewport(true);
        TestPipeline pipeline = new TestPipeline();
        camera.setPipeline(pipeline);
        camera.setMillisecondsPermissionTimeout(3000); // Give plenty of time for the internal code to ready to avoid errors
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                telemetry.addLine("Camera stream initialized");
                telemetry.update();
            }
            @Override
            public void onError(int errorCode) {
                throw new OpenCvCameraException("Could not open camera device. Error code: " + errorCode) ;
                // This will be called if the camera could not be opened
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

        switch (element_position) {
            case LEFT: {
                // low
                drive.setPoseEstimate(redStartPoseCarousel);
                drive.followTrajectorySequence(lowGoal);
                requestOpModeStop();
            }
            case CENTER: {
                // mid
                requestOpModeStop();
            }
            case RIGHT: {
                // up
                requestOpModeStop();
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
        Point Left1 = new Point(0, 520);
        Point Left2 = new Point(200, 720);
        Rect LeftRect = new Rect(Left1, Left2);

        Point Center1 = new Point(500, 520);
        Point Center2 = new Point(700, 720);
        Rect CenterRect = new Rect(Center1, Center2);

        Point Right1 = new Point(1000, 520);
        Point Right2 = new Point(1200, 720);
        Rect RightRect = new Rect(Right1, Right2);

        Rect[] rectangles = {LeftRect, CenterRect, RightRect};

        // Record or not?
        boolean record = false;
        boolean returnResult = true;

        Scalar noDraw = new Scalar(0, 0, 0);
        Scalar Draw = new Scalar(255, 0, 0);
        Scalar red = new Scalar(255, 0, 0);

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

            Imgproc.rectangle(firstFrame, LeftRect, red, 10);
            Imgproc.rectangle(firstFrame, CenterRect, red, 10);
            Imgproc.rectangle(firstFrame, RightRect, red, 10);

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

            int mostGreen = Math.max(leftAvg, Math.max(centerAvg, rightAvg));

            //Scalar left_draw_color = Draw;
            //Scalar center_draw_color = Draw;
            //Scalar right_draw_color = Draw;

            if (mostGreen == leftAvg) {
                position = ElementPosition.LEFT;
                //left_draw_color = Draw;
            } else if (mostGreen == centerAvg) {
                position = ElementPosition.CENTER;
                //center_draw_color = Draw;
            } else {
                position = ElementPosition.RIGHT;
                //right_draw_color = Draw;
            }

            //Imgproc.rectangle(filteredImage, LeftRect, left_draw_color, 10);
            //Imgproc.rectangle(filteredImage, CenterRect, center_draw_color, 10);
            //Imgproc.rectangle(filteredImage, RightRect, right_draw_color, 10);

            if (returnResult) {
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

