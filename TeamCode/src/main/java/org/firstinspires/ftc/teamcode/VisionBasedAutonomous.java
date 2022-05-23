package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous
public class VisionBasedAutonomous extends LinearOpMode {
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

        Pose2d startPose = new Pose2d();
        TrajectorySequence test_autonomous_path = drive.trajectorySequenceBuilder(startPose)
                .forward(32)
                .turn(Math.toRadians(-90))
                .forward(107)
                .build();
        //lowGoal2 = drive.trajectorySequenceBuilder(new Pose2d(-11.75, -47.75, Math.toRadians(90)))

        // OpenCV begins here
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        camera.showFpsMeterOnViewport(true);
        //TestPipeline pipeline = new TestPipeline();
        //camera.setPipeline(pipeline);
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
            //telemetry.addData("FPS: ", camera.getFps());
            telemetry.update();
            sleep(100);
        }
        drive.followTrajectorySequence(test_autonomous_path);
    }
    /*
    public static class TestPipeline extends OpenCvPipeline {
        // Record or not?
        final boolean record = false;
        final boolean returnResult = true;

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

            //String fileName = Environment.getExternalStorageDirectory() + "/Pictures/image.png";
            //Imgcodecs.imwrite(fileName, firstFrame);
        }
        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsvImage, min, max, filteredImage);
            leftAvg = (int) Core.mean(LeftMat).val[0];
            centerAvg = (int) Core.mean(CenterMat).val[0];
            rightAvg = (int) Core.mean(RightMat).val[0];

            int mostGreen = Math.max(leftAvg, Math.max(centerAvg, rightAvg));

            if (returnResult) {
                return filteredImage;
            } else {
                return input;
            }
        }
    }
     */
}

