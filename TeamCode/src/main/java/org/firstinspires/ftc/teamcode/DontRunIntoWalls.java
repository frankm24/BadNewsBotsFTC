package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.Speed;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.PipelineRecordingParameters;

public class DontRunIntoWalls  extends LinearOpMode {
    // Elapsed time
    private ElapsedTime runtime = new ElapsedTime();

    // Declare motors
    private DcMotor back_left;
    private DcMotor front_left;
    private DcMotor back_right;
    private DcMotor front_right;
    private DcMotor[] motors = {back_left, front_left, front_right, back_right};

    private Servo arm;
    private Servo door;

    private DcMotorEx intake;
    private DcMotorEx carousel;

    private BNO055IMU imu;
    private Rev2mDistanceSensor front_tof;
    private Servo pusher;
    private OpenCvWebcam camera;

    float SpeedMultiplier = 0.5f; //scale movement speed

    public void setMotorPower(double power) {
        for (DcMotor motor : motors) {
            motor.setPower(power);
        }
    }
    public void setMotorPowerControllerVector(double LeftStickX, double LeftStickY, double RightStickX) {
        LeftStickX *= SpeedMultiplier;
        LeftStickY *= SpeedMultiplier;
        RightStickX *= SpeedMultiplier;
        double denominator = Math.max(Math.abs(LeftStickY) + Math.abs(LeftStickX) + Math.abs(RightStickX), 1);
        double front_leftPower = (LeftStickY + LeftStickX + RightStickX) / denominator;
        double back_leftPower = (LeftStickY - LeftStickX + RightStickX) / denominator;
        double front_rightPower = (LeftStickY - LeftStickX - RightStickX) / denominator;
        double back_rightPower = (LeftStickY + LeftStickX - RightStickX) / denominator;
        front_left.setPower(front_leftPower);
        back_left.setPower(back_leftPower);
        front_right.setPower(front_rightPower);
        back_right.setPower(back_rightPower);
    }

    @Override
    public void runOpMode() {
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        try {
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            carousel = hardwareMap.get(DcMotorEx.class, "duck");
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

        // OpenCV begins here
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        camera.showFpsMeterOnViewport(true);
        RecordingPipeline pipeline = new RecordingPipeline();
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

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();  // Wait for play button to be pressed
        runtime.reset();

        while (opModeIsActive()) {
            double d = front_tof.getDistance(DistanceUnit.METER);
            if (d < 1.00) {
                setMotorPowerControllerVector(0, 0, 0.5);
                sleep(500);
                setMotorPowerControllerVector(0, 0,0);
            } else {
                setMotorPowerControllerVector(0, 0.5, 0);
            }
            // Fill in with code for driving, turning using control loop
        }
    }

    public class RecordingPipeline extends OpenCvPipeline {
        final boolean record = false;

        @Override
        public Mat processFrame(Mat input) {
            return input;
        }

        @Override
        public void init(Mat input) {
            if (record) {
                camera.startRecordingPipeline(
                        new PipelineRecordingParameters.Builder()
                                .setBitrate(4, PipelineRecordingParameters.BitrateUnits.Mbps)
                                .setEncoder(PipelineRecordingParameters.Encoder.H264)
                                .setOutputFormat(PipelineRecordingParameters.OutputFormat.MPEG_4)
                                .setFrameRate(30)
                                .setPath(Environment.getExternalStorageDirectory() + "/pipeline_rec_" + ".mp4")
                                .build());
            }
        }
    }
}
