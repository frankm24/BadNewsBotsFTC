package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.slam.Map;
import org.firstinspires.ftc.teamcode.slam.UltrasonicSensor;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.PipelineRecordingParameters;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

@Autonomous
public class DontRunIntoWalls  extends LinearOpMode {
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
    private Servo pusher;
    private OpenCvWebcam camera;

    private Rev2mDistanceSensor front_tof;
    private ModernRoboticsI2cRangeSensor mr_sensor;

    private UltrasonicSensor front_center_ultrasonic;
    private List<UltrasonicSensor> ultrasonicSensors;
    private List<Pose2d> initial_points;
    private Map map;

    private SampleMecanumDrive drive;

    private FtcDashboard ftcDashboard;

    float SpeedMultiplier = 0.5f; //scale movement speed
    Pose2d startPose = new Pose2d();

    private void setMotorPower(double power) {
        for (DcMotor motor : motors) {
            motor.setPower(power);
        }
    }

    private void setMotorPowerControllerVector(double LeftStickX, double LeftStickY, double RightStickX) {
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

    private List<Double> getSensorReadings(DistanceUnit unit) {
        List<Double> readings = new ArrayList<>();
        for (UltrasonicSensor sensor : ultrasonicSensors) {
            readings.add(sensor.getSensor().getDistance(unit));
        }
        return readings;
    }
    private List<Pose2d> convertReadingsToPoints(List<Double> readings) {
        /*
        if the sensor is facing forward, add reading to x
        if the sensor is facing right, add reading to y
        if the sensor is facing backward, add -1 * reading to x
        if the sensor is facing left, add -1 * reading to y
         */
        Iterator<UltrasonicSensor> sensor_iter = ultrasonicSensors.iterator();
        Iterator<Double> readings_iter = readings.iterator();
        List<Pose2d> points = new ArrayList<>();
        while (sensor_iter.hasNext() && readings_iter.hasNext()) {
            UltrasonicSensor sensor = sensor_iter.next();
            double reading = readings_iter.next();
            // If reading is out of sensor range (shows up as max double value), skip!
            if (reading == DistanceUnit.infinity) {
                continue;
            }
            /*
            the following step is not done using normal trig angles, 0 radians is forward, not right
            this is to match the x axis being forward and the y axis being left/right according to
            the robot's localizer

             */
            double sensor_heading = sensor.getPosition().getHeading();
            Pose2d reading_pose = new Pose2d(Math.cos(sensor_heading) * reading, Math.sin(sensor_heading) * reading);
            Pose2d point = drive.getPoseEstimate().plus(sensor.getPosition()).plus(reading_pose);
            points.add(point);
        }
        return points;
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
            mr_sensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "mr_sensor");
        } catch (IllegalArgumentException e) {
            telemetry.addLine("expansion hub not working");
            telemetry.update();
        }
        arm = hardwareMap.get(Servo.class, "arm");
        door = hardwareMap.get(Servo.class, "door");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        ftcDashboard = FtcDashboard.getInstance();
        telemetry = ftcDashboard.getTelemetry();

        // Reverse the motors that runs backwards (LEFT SIDE)
        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu.initialize(parameters);

        drive = new SampleMecanumDrive(hardwareMap);
        map = new Map();

        front_center_ultrasonic = new UltrasonicSensor(mr_sensor, new Pose2d());
        ultrasonicSensors.add(front_center_ultrasonic);

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
                FtcDashboard.getInstance().startCameraStream(camera, 30);
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

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("rev_light", front_tof.getDistance(DistanceUnit.CM));
            telemetry.addData("mr_light: ", mr_sensor.cmOptical());
            telemetry.addData("mr_ultrasonic: ", mr_sensor.cmUltrasonic());
            telemetry.update();
            sleep(250);
        }
        drive.setPoseEstimate(startPose);

        while (opModeIsActive()) {
            double d = front_tof.getDistance(DistanceUnit.INCH); // FOV = 25 deg
            double d1 = mr_sensor.getDistance(DistanceUnit.INCH); // FOV = ? -maybe 15 deg
            List<Double> readings = getSensorReadings(DistanceUnit.INCH);
            List<Pose2d> points = convertReadingsToPoints(readings);
            map.addPoints(points);
            if (d < 1.00) {
                setMotorPowerControllerVector(0, 0, 0.5);
                sleep(500);
                setMotorPowerControllerVector(0, 0,0);
            } else {
                setMotorPowerControllerVector(0, 0.5, 0);
            }
            telemetry.addData("rev 2m tof: ", d);
            telemetry.addData("modern robotics tof: ", d1);
            telemetry.addData("pose: ", drive.getPoseEstimate());
            telemetry.addData("Points: ", map.getPoints());
            telemetry.update();
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
