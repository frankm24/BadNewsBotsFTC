package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
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

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
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
    private OpenCvCamera camera;

    private Rev2mDistanceSensor front_tof;
    private ModernRoboticsI2cRangeSensor mr_sensor;

    private UltrasonicSensor front_center_ultrasonic;
    private List<UltrasonicSensor> ultrasonicSensors;
    private List<Pose2d> initial_points;
    private Map map;

    private SampleMecanumDrive drive;

    private FtcDashboard ftcDashboard;

    float SpeedMultiplier = 0.5f; //scale movement speed

    double bot_width = 13.5; //in
    Pose2d startPose = new Pose2d();

    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/detect.tflite";
    private static final String TFOD_MODEL_LABELS = "/sdcard/FIRST/labelmap.txt";
    private String[] labels = {};
    private TFObjectDetector tfod;
    VuforiaLocalizer vuforia = null;
    // Function to convert ArrayList<String> to String[]
    private String[] getStringArray(ArrayList<String> arr) {
        // declaration and initialize String Array
        String str[] = new String[arr.size()];

        // Convert ArrayList to object array
        Object[] objArr = arr.toArray();

        // Iterating and converting to String
        int i = 0;
        for (Object obj : objArr) {
            str[i++] = (String)obj;
        }

        return str;
    }
    private void readLabels() {
        ArrayList<String> labelList = new ArrayList<>();

        // try to read in the the labels.
        try (BufferedReader br = new BufferedReader(new FileReader(TFOD_MODEL_LABELS))) {
            int index = 0;
            while (br.ready()) {
                // skip the first row of the labelmap.txt file.
                // if you look at the TFOD Android example project (https://github.com/tensorflow/examples/tree/master/lite/examples/object_detection/android)
                // you will see that the labels for the inference model are actually extracted (as metadata) from the .tflite model file
                // instead of from the labelmap.txt file. if you build and run that example project, you'll see that
                // the label list begins with the label "person" and does not include the first line of the labelmap.txt file ("???").
                // i suspect that the first line of the labelmap.txt file might be reserved for some future metadata schema
                // (or that the generated label map file is incorrect).
                // for now, skip the first line of the label map text file so that your label list is in sync with the embedded label list in the .tflite model.
                if(index == 0) {
                    // skip first line.
                    br.readLine();
                } else {
                    labelList.add(br.readLine());
                }
                index++;
            }
        } catch (Exception e) {
            telemetry.addData("Exception", e.getLocalizedMessage());
            telemetry.update();
        }

        if (labelList.size() > 0) {
            labels = getStringArray(labelList);
            RobotLog.vv("readLabels()", "%d labels read.", labels.length);
            for (String label : labels) {
                RobotLog.vv("readLabels()", " " + label);
            }
        } else {
            RobotLog.vv("readLabels()", "No labels read!");
        }
    }


    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, labels);
    }

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
        return new ArrayList<>(readings);
    }
    private List<Vector2d> convertReadingsToPoints(List<Double> readings) {
        /*
        if the sensor is facing forward, add reading to x
        if the sensor is facing right, add reading to y
        if the sensor is facing backward, add -1 * reading to x
        if the sensor is facing left, add -1 * reading to y
         */
        List<Vector2d> points = new ArrayList<>();
        for (int i = 0; i < readings.size(); i++) {
            UltrasonicSensor sensor = ultrasonicSensors.get(i);
            double reading = readings.get(i);
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
            Vector2d point_no_theta = new Vector2d(point.getX(), point.getY());
            points.add(point_no_theta);
        }
        return new ArrayList<>(points);
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
        //telemetry = ftcDashboard.getTelemetry();
        telemetry.setMsTransmissionInterval(33);

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
        map = new Map(new ArrayList<>(), DistanceUnit.INCH, 0.1);
        //15.25/2 + (1 + 15/16)
        front_center_ultrasonic = new UltrasonicSensor(mr_sensor, new Pose2d());
        ultrasonicSensors = new ArrayList<>();
        ultrasonicSensors.add(front_center_ultrasonic);


        // OpenCV begins here
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);

        VuforiaLocalizer.Parameters vu_parameters = new VuforiaLocalizer.Parameters(viewportContainerIds[0]);
        vu_parameters.vuforiaLicenseKey = "AS27DuH/////AAABmTCx7YbI4khghC3lgWAjQo93WEllOCIMXs6D+me71rAjot43I8SxjF0AYT+65Zeuc+9biDnuDpRCjKWAoAa+YIwr/0BG+SXWuYWE3M/rsiwaiDE2UrkFnNVwAHHAClTI4lEEzY83m4wtLSrawB2q/OXrZ5oZNd3Pdf3gZHpv7S9QXEGpPxB8Rvwgi1rieSQIg0X2BBR1JSYRDVibN9Pymw8i/1CihRk3d84+bmlKXB9xxD9SPZgx2VRUQGFldkEllKapArv/k495zE5SKnYF6AYoYUTEx1ayO8Hrh/Ae3W7xuOc0GL7CB395oVdyamnhZrZ9zg7rGglSaYg3Fcz3aeA06OahIrcHXVB4X6Jg2Xcp";
        vu_parameters.secondsUsbPermissionTimeout = 3;
        // Uncomment this line below to use a webcam
        vu_parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(vu_parameters);
        camera = OpenCvCameraFactory.getInstance().createVuforiaPassthrough(vuforia, vu_parameters, viewportContainerIds[1]);

        RecordingPipeline pipeline = new RecordingPipeline();
        camera.setPipeline(pipeline);
        //camera.setMillisecondsPermissionTimeout(3000); // Give plenty of time for the internal code to ready to avoid errors
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                camera.showFpsMeterOnViewport(true);
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
        readLabels();
        initTfod();

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.speak("Robot Status: Initialized");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("rev_light", front_tof.getDistance(DistanceUnit.CM));
            telemetry.addData("mr_light: ", mr_sensor.cmOptical());
            telemetry.addData("mr_ultrasonic: ", mr_sensor.cmUltrasonic());
            telemetry.addData("mr_inches: ", mr_sensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
            sleep(250);
        }
        drive.setPoseEstimate(startPose);

        Pose2d prev_pose = new Pose2d();
        while (opModeIsActive()) {
            /*
            drive.updatePoseEstimate();
            Pose2d current_pose = drive.getPoseEstimate();

            double d = front_tof.getDistance(DistanceUnit.INCH); // FOV = 25 deg
            double d1 = mr_sensor.getDistance(DistanceUnit.INCH); // FOV = ? -maybe 15 deg

            List<Double> readings = getSensorReadings(DistanceUnit.INCH);
            if (!(prev_pose == current_pose)) {
                List<Vector2d> points = convertReadingsToPoints(readings);
                map.addPoints(points);
            }
            List<Vector2d> points = map.getPoints();

            if (d1 < 10) {
                setMotorPowerControllerVector(0, 0, 0.5);
                sleep(500);
                setMotorPowerControllerVector(0, 0,0);
            } else {
                setMotorPowerControllerVector(0, 0.5, 0);
            }
            telemetry.addData("ultrasonic: ", d1);
            telemetry.addData("prev. pose", prev_pose);
            telemetry.addData("pose: ", current_pose);
            telemetry.addData("Points: ", points);
            telemetry.update();
            */
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        i++;
                    }
                }
            }
            telemetry.update();
        }
        // Code after stop requested
        String fileName = Environment.getExternalStorageDirectory() + "/Pictures/map.txt";
        try {
            FileWriter fileWriter = new FileWriter(fileName);
            fileWriter.write(map.getPointsAsCSV());
            fileWriter.close();
            System.out.println("Map saved.");
        } catch (IOException e) {
            System.out.println("Error: The map save file could not be written.");
            e.printStackTrace();
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
