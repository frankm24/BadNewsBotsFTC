package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import badnewsbots.slam.Map;
import badnewsbots.slam.UltrasonicSensor;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.PipelineRecordingParameters;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import badnewsbots.ml.FTC_TFLiteObjectDetector;
import badnewsbots.ml.YOLODarknetObjectDetector;
import badnewsbots.hardware.GamepadEx;
import badnewsbots.robots.AutonomousTestingBot;

@Disabled
@TeleOp
public class DontRunIntoWalls extends LinearOpMode {
    // Robot object
    AutonomousTestingBot robot;
    FTC_TFLiteObjectDetector ftcTfLiteObjectDetector;

    GamepadEx smartGamepad;

    FtcDashboard ftcDashboard;

    private List<Pose2d> initial_points;
    private Map map;

    float speedMultiplier = 0.5f; //scale movement speed
    boolean openCVStreaming = false;
    boolean tfodStreaming = false;

    double botWidth = 13.5; //in
    double webcam_focalLength = 1079.21311;
    double human_knownWidth = 15.25; //in, (average for men + average for women) / 2
    //Equation: Focal length = Detected obj. width (px) * Distance from camera (in.) / Object known width (in.)

    enum OperationMode {
        AUTONOMOUS,
        DATA_COLLECTION,
        TFOD_TEST,
        OPENCV_DNN_TEST
    }

    Pose2d startPose = new Pose2d();

    private List<Double> getSensorReadings(DistanceUnit unit) {
        List<Double> readings = new ArrayList<>();
        for (UltrasonicSensor sensor : robot.ultrasonicSensors) {
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
            UltrasonicSensor sensor = robot.ultrasonicSensors.get(i);
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
            Pose2d point = robot.drive.getPoseEstimate().plus(sensor.getPosition()).plus(reading_pose);
            Vector2d point_no_theta = new Vector2d(point.getX(), point.getY());
            points.add(point_no_theta);
        }
        return new ArrayList<>(points);
    }

    @Override
    public void runOpMode() {
        robot = new AutonomousTestingBot(this);
        ftcTfLiteObjectDetector = new FTC_TFLiteObjectDetector(this);

        ftcDashboard = FtcDashboard.getInstance();

        map = new Map(new ArrayList<>(), DistanceUnit.INCH, 0.1);
        //15.25/2 + (1 + 15/16)
        telemetry.addLine("Camera stream loading...");
        telemetry.update();

        //ObjectDetectPipeline pipeline = new ObjectDetectPipeline();
        //robot.camera.setMillisecondsPermissionTimeout(3000); // Give plenty of time for the internal code to ready to avoid errors
        //robot.camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.SOFTWARE);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //robot.camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        /*
        robot.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                telemetry.addLine("Camera stream initialized");
                telemetry.update();
            }
            @Override
            public void onError(int errorCode) {
                throw new OpenCvCameraException("Could not open camera device. Error code: " + errorCode) ;
            }
        });
        */
        smartGamepad = new GamepadEx(gamepad1);
        OperationMode operationMode = OperationMode.DATA_COLLECTION;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            //telemetry.addData("rev_light", front_tof.getDistance(DistanceUnit.CM));
            //telemetry.addData("mr_light: ", robot.mr_sensor.cmOptical());
            //telemetry.addData("mr_ultrasonic: ", robot.mr_sensor.cmUltrasonic());
            //telemetry.addData("mr_inches: ", robot.mr_sensor.getDistance(DistanceUnit.INCH));
            //telemetry.addData("FPS", robot.camera.getFps());
            telemetry.update();
            sleep(250);
        }

        robot.drive.setPoseEstimate(startPose);

        Pose2d prev_pose = new Pose2d();
        Recognition prev_target = null;

        TFObjectDetector tfod = ftcTfLiteObjectDetector.getTfod();
        VuforiaLocalizer vuforia = ftcTfLiteObjectDetector.getVuforia();

        while (opModeIsActive()) {
            smartGamepad.update();

            float LeftStickY = -1 * smartGamepad.left_stick_y * speedMultiplier;
            float LeftStickX = smartGamepad.left_stick_x * speedMultiplier;
            float RightStickY = -1 * smartGamepad.right_stick_y * speedMultiplier;
            float RightStickX = smartGamepad.right_stick_x * speedMultiplier;

            if (smartGamepad.dpad_up_pressed) {operationMode = OperationMode.DATA_COLLECTION;}
            if (smartGamepad.dpad_left_pressed) {operationMode = OperationMode.AUTONOMOUS;}
            if (smartGamepad.dpad_right_pressed) {operationMode = OperationMode.TFOD_TEST;}
            if (smartGamepad.dpad_down_pressed) {operationMode = OperationMode.OPENCV_DNN_TEST;}

            robot.drive.update();
            Pose2d current_pose = robot.drive.getPoseEstimate();

            //double d = front_tof.getDistance(DistanceUnit.INCH); // FOV = 25 deg
            double d1 = robot.mr_sensor.getDistance(DistanceUnit.INCH); // FOV = ? -maybe 15 deg

            List<Double> readings = getSensorReadings(DistanceUnit.INCH);
            if (operationMode == OperationMode.AUTONOMOUS) {
                if (d1 < 10) {
                    robot.setDriveMotorPowerControllerVector(0, 0, 0.5, speedMultiplier);
                    sleep(500);
                    robot.setDriveMotorPowerControllerVector(0, 0, 0, speedMultiplier);
                } else {
                    robot.setDriveMotorPowerControllerVector(0, 0.5, 0, speedMultiplier);
                }
            }
            else if (operationMode == OperationMode.DATA_COLLECTION) {
                if (!(prev_pose == current_pose)) {
                    List<Vector2d> points = convertReadingsToPoints(readings);
                    map.addPoints(points);
                }
                List<Vector2d> points = map.getPoints();

                double denominator = Math.max(Math.abs(LeftStickY) + Math.abs(LeftStickX) + Math.abs(RightStickX), 1);
                double front_leftPower = (LeftStickY + LeftStickX + RightStickX) / denominator;
                double back_leftPower = (LeftStickY - LeftStickX + RightStickX) / denominator;
                double front_rightPower = (LeftStickY - LeftStickX - RightStickX) / denominator;
                double back_rightPower = (LeftStickY + LeftStickX - RightStickX) / denominator;
                robot.front_left.setPower(front_leftPower);
                robot.back_left.setPower(back_leftPower);
                robot.front_right.setPower(front_rightPower);
                robot.back_right.setPower(back_rightPower);
            }
            else if (operationMode == OperationMode.TFOD_TEST) {
                if (!tfodStreaming) {
                    tfodStreaming = true;
                    ftcDashboard.startCameraStream(tfod, 30);
                }
                double target_angleDelta = 0;
                double target_width = 0;
                double target_distance = 0;
                List<Recognition> Recognitions = tfod.getRecognitions();
                List<Recognition> people = new ArrayList<>();
                if (Recognitions != null) {
                    for (Recognition recognition : Recognitions) {
                        if (recognition.getLabel().equals("person")) {
                            people.add(recognition);
                        }
                    }
                }
                if (people.size() > 0) {
                    Recognition target = people.get(0);
                    target_width = target.getWidth();
                    target_distance = human_knownWidth * webcam_focalLength / target_width;
                    // Obj. distance from robot (in.) = Obj. known width (in.) * Focal length constant / Detected obj. width (px)
                    target_angleDelta = -1 * target.estimateAngleToObject(AngleUnit.RADIANS);
                    //robot.drive.turn(target_angleDelta);
                    // 39 in. * 422 / 15.25
                    // focal length const. measured as: 1079.21311
                    prev_target = target;
                    telemetry.addData("Target width (px): ", target_width);
                    telemetry.addData("Target Estimated Distance (in): ", target_distance);
                    telemetry.addData("Target angle delta (rad): ", target_angleDelta);
                }
            }
            //Mat rendered_points = map.renderPointsOntoSquareImage(current_pose.vec(), 101);
            //Bitmap bitmap = Bitmap.createBitmap(101, 101, Bitmap.Config.RGB_565);
            //Utils.matToBitmap(rendered_points, bitmap);
            //ftcDashboard.sendImage(bitmap);

            //telemetry.addData("mat channels", rendered_points.channels());
            telemetry.addData("OpMode", operationMode);
            telemetry.addData("ultrasonic: ", d1);
            telemetry.addData("prev. pose", prev_pose);
            telemetry.addData("pose: ", current_pose);

            telemetry.update();
        }
        // Code after stop requested
        if (isStopRequested()) {
            map.removeDuplicatePoints();
            String fileName = Environment.getExternalStorageDirectory() + "/Pictures/map.txt";
            try {
                FileWriter fileWriter = new FileWriter(fileName);
                fileWriter.write(map.getPointsAsTXT());
                fileWriter.close();
                System.out.println("Map saved.");
            } catch (IOException e) {
                System.out.println("Error: The map save file could not be written.");
                e.printStackTrace();
            }
        }
    }

    public class ObjectDetectPipeline extends OpenCvPipeline {
        final boolean record = false;
        Mat rgbFrame = new Mat();
        YOLODarknetObjectDetector objectDetector = new YOLODarknetObjectDetector(telemetry);

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, rgbFrame, Imgproc.COLOR_RGBA2RGB);
            return objectDetector.predict(rgbFrame);
            //return input;
        }

        @Override
        public void init(Mat input) {
            if (record) {
                robot.camera.startRecordingPipeline(
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
