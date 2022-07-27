package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamClient;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.PipelineRecordingParameters;

import java.util.Arrays;
import java.util.List;
import java.util.ListIterator;

import badnewsbots.hardware.GamepadEx;
import badnewsbots.robots.RingBot;

@TeleOp
public class RingBotControl extends LinearOpMode {
    // Robot object
    RingBot ringBot;

    // Settings
    int flywheelTargetSpeed = 1000;
    float SpeedMultiplier = 1.0f; // scale movement speed

    FtcDashboard ftcDashboard;

    boolean dataCollection = false;
    boolean flywheelOn = false;

    public enum PusherState {
        IN,
        MOVING_OUT,
        OUT,
        MOVING_IN
    }
    PusherState pusherState = PusherState.IN;
    double pusherTime = 0;

    List<RevBlinkinLedDriver.BlinkinPattern> blinkinPatterns = Arrays.asList(RevBlinkinLedDriver.BlinkinPattern.values());

    GamepadEx smartGamepad;

    public void mainLoop() {
        ListIterator<RevBlinkinLedDriver.BlinkinPattern> blinkinPatternIterator = blinkinPatterns.listIterator();
        double prevTime = 0;
        while (opModeIsActive()) {
            double currentTime = getRuntime();
            double deltaTime = currentTime - prevTime;
            smartGamepad.update();
            float LeftStickY = -1 * smartGamepad.left_stick_y * SpeedMultiplier;
            float LeftStickX = smartGamepad.left_stick_x * SpeedMultiplier;
            float RightStickY = -1 * smartGamepad.right_stick_y * SpeedMultiplier;
            float RightStickX = smartGamepad.right_stick_x * SpeedMultiplier;

            if (smartGamepad.y_pressed) {
                dataCollection = !dataCollection;
            }

            if (smartGamepad.x_pressed) {
                if (blinkinPatternIterator.hasNext()) {
                    //ringBot.led_driver.setPattern(blinkinPatternIterator.next());
                } else {
                    blinkinPatternIterator = blinkinPatterns.listIterator();
                    //ringBot.led_driver.setPattern(blinkinPatternIterator.next());
                }
            }
            if (smartGamepad.dpad_up_pressed) {
                flywheelTargetSpeed += 500;
            }
            if (smartGamepad.dpad_down_pressed) {
                flywheelTargetSpeed -= 500;
            }
            if (smartGamepad.a_pressed) {
                if (!flywheelOn) {
                    ringBot.flywheel.setVelocity(flywheelTargetSpeed);
                    flywheelOn = true;
                } else {
                    flywheelOn = false;
                    ringBot.flywheel.setVelocity(0);
                }
            }
            if (smartGamepad.right_trigger_pressed) {
                telemetry.addLine("right trigger pressed");
                if (pusherState == PusherState.IN) {
                    pusherState = PusherState.MOVING_OUT;
                    pusherTime = currentTime;
                    ringBot.pusher.setPosition(0);
                }
            }
            if (pusherState == PusherState.MOVING_OUT) {
                if (currentTime - pusherTime >= 0.460) {
                    ringBot.pusher.setPosition(0.48);
                    pusherTime = currentTime;
                    pusherState = PusherState.MOVING_IN;
                }
            }
            if (pusherState == PusherState.MOVING_IN) {
                if (currentTime - pusherTime >= 0.460) {
                    pusherState = PusherState.IN;
                }
            }
            if (smartGamepad.start_pressed) {
                if (SpeedMultiplier == 0.5f) {
                    SpeedMultiplier = 1.0f;
                } else {
                    SpeedMultiplier = 0.5f;
                }
            }
            double denominator = Math.max(Math.abs(LeftStickY) + Math.abs(LeftStickX) + Math.abs(RightStickX), 1);
            double front_leftPower = (LeftStickY + LeftStickX + RightStickX) / denominator;
            double back_leftPower = (LeftStickY - LeftStickX + RightStickX) / denominator;
            double front_rightPower = (LeftStickY - LeftStickX - RightStickX) / denominator;
            double back_rightPower = (LeftStickY + LeftStickX - RightStickX) / denominator;
            ringBot.front_left.setPower(front_leftPower);
            ringBot.back_left.setPower(back_leftPower);
            ringBot.front_right.setPower(front_rightPower);
            ringBot.back_right.setPower(back_rightPower);
            telemetry.addData("Flywheel speed: ", ringBot.flywheel.getVelocity());
            telemetry.addData("Control FPS: ", deltaTime);
            telemetry.addData("IMU Data", ringBot.imu.getAngularOrientation());
            telemetry.addData("front_tof: ", ringBot.front_tof.getDistance(DistanceUnit.INCH));
            telemetry.update();
            smartGamepad.postUpdate();
            prevTime = currentTime;
        }
    }

    @Override
    public void runOpMode() {
        smartGamepad = new GamepadEx(gamepad1);
        ringBot = new RingBot(this);
        ftcDashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, ftcDashboard.getTelemetry());

        // OpenCV begins here
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        ringBot.webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        ringBot.camera = OpenCvCameraFactory.getInstance().createWebcam(ringBot.webcamName, cameraMonitorViewId);
        ringBot.camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        RecordingPipeline pipeline = new RecordingPipeline();
        ringBot.camera.setPipeline(pipeline);
        ringBot.camera.setMillisecondsPermissionTimeout(3000); // Give plenty of time for the internal code to ready to avoid errors
        ringBot.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                ringBot.camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
                /*
                OpenCV Camera resolution FPS Test :
                    1280x720: 7.5 FPS (Theoretical max 55)
                    640x360: 30 FPS (Theoretical max 200)
                    480x360:
                 */
                telemetry.addLine("Camera stream initialized");
                FtcDashboard.getInstance().startCameraStream(ringBot.camera, 30);
                telemetry.update();
            }
            @Override
            public void onError(int errorCode) {
                throw new OpenCvCameraException("Could not open camera device. Error code: " + errorCode) ;
                // This will be called if the camera could not be opened
            }
        });

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status: ", "Initialized");
            telemetry.addData("FPS: ", ringBot.camera.getFps());
            telemetry.addData("THEORETICAL MAX FPS: ", ringBot.camera.getCurrentPipelineMaxFps());
            telemetry.update();
            idle();
        }
        // telemetry.addData("Status", "Initialized");
        // telemetry.update();
        // waitForStart();  // Wait for play button to be pressed
        mainLoop();
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
                ringBot.camera.startRecordingPipeline(
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


