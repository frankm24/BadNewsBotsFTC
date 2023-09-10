package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.PowerPlayCompBotMecanumDrive;
import org.opencv.core.Core;
import org.opencv.photo.CalibrateCRF;
import org.openftc.easyopencv.OpenCvWebcam;

import badnewsbots.OpenCvRecorder;
import badnewsbots.hardware.GamepadEx;
import badnewsbots.hardware.RotatingClaw;
import badnewsbots.robots.PowerPlayCompBot;

@TeleOp
public class PowerPlayTeleOp extends LinearOpMode {
    // Robot object
    private PowerPlayCompBot robot;

    // Settings
    private final float slowSpeed = 0.25f;
    private final float fastSpeed = 0.5f;
    private float speedMultiplier = slowSpeed; // scale movement speed

    private GamepadEx smartGamepad;
    private FtcDashboard ftcDashboard;
    private OpenCvRecorder recorder;

    private OpenCvWebcam camera;
    private PowerPlayCompBotMecanumDrive drive;
    private RotatingClaw claw;

    public void mainLoop() {
        double prevTime = 0;
        while (opModeIsActive()) {
            double currentTime = getRuntime();
            double deltaTime = currentTime - prevTime;
            smartGamepad.update();
            //claw.update();

            float LeftStickY = smartGamepad.left_stick_y * speedMultiplier;
            float LeftStickX = -smartGamepad.left_stick_x * speedMultiplier;
            float RightStickY = smartGamepad.right_stick_y * speedMultiplier;
            float RightStickX = smartGamepad.right_stick_x * speedMultiplier;

            double denominator = Math.max(Math.abs(LeftStickY) + Math.abs(LeftStickX) + Math.abs(RightStickX), 1);
            double front_leftPower = (LeftStickY + LeftStickX + RightStickX) / denominator;
            double back_leftPower = (LeftStickY - LeftStickX + RightStickX) / denominator;
            double front_rightPower = (LeftStickY - LeftStickX - RightStickX) / denominator;
            double back_rightPower = (LeftStickY + LeftStickX - RightStickX) / denominator;

            if (smartGamepad.start_pressed) changeSpeedScale();

            if (smartGamepad.dpad_up) claw.moveSlidesToPresetHeight(RotatingClaw.SlideHeight.HIGH_GOAL);
            if (smartGamepad.dpad_left) claw.moveSlidesToPresetHeight(RotatingClaw.SlideHeight.MID_GOAL);
            if (smartGamepad.dpad_down) claw.moveSlidesToPresetHeight(RotatingClaw.SlideHeight.LOW_GOAL);

            if (smartGamepad.right_trigger_pressed) claw.incrementSlidePositions();
            if (smartGamepad.left_trigger_pressed) claw.decrementSlidePositions();
            if (smartGamepad.y_pressed) claw.rotateToOtherSide();
            if (smartGamepad.a_pressed) claw.toggleGrip();
            if (smartGamepad.b_pressed) claw.moveSlidesToPresetHeight(RotatingClaw.SlideHeight.BOTTOM);
            if (smartGamepad.left_bumper_pressed) claw.initialGrab();
            if (smartGamepad.right_bumper_pressed) claw.letGo();

            drive.setMotorPowers(front_leftPower, back_leftPower, back_rightPower, front_rightPower);
            telemetry.addData("Claw state", claw.getCurrentGripperState());
            telemetry.addData("Claw preset height", claw.getCurrentSlideHeight());
            telemetry.addData("Slide position", claw.getSlide2PosTicks());
            telemetry.addData("front left power", front_leftPower);
            telemetry.addData("back left power", back_leftPower);
            telemetry.addData("front right power", front_rightPower);
            telemetry.addData("back right power", back_rightPower);
            telemetry.addData("dt (ms)", deltaTime * 1E3);
            telemetry.update();
            prevTime = currentTime;
        }
    }

    @Override
    public void runOpMode() {
        robot = new PowerPlayCompBot(this);
        drive = robot.getDrive();
        //camera = robot.getCamera();
        claw = robot.getRotatingClaw();
        smartGamepad = new GamepadEx(gamepad1);
        ftcDashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, ftcDashboard.getTelemetry());
        //telemetry.setMsTransmissionInterval(17);

        // OpenCV begins here
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        //recorder = new OpenCvRecorder(camera, telemetry, false);
        //recorder.openCameraAsync();

        // init loop
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status: ", "Initialized");

            //if (recorder.isRecording()) {
            //    telemetry.addData("Recording FPS: ", camera.getFps());
             //   telemetry.addData("Theoretical Max FPS: ", camera.getCurrentPipelineMaxFps());
            //}

            telemetry.update();
            idle();
        }
        mainLoop();
    }

    private void changeSpeedScale()  {
        if (speedMultiplier == fastSpeed) speedMultiplier = slowSpeed; else speedMultiplier = fastSpeed;
    }
}



