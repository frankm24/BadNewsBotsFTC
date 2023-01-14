package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.PowerPlayCompBotMecanumDrive;
import org.openftc.easyopencv.OpenCvCameraFactory;
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
    private float SpeedMultiplier = 1.0f; // scale movement speed

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

            float LeftStickY = -1 * smartGamepad.left_stick_y * SpeedMultiplier;
            float LeftStickX = smartGamepad.left_stick_x * SpeedMultiplier;
            float RightStickY = -1 * smartGamepad.right_stick_y * SpeedMultiplier;
            float RightStickX = smartGamepad.right_stick_x * SpeedMultiplier;

            double denominator = Math.max(Math.abs(LeftStickY) + Math.abs(LeftStickX) + Math.abs(RightStickX), 1);
            double front_leftPower = (LeftStickY + LeftStickX + RightStickX) / denominator;
            double back_leftPower = (LeftStickY - LeftStickX + RightStickX) / denominator;
            double front_rightPower = (LeftStickY - LeftStickX - RightStickX) / denominator;
            double back_rightPower = (LeftStickY + LeftStickX - RightStickX) / denominator;

            if (smartGamepad.start_pressed) {changeSpeedScale();}

            if (smartGamepad.dpad_up) {claw.moveSlideToPresetHeight(RotatingClaw.SlideHeight.HIGH_GOAL);}
            if (smartGamepad.dpad_left) {claw.moveSlideToPresetHeight(RotatingClaw.SlideHeight.MID_GOAL);}
            if (smartGamepad.dpad_down) {claw.moveSlideToPresetHeight(RotatingClaw.SlideHeight.LOW_GOAL);}

            if (smartGamepad.right_trigger_pressed) {claw.incrementSlidePosition();}
            if (smartGamepad.left_trigger_pressed) {claw.decrementSlidePosition();}
            if (smartGamepad.y_pressed) {claw.rotateToOtherSide();}
            if (smartGamepad.a_pressed) {claw.toggleGrip();}
            if (smartGamepad.left_bumper_pressed) {claw.initialGrab();}
            if (smartGamepad.right_bumper_pressed) {claw.letGo();}

            drive.setMotorPowers(front_leftPower, back_leftPower, back_rightPower, front_rightPower);

            telemetry.addData("front left power", front_leftPower);
            telemetry.addData("back left power", back_leftPower);
            telemetry.addData("front right power", front_rightPower);
            telemetry.addData("back right power", back_rightPower);
            telemetry.addData("dt", deltaTime);
            telemetry.update();
            prevTime = currentTime;
        }
    }

    @Override
    public void runOpMode() {
        robot = new PowerPlayCompBot(this);
        drive = robot.getDrive();
        camera = robot.getCamera();
        claw = robot.getRotatingClaw();
        smartGamepad = new GamepadEx(gamepad1);
        ftcDashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, ftcDashboard.getTelemetry());
        //telemetry.setMsTransmissionInterval(17);

        // OpenCV begins here

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        recorder = new OpenCvRecorder(camera, telemetry, false);
        recorder.openCameraAsync();

        // init loop
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status: ", "Initialized");

            if (recorder.isRecording()) {
                telemetry.addData("Recording FPS: ", camera.getFps());
                telemetry.addData("Theoretical Max FPS: ", camera.getCurrentPipelineMaxFps());
            }

            telemetry.update();
            idle();
        }
        // telemetry.addData("Status", "Initialized");
        // telemetry.update();
        // waitForStart();  // Wait for play button to be pressed
        mainLoop();
    }
    private void changeSpeedScale()  {
        if (SpeedMultiplier == 0.5f) SpeedMultiplier = 1.0f; else SpeedMultiplier = 0.5f;
    }
}



