package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.PowerPlayCompBotMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

import badnewsbots.hardware.GamepadEx;
import badnewsbots.hardware.RotatingClaw;
import badnewsbots.pipelines.SignalSleevePipeline;
import badnewsbots.robots.PowerPlayCompBot;

@Autonomous
public class AutoBlue2HighGoal extends LinearOpMode {

    private PowerPlayCompBot robot;
    private GamepadEx smartGamepad;
    private FtcDashboard ftcDashboard;
    private SignalSleevePipeline.ConeOrientation coneOrientation;
    private double[] colorFilterAverages;
    private PowerPlayCompBotMecanumDrive drive;
    private RotatingClaw claw;

    private final double tileSize = 23.5;
    private OpenCvCamera camera;

    private TrajectorySequence blueAuto2_1;
    private TrajectorySequence blueAuto2_2;
    private TrajectorySequence blueAuto2_3;
    private Pose2d blueStartPose2;

    @Override
    public void runOpMode() {
        robot = new PowerPlayCompBot(this);
        drive = robot.getDrive();
        claw = robot.getRotatingClaw();

        initializeAutonomousTrajectories();

        camera = robot.getCamera();
        smartGamepad = new GamepadEx(gamepad1);
        ftcDashboard = FtcDashboard.getInstance();

        SignalSleevePipeline pipeline = new SignalSleevePipeline(SignalSleevePipeline.CameraOrientation.LEFT);
        initOpenCV(pipeline);

        while (!isStarted() && !isStopRequested()) {
            coneOrientation = pipeline.getConeOrientation();
            colorFilterAverages = pipeline.getFilterAverages();
            telemetry.addData("Status: ", "Initialized");
            telemetry.addData("Cone filter averages: (G, M, O)", Arrays.toString(colorFilterAverages));
            telemetry.addData("Cone orientation: ", coneOrientation);
            telemetry.addData("FPS: ", camera.getFps());
            telemetry.update();
            idle();
        }

        drive.setPoseEstimate(blueStartPose2);
        claw.grip();
        if (coneOrientation == SignalSleevePipeline.ConeOrientation.ONE) {
            drive.followTrajectorySequence(blueAuto2_1);
        }
        if (coneOrientation == SignalSleevePipeline.ConeOrientation.TWO) {
            //drive.followTrajectorySequence(blueAuto2_2);
        }
        if (coneOrientation == SignalSleevePipeline.ConeOrientation.THREE) {
            //drive.followTrajectorySequence(blueAuto2_3);
        }
    }

    private void initializeAutonomousTrajectories() {
        blueStartPose2 = new Pose2d(-(1.5 * tileSize), 3 * tileSize - robot.width/2, Math.toRadians(0));
        blueAuto2_1 = drive.trajectorySequenceBuilder(blueStartPose2)
                .back(tileSize)
                .addTemporalMarker(() -> {
                    //
                })
                .forward(2.5)
                .lineTo(new Vector2d( -2.5*tileSize + 2.5, tileSize/2))
                .lineTo(new Vector2d(-3*tileSize + robot.length/2, tileSize/2))
                .addTemporalMarker(() -> {
                    // pick up next cone
                    // tell linear mechanism to begin moving up, rotating as necessary
                })
                .waitSeconds(0.5)
                .setReversed(false)
                .lineTo(new Vector2d(-2*tileSize, tileSize/2))
                .splineTo(new Vector2d(-tileSize - 6.9, 6.4), Math.toRadians(-42.5))
                .setReversed(true)
                .addTemporalMarker(() -> {

                })
                .waitSeconds(.5)
                .splineTo(new Vector2d(-2*tileSize, tileSize/2), Math.toRadians(180))
                .lineTo(new Vector2d( -2.5*tileSize + 2.5, tileSize/2))
                .addTemporalMarker(() -> {

                })
                .waitSeconds(.5)
                .setReversed(false)
                . build();
    }

    private void initOpenCV(OpenCvPipeline pipeline) {
        camera.setPipeline(pipeline);
        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        //camera.showFpsMeterOnViewport(true);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //ftcDashboard.getTelemetry().addLine("Camera stream initialized");
                telemetry.update();
                camera.startStreaming(640, 480);
                //ftcDashboard.startCameraStream(camera, 30);
            }
            @Override
            public void onError(int errorCode) {
                throw new OpenCvCameraException("Could not open camera device. Error code: " + errorCode) ;
            }
        });
    }
}


