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
import badnewsbots.pipelines.SignalSleevePipeline;
import badnewsbots.robots.PowerPlayCompBot;

@Autonomous
public class PowerPlayAutoRed2 extends LinearOpMode {

    private PowerPlayCompBot robot;
    private GamepadEx smartGamepad;
    private FtcDashboard ftcDashboard;
    private SignalSleevePipeline.ConeOrientation coneOrientation;
    private double[] colorFilterAverages;
    private PowerPlayCompBotMecanumDrive drive;

    private OpenCvCamera camera;

    private void initOpenCV(OpenCvPipeline pipeline) {
        camera.setPipeline(pipeline);
        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        //camera.showFpsMeterOnViewport(true);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                ftcDashboard.getTelemetry().addLine("Camera stream initialized");
                telemetry.update();
                camera.startStreaming(640, 480);
                ftcDashboard.startCameraStream(camera, 30);
            }
            @Override
            public void onError(int errorCode) {
                throw new OpenCvCameraException("Could not open camera device. Error code: " + errorCode) ;
            }
        });
    }

    @Override
    public void runOpMode() {
        robot = new PowerPlayCompBot(this);
        camera = robot.getCamera();
        drive = robot.getDrive();
        smartGamepad = new GamepadEx(gamepad1);
        ftcDashboard = FtcDashboard.getInstance();

        SignalSleevePipeline pipeline = new SignalSleevePipeline();
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

        if (coneOrientation == SignalSleevePipeline.ConeOrientation.ONE) {
            drive.followTrajectorySequence(redAutoParking2_1);
        }
        if (coneOrientation == SignalSleevePipeline.ConeOrientation.TWO) {
            drive.followTrajectorySequence(redAutoParking2_2);
        }
        if (coneOrientation == SignalSleevePipeline.ConeOrientation.THREE) {
            drive.followTrajectorySequence(redAutoParking2_3);
        }
    }
    float robotLength = 15.0f;
    float robotWidth = 14.6f;
    float tileSize = 24.0f;
    Pose2d redStartPose2 = new Pose2d(1.5 * tileSize, -3 * tileSize + robotWidth/2, Math.toRadians(0));

    TrajectorySequence redAutoParking2_1 = robot.getDrive().trajectorySequenceBuilder(redStartPose2)
            .setReversed(true)
            .splineTo(new Vector2d(redStartPose2.getX() - tileSize, redStartPose2.getY() + tileSize + robotLength/2),
                    Math.toRadians(90))
            .setReversed(false)
            .build();

    TrajectorySequence redAutoParking2_2 = robot.getDrive().trajectorySequenceBuilder(redStartPose2)
            .setReversed(true)
            .strafeLeft(1 * tileSize + robotWidth/2)
            .build();

    TrajectorySequence redAutoParking2_3 = robot.getDrive().trajectorySequenceBuilder(redStartPose2)
            .splineTo(new Vector2d(redStartPose2.getX() + tileSize, redStartPose2.getY() + tileSize + robotLength/2),
                    Math.toRadians(90))
            .build();
}
