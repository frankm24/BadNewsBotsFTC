package badnewsbots.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.PowerPlayCompBotMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;

import badnewsbots.hardware.GamepadEx;
import badnewsbots.pipelines.SignalSleevePipeline;
import badnewsbots.robots.PowerPlayCompBot;

public abstract class SignalSleeveAutonomous extends LinearOpMode {
    private PowerPlayCompBot robot;
    private GamepadEx smartGamepad;
    private FtcDashboard ftcDashboard;
    private SignalSleevePipeline.ConeOrientation coneOrientation;
    private double[] colorFilterAverages;
    private PowerPlayCompBotMecanumDrive drive;

    private final double tileSize = 24.0;
    private OpenCvCamera camera;

    private TrajectorySequence trajectory1;
    private TrajectorySequence trajectory2;
    private TrajectorySequence trajectory3;
    private Pose2d startPose;

    @Override
    public void runOpMode() {
        robot = new PowerPlayCompBot(this);
        drive = robot.getDrive();

        initializeAutonomousTrajectories();

        camera = robot.getCamera();
        smartGamepad = new GamepadEx(gamepad1);
        ftcDashboard = FtcDashboard.getInstance();

        SignalSleevePipeline pipeline = new SignalSleevePipeline(SignalSleevePipeline.CameraOrientation.RIGHT );
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
            drive.followTrajectorySequence(trajectory1);
        }
        if (coneOrientation == SignalSleevePipeline.ConeOrientation.TWO) {
            drive.followTrajectorySequence(trajectory2);
        }
        if (coneOrientation == SignalSleevePipeline.ConeOrientation.THREE) {
            drive.followTrajectorySequence(trajectory3);
        }
    }

    private void initializeAutonomousTrajectories() {
        startPose = new Pose2d(-1.5 * tileSize, -3 * tileSize + robot.width/2, Math.toRadians(0));

        trajectory1 = robot.getDrive().trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(startPose.getX() - tileSize, startPose.getY() + tileSize + robot.length/2),
                        Math.toRadians(90))
                .setReversed(false)
                .build();

        trajectory2 = robot.getDrive().trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .strafeLeft(1 * tileSize + robot.width/2)
                .build();

        trajectory3 = robot.getDrive().trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(startPose.getX() + tileSize, startPose.getY() + tileSize + robot.length/2),
                        Math.toRadians(90))
                .build();
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
