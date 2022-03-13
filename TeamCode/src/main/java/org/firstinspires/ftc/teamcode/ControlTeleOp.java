package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@TeleOp
public class ControlTeleOp extends LinearOpMode {
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


    //settings
    final int IntakeTargetSpeed = 1000; //ticks per second
    final int CarouselTargetSpeed = 2000; //ticks per second
    float SpeedMultiplier = 0.5f; //scale movement speed

    //Button code
    double DebounceTimer = 0.3; // 0.3 seconds

    boolean RevStatus = false;
    boolean doorClosed = false;
    boolean servoUp = false;
    boolean isArmInMotion = false;
    boolean armIsRaisedBarrier = false;
    int duckIsSpinning = 0;
    boolean bButtonServoIsInHighPos = false;
    boolean ADebounce = false;
    double ADebounceTime = 0;
    boolean XDebounce = false;
    double XDebounceTime = 0;
    boolean YDebounce = false;
    double YDebounceTime = 0;
    boolean BDebounce = false;
    double BDebounceTime = 0;
    boolean dpad_upDebounce = false;
    double dpad_upDebounceTime = 0;
    boolean right_bumperDebounce = false;
    double right_bumperDebounceTime = 0;
    boolean left_bumperDebounce = false;
    double left_bumperDebounceTime = 0;
    boolean startDebounce = false;
    double startDebounceTime = 0;


    class InputLoop implements Runnable {
        @Override
        public void run() {
            while (opModeIsActive()) {
                idle();
            }
        }
    }

    class ActionLoop implements Runnable {
        @Override
        public void run() {
            while (opModeIsActive()) {
                idle();
            }
        }
    }

    public void waitWithoutSleep(double time) {
        double t = getRuntime();
        while (true) {
            if (getRuntime() - t >= time) {
                return;
            }
        }
    }

    public void enableGamepadControl() {
        String LeftStickInputDirection = "";
        telemetry.addData("Left Stick Input Direction", LeftStickInputDirection);
        String RightStickInputDirection = "";
        telemetry.addData("Right Stick Input Direction", RightStickInputDirection);
        telemetry.update();

        while (opModeIsActive()) {
            float LeftStickY = -1 * gamepad1.left_stick_y * SpeedMultiplier;
            float LeftStickX = gamepad1.left_stick_x * SpeedMultiplier;
            float RightStickY = -1 * gamepad1.right_stick_y * SpeedMultiplier;
            float RightStickX = gamepad1.right_stick_x * SpeedMultiplier;
            boolean A = gamepad1.a;
            boolean X = gamepad1.x;
            boolean Y = gamepad1.y;
            boolean B = gamepad1.b;
            boolean dpad_up = gamepad1.dpad_up;
            boolean right_bumper = gamepad1.right_bumper;
            boolean left_bumper = gamepad1.left_bumper;
            boolean start = gamepad1.start;
            double time = getRuntime();

            if (Y && !YDebounce) {
                YDebounce = true;
                arm.setPosition(0.5);
                sleep(1000);
                arm.setPosition(-0.35);
                YDebounce = false;
            }
            if (X && !XDebounce) {
                XDebounce = true;
                door.setPosition(0);
                sleep(100);
                arm.setPosition(0.78);
                sleep(2000);
                door.setPosition(0.65);
                sleep(200);
                arm.setPosition(0);
                XDebounce = false;
            }
            if (A && !ADebounce) {
                ADebounceTime = time;
                ADebounce = true;
                if (!RevStatus) {
                    intake.setVelocity(IntakeTargetSpeed);
                    RevStatus = true;
                } else {
                    intake.setVelocity(0);
                    RevStatus = false;
                }
            }
            if (B && !BDebounce) {
                BDebounceTime = time;
                BDebounce = true;
                if (bButtonServoIsInHighPos) {
                    arm.setPosition(0.47); //0.47
                }
                else {
                    bButtonServoIsInHighPos = true;
                    arm.setPosition(0.79);
                }
            }
            if (dpad_up && !dpad_upDebounce) {
                dpad_upDebounceTime = time;
                dpad_upDebounce = true;
                arm.setPosition(0.6);
            }
            if (right_bumper && !right_bumperDebounce) {
                right_bumperDebounceTime = time;
                right_bumperDebounce = true;
                if (duckIsSpinning == 2) {
                    duckIsSpinning = 0;
                    carousel.setVelocity(-CarouselTargetSpeed);
                } else if (duckIsSpinning == 1) {
                    duckIsSpinning = 2;
                    carousel.setVelocity(0);
                } else {
                    duckIsSpinning = 1;
                    carousel.setVelocity(CarouselTargetSpeed);
                }

            }
            if (left_bumper && !left_bumperDebounce) {
                left_bumperDebounceTime = time;
                left_bumperDebounce = true;
                if (armIsRaisedBarrier) {
                    armIsRaisedBarrier = false;
                    arm.setPosition(0);
                    sleep(30);
                    door.setPosition(0.65);
                } else {
                    armIsRaisedBarrier = true;
                    arm.setPosition(0.2);
                    door.setPosition(0);
                }

            }
            if (start && !startDebounce) {
                startDebounce = true;
                if (SpeedMultiplier == 0.5f) {
                    SpeedMultiplier = 1.0f;
                } else {
                    SpeedMultiplier = 0.5f;
                }
            }
            if (time - ADebounceTime >= DebounceTimer) {
                ADebounce = false;
            }
            if (time - BDebounceTime >= DebounceTimer) {
                BDebounce = false;
            }
            if (time - dpad_upDebounceTime >= DebounceTimer) {
                dpad_upDebounce = false;
            }
            if (time - right_bumperDebounceTime >= DebounceTimer) {
                right_bumperDebounce = false;
            }
            if (time - left_bumperDebounceTime >= DebounceTimer) {
                left_bumperDebounce = false;
            }
            if (time - startDebounceTime >= DebounceTimer) {
                startDebounce = false;
            }
            double denominator = Math.max(Math.abs(LeftStickY) + Math.abs(LeftStickX) + Math.abs(RightStickX), 1);
            double front_leftPower = (LeftStickY + LeftStickX + RightStickX) / denominator;
            double back_leftPower = (LeftStickY - LeftStickX + RightStickX) / denominator;
            double front_rightPower = (LeftStickY - LeftStickX - RightStickX) / denominator;
            double back_rightPower = (LeftStickY + LeftStickX - RightStickX) / denominator;
            front_left.setPower(front_leftPower);
            back_left.setPower(back_leftPower);
            front_right.setPower(front_rightPower);
            back_right.setPower(back_rightPower);

            telemetry.addData("IMU Data", imu.getAngularOrientation());
            telemetry.addData("front_tof: ", front_tof.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
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
        enableGamepadControl();
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


