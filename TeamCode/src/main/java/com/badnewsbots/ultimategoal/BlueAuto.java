package com.badnewsbots.ultimategoal;

import com.badnewsbots.ultimategoal.Drive;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@Autonomous(name = "Blue Autonomous")

public class BlueAuto extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private BNO055IMU imu;
    private ColorSensor colorSensor;
    private DcMotorEx flywheel;

    //Declare motors
    private DcMotor back_left;
    private DcMotor front_left;
    private DcMotor back_right;
    private DcMotor front_right;

    private final int tfChecks = 10;
    private char targetZone;

    private static final String VUFORIA_KEY =
            "AS27DuH/////AAABmTCx7YbI4khghC3lgWAjQo93WEllOCIMXs6D+me71rAjot43I8SxjF0AYT+65Zeuc+9biDnuDpRCjKWAoAa+YIwr/0BG+SXWuYWE3M/rsiwaiDE2UrkFnNVwAHHAClTI4lEEzY83m4wtLSrawB2q/OXrZ5oZNd3Pdf3gZHpv7S9QXEGpPxB8Rvwgi1rieSQIg0X2BBR1JSYRDVibN9Pymw8i/1CihRk3d84+bmlKXB9xxD9SPZgx2VRUQGFldkEllKapArv/k495zE5SKnYF6AYoYUTEx1ayO8Hrh/Ae3W7xuOc0GL7CB395oVdyamnhZrZ9zg7rGglSaYg3Fcz3aeA06OahIrcHXVB4X6Jg2Xcp";

    //V stupid object oriented programming V

    //instance of the Vuforia localizer engine
    private VuforiaLocalizer vuforia;

    //instance of the TensorFlow Object Detection engine.
    private TFObjectDetector tfod;

    public void smartWait(double time) {
        double t = getRuntime();
        while (true) {
            if (getRuntime() - t >= time) {
                return;
            }
        }
    }
    @Override

    public void runOpMode() {

        back_left = hardwareMap.get(DcMotor.class, "back_left");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        front_right = hardwareMap.get(DcMotor.class, "front_right");

        back_left.setDirection(DcMotor.Direction.REVERSE);
        front_left.setDirection(DcMotor.Direction.REVERSE);

        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;  //deg instead of rad
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC; //m/s^2
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        //Test code for determining game type
        /*This may be called on init so it detects the correct zone early on and doesn't...
        need to take time of the autonomous section to do it*/
        //Might want to Zoom in to only detect the rings in the right place to save energy and
        //CPU overhead
        while (!opModeIsActive() && !isStopRequested()) {
            List<Recognition> StarterStack = tfod.getRecognitions();
            if (StarterStack.size() == 1) {
                if (StarterStack.get(0).getLabel() == "Single") {
                    targetZone = 'B';
                }
                else if (StarterStack.get(0).getLabel() == "Quad") {
                    targetZone = 'C';
                }
            }
            else if (StarterStack.size() == 0) {
                targetZone = 'A';
            }
            telemetry.addData("Target Zone", targetZone);
            telemetry.update();
        }

        //Code here
        if (opModeIsActive()) {
            Drive drive = new Drive();
            drive.setMotors(back_left, front_left, back_right, front_right);

            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

                //Blue value Grey = 3/4, blue = 7+, blue is low
                //Red value Grey = 3/4, red = 20+, blue is low
                //White line if blue and red are both 20+
            byte BlueLineHits = 0;
            if (targetZone == 'A') {
                while (BlueLineHits < 1) {
                    drive.backward(10);
                    if ((colorSensor.blue() >= 14) && (colorSensor.red() < 5)) {
                        BlueLineHits += 1;
                        smartWait(0.2);
                    }
                }
                drive.forward(10);
                smartWait(2);
                drive.setMotorsToZero();
            }
            if (targetZone == 'B') {
                while (BlueLineHits < 1) {
                    drive.backward(10);
                    if ((colorSensor.blue() >= 14) && (colorSensor.red() < 5)) {
                        BlueLineHits += 1;
                        smartWait(0.2);
                    }
                }
                drive.setMotorsToZero();
                drive.backLeft(10);
                smartWait(.5);
                drive.setMotorsToZero();
            }
            if (targetZone == 'C') {
                while (BlueLineHits < 3) {
                    drive.backward(10);
                    if ((colorSensor.blue() >= 14) && (colorSensor.red() < 5)) {
                        BlueLineHits += 1;
                        smartWait(0.2);
                    }
                }
                drive.setMotorsToZero();
                drive.forward(10);
                smartWait(2);
                drive.setMotorsToZero();
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8F; //Can lower on the field
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}






