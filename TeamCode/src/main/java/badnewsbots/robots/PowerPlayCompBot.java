package badnewsbots.robots;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.PowerPlayCompBotMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import badnewsbots.hardware.RotatingClaw;

public class PowerPlayCompBot {
    // OOP
    OpMode opMode;
    HardwareMap hardwareMap;
    Telemetry telemetry;

    private PowerPlayCompBotMecanumDrive drive;

    // Sensors
    private BNO055IMU imu;
    private OpenCvWebcam camera;

    // Mechanisms
    private RotatingClaw rotatingClaw;

    // Other
    private WebcamName webcamName;

    public PowerPlayCompBot(OpMode opMode) {
        this.opMode = opMode;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        init();
    }

    private void init() {
        // Enables automatic "bulk reads" from robot hardware, so multiple .get()'s on hardware
        // Should improve performance significantly, since hardwareMap read calls take 2ms each
        for (LynxModule module : hardwareMap.getAll( LynxModule.class ) )
            module.setBulkCachingMode( LynxModule.BulkCachingMode.AUTO );

        //back_left = hardwareMap.get(DcMotor.class, "back_left");
        //front_left = hardwareMap.get(DcMotor.class, "front_left");
        //back_right = hardwareMap.get(DcMotor.class, "back_right");
        //front_right = hardwareMap.get(DcMotor.class, "front_right");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1" );
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);

        // Reverse the motors that runs backwards (LEFT SIDE)
        //front_left.setDirection(DcMotor.Direction.REVERSE);
        //back_left.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu.initialize(parameters);

        drive = new PowerPlayCompBotMecanumDrive(hardwareMap);

        rotatingClaw = new RotatingClaw(hardwareMap, telemetry);
    }

    public PowerPlayCompBotMecanumDrive getDrive() {return drive; }
    public OpenCvWebcam getCamera() {return camera;}
    public RotatingClaw getRotatingClaw() {return rotatingClaw;}


    // servo position:
    // open = 0.00
    // closed = 0.09

}
