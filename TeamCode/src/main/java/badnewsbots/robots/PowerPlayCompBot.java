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
    private OpMode opMode;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public final double length = 15.0;
    public final double width = 14.6;
    private final PowerPlayCompBotMecanumDrive drive;
    private final OpenCvWebcam camera;

    // Mechanisms
    private final RotatingClaw rotatingClaw;

    public PowerPlayCompBot(OpMode opMode) {
        this.opMode = opMode;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        // Enables automatic "bulk reads" from robot hardware, so multiple .get()'s on hardware
        // Should improve performance significantly, since hardwareMap read calls take 2ms each
        for (LynxModule module : hardwareMap.getAll( LynxModule.class ) )
            module.setBulkCachingMode( LynxModule.BulkCachingMode.AUTO );

        // Sensors
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu.initialize(parameters);

        drive = new PowerPlayCompBotMecanumDrive(hardwareMap);

        rotatingClaw = new RotatingClaw(hardwareMap, telemetry);
        // Other
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        //telemetry.addLine("Robot object created");
        //telemetry.update();
    }

    public PowerPlayCompBotMecanumDrive getDrive() {return drive;}
    public OpenCvWebcam getCamera() {return camera;}
    public RotatingClaw getRotatingClaw() {return rotatingClaw;}

}
