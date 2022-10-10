package badnewsbots.robots;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvWebcam;

public class RingBot {
    // OOP
    OpMode opMode;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    FtcDashboard ftcDashboard;
    // Drive
    public DcMotor back_left;
    public DcMotor front_left;
    public DcMotor back_right;
    public DcMotor front_right;
    // Mechanisms
    public Servo pusher;
    public DcMotorEx intake;
    public DcMotorEx flywheel;
    // Sensors
    public BNO055IMU imu;
    public Rev2mDistanceSensor front_tof;
    public OpenCvWebcam camera;
    // Other
    public RevBlinkinLedDriver led_driver;
    public WebcamName webcamName;

    public RingBot(OpMode opMode) {
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

        back_left = hardwareMap.get(DcMotor.class, "back_left");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        //led_driver = hardwareMap.get(RevBlinkinLedDriver.class, "led_driver");
        try {
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            flywheel = hardwareMap.get(DcMotorEx.class, "duck");
            front_tof = hardwareMap.get(Rev2mDistanceSensor.class, "front_tof");
        } catch (Exception e) {
            telemetry.addLine("Expansion hub not working.");
        }
        pusher = hardwareMap.get(Servo.class, "pusher");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
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
    }
}
