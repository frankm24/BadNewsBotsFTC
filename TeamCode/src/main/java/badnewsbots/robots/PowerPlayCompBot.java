package badnewsbots.robots;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import badnewsbots.slam.UltrasonicSensor;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

public class PowerPlayCompBot {
    // OOP
    OpMode opMode;
    HardwareMap hardwareMap;
    Telemetry telemetry;

    // Drivetrain
    public DcMotor back_left;
    public DcMotor front_left;
    public DcMotor back_right;
    public DcMotor front_right;

    // Sensors
    public BNO055IMU imu;
    public OpenCvWebcam camera;
    public SampleMecanumDrive drive;

    // Other
    public WebcamName webcamName;

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

        drive = new SampleMecanumDrive(hardwareMap);
    }
    public void setDriveMotorPowerControllerVector(double LeftStickX, double LeftStickY, double RightStickX, double speedMultiplier) {
        LeftStickX *= speedMultiplier;
        LeftStickY *= speedMultiplier;
        RightStickX *= speedMultiplier;
        double denominator = Math.max(Math.abs(LeftStickY) + Math.abs(LeftStickX) + Math.abs(RightStickX), 1);
        double front_leftPower = (LeftStickY + LeftStickX + RightStickX) / denominator;
        double back_leftPower = (LeftStickY - LeftStickX + RightStickX) / denominator;
        double front_rightPower = (LeftStickY - LeftStickX - RightStickX) / denominator;
        double back_rightPower = (LeftStickY + LeftStickX - RightStickX) / denominator;
        front_left.setPower(front_leftPower);
        back_left.setPower(back_leftPower);
        front_right.setPower(front_rightPower);
        back_right.setPower(back_rightPower);
    }
    public void setAllDriveMotorsPower(double power) {
        front_left.setPower(power);
        front_right.setPower(power);
        back_left.setPower(power);
        back_right.setPower(power);
    }
}
