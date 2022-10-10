package badnewsbots.ml;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.openftc.easyopencv.OpenCvCamera;

public class FTC_TFLiteObjectDetector {
    private static final String VUFORIA_KEY = "AS27DuH/////AAABmTCx7YbI4khghC3lgWAjQo93WEllOCIMXs6D+me71rAjot43I8SxjF0AYT+65Zeuc+9biDnuDpRCjKWAoAa+YIwr/0BG+SXWuYWE3M/rsiwaiDE2UrkFnNVwAHHAClTI4lEEzY83m4wtLSrawB2q/OXrZ5oZNd3Pdf3gZHpv7S9QXEGpPxB8Rvwgi1rieSQIg0X2BBR1JSYRDVibN9Pymw8i/1CihRk3d84+bmlKXB9xxD9SPZgx2VRUQGFldkEllKapArv/k495zE5SKnYF6AYoYUTEx1ayO8Hrh/Ae3W7xuOc0GL7CB395oVdyamnhZrZ9zg7rGglSaYg3Fcz3aeA06OahIrcHXVB4X6Jg2Xcp";
    private static final String TFOD_MODEL_ASSET = Environment.getExternalStorageDirectory() + "/FIRST/tflitemodels/detect.tflite";
    private static final String TFOD_MODEL_LABELS = Environment.getExternalStorageDirectory() + "/FIRST/labelmap-detect.txt";

    private OpMode opMode;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private OpenCvCamera camera;

    public FTC_TFLiteObjectDetector(OpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;

        init();
    }
    private void init() {
        initVuforia();
        initTfod();
        if (tfod != null) {
            telemetry.addLine("TFOD activated");
            telemetry.update();
            tfod.activate();
            /*
            The TensorFlow software will scale the input images from the camera to a lower resolution.
            This can result in lower detection accuracy at longer distances (> 55cm or 22").
            If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            should be set to the value of the images used to create the TensorFlow Object Detection model
            (typically 16/9).
             */
            tfod.setZoom(1.0, 16.0/9.0);
        }
    }
    private void initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vu_parameters = new VuforiaLocalizer.Parameters();
        vu_parameters.vuforiaLicenseKey = VUFORIA_KEY;
        vu_parameters.secondsUsbPermissionTimeout = 3;
        // Uncomment this line below to use a webcam
        vu_parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(vu_parameters);
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, Labels.readLabelsAsArray(TFOD_MODEL_LABELS, telemetry));
    }
    public TFObjectDetector getTfod() {return tfod;}
    public VuforiaLocalizer getVuforia() {return vuforia;}
}
