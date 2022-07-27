package badnewsbots.ml;

import android.app.Activity;
import android.os.Environment;

import org.opencv.core.Mat;
import org.tensorflow.lite.Interpreter;
import org.tensorflow.lite.nnapi.NnApiDelegate;
import org.tensorflow.lite.support.common.FileUtil;

import java.io.IOException;

public class TFLiteObjectDetector extends Activity {
    private final String modelPath = Environment.getExternalStorageDirectory() + "/FIRST/detect.tflite";
    private final String weightsPath = Environment.getExternalStorageDirectory() + "/FIRST/labelmap-detect.txt";
    private float accuracyThreshold = 0.5f;

    private Interpreter interpreter;
    private NnApiDelegate nnApiDelegate;

    public void init() throws IOException {
        nnApiDelegate = new NnApiDelegate();
        interpreter = new Interpreter(FileUtil.loadMappedFile(this, modelPath), new Interpreter.Options().addDelegate(nnApiDelegate));
    }
    public Mat predict() {
        return new Mat();
    }
}
