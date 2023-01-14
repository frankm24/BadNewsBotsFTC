package badnewsbots.pipelines;

import android.os.Environment;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SignalSleevePipeline extends OpenCvPipeline {
    private final boolean returnResult = true;
    private final boolean saveImage = false;

    public enum ConeOrientation {
        ONE,
        TWO,
        THREE,
        NONE
    }

    private ConeOrientation coneOrientation = ConeOrientation.NONE;

    private final Scalar greenMin = new Scalar(45, 100, 100);
    private final Scalar greenMax = new Scalar(70, 255, 255);

    private final Scalar magentaMin = new Scalar(150, 120, 150);
    private final Scalar magentaMax = new Scalar(170, 200, 200);

    private final Scalar orangeMin = new Scalar(13, 190, 150);
    private final Scalar orangeMax = new Scalar(20, 255, 255);

    private final Scalar roiOutlineColor = new Scalar(255, 0, 0);
    private final Scalar green = new Scalar(0, 255, 0);
    private final Scalar magenta = new Scalar(255, 0, 255);
    private final Scalar orange = new Scalar(255, 127, 0);

    private final Point point1 = new Point(300, 300);
    private final Point point2 = new Point(400, 400);
    private final Rect roi = new Rect(point1, point2);

    private int greenCount;
    private int magentaCount;
    private int orangeCount;

    private Size pipelineSize;

    private Mat hsvImage;
    private Mat greenFiltered;
    private Mat orangeFiltered;
    private Mat magentaFiltered;

    @Override
    public void init(Mat input) {
        pipelineSize = input.size();
        hsvImage = new Mat();

        greenFiltered = new Mat();
        magentaFiltered = new Mat();
        orangeFiltered = new Mat();
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvImage, greenMin, greenMax, greenFiltered);
        Core.inRange(hsvImage, magentaMin, magentaMax, magentaFiltered);
        Core.inRange(hsvImage, orangeMin, orangeMax, orangeFiltered);

        Mat greenROI = greenFiltered.submat(roi);
        Mat magentaROI = magentaFiltered.submat(roi);
        Mat orangeROI = orangeFiltered.submat(roi);

        greenCount = Core.countNonZero(greenROI);
        magentaCount = Core.countNonZero(magentaROI);
        orangeCount = Core.countNonZero(orangeROI);

        double maxMean = Math.max(magentaCount, Math.max(greenCount, orangeCount));

        if (maxMean == greenCount) {
            coneOrientation = ConeOrientation.ONE;
        } else if (maxMean == magentaCount) {
            coneOrientation = ConeOrientation.TWO;
        } else if (maxMean == orangeCount) {
            coneOrientation = ConeOrientation.THREE;
        } else {
            coneOrientation = ConeOrientation.NONE;
        }

        // Code to visualize results (NOT vital)
        input.setTo(green, greenFiltered);
        input.setTo(magenta, magentaFiltered);
        input.setTo(orange, orangeFiltered);
        Imgproc.rectangle(input, roi, roiOutlineColor);

        if (saveImage) {Imgcodecs.imwrite(Environment.getExternalStorageDirectory() + "/signal.png", hsvImage);}

        if (returnResult) {return input;} else {return input;}
    }

    public ConeOrientation getConeOrientation() {return coneOrientation;}

    public double[] getFilterAverages() {
        return new double[] {greenCount, magentaCount, orangeCount};
    }

    @Override
    public void onViewportTapped() {

    }

}
