package badnewsbots.pipelines;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SignalSleevePipelineAlternate extends OpenCvPipeline {
    private final boolean returnResult = true;

    public enum ConeOrientation {
        ONE,
        TWO,
        THREE,
        NONE
    }

    private volatile ConeOrientation coneOrientation = ConeOrientation.NONE;

    private final Scalar greenMin = new Scalar(45, 30, 50);
    private final Scalar greenMax = new Scalar(70, 255, 255);

    private final Scalar magentaMin = new Scalar(145, 30, 50);
    private final Scalar magentaMax = new Scalar(155, 255, 255);

    private final Scalar orangeMin = new Scalar(15, 30, 50);
    private final Scalar orangeMax = new Scalar(25, 255, 255);

    private final Scalar white = new Scalar(255, 255, 255);
    private final Scalar green = new Scalar(0, 255, 0);
    private final Scalar magenta = new Scalar(255, 0, 255);
    private final Scalar orange = new Scalar(255, 127, 0);

    private final Point point1 = new Point(300, 300);
    private final Point point2 = new Point(400, 400);
    private final Rect roi = new Rect(point1, point2);

    private int greenMean;
    private int magentaMean;
    private int orangeMean;

    private Size pipelineSize;

    private Mat hsvImage;
    private Mat submat;
    private Mat greenFiltered;
    private Mat orangeFiltered;
    private Mat magentaFiltered;

    @Override
    public void init(Mat input) {
        pipelineSize = input.size();
        hsvImage = new Mat();
        submat = hsvImage.submat(roi);

        greenFiltered = new Mat();
        magentaFiltered = new Mat();
        orangeFiltered = new Mat();
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_RGB2HSV);
        Core.inRange(submat, greenMin, greenMax, greenFiltered);
        Core.inRange(submat, magentaMin, magentaMax, magentaFiltered);
        Core.inRange(submat, orangeMin, orangeMax, orangeFiltered);

        greenMean = Core.countNonZero(greenFiltered);
        magentaMean = Core.countNonZero(magentaFiltered);
        orangeMean = Core.countNonZero(orangeFiltered);

        double maxMean = Math.max(magentaMean, Math.max(greenMean, orangeMean));

        if (maxMean == greenMean) {
            coneOrientation = ConeOrientation.ONE;
        } else if (maxMean == magentaMean) {
            coneOrientation = ConeOrientation.TWO;
        } else if (maxMean == orangeMean) {
            coneOrientation = ConeOrientation.THREE;
        } else {
            coneOrientation = ConeOrientation.NONE;
        }

        // Code to visualize results (NOT vital)
        Imgproc.rectangle(input, roi, white);

        if (returnResult) {return input;} else {return input;}
    }

    public ConeOrientation getConeOrientation() {return coneOrientation;}

    public double[] getFilterAverages() {
        return new double[] {greenMean, magentaMean, orangeMean};
    }

    @Override
    public void onViewportTapped() {

    }

}
