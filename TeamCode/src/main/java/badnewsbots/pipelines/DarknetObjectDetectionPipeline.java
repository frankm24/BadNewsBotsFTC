package badnewsbots.pipelines;

import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfRect2d;
import org.opencv.core.Point;
import org.opencv.core.Rect2d;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.dnn.Dnn;
import org.opencv.dnn.Net;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;
import java.io.FileNotFoundException;
import java.net.URL;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

@Deprecated
public class DarknetObjectDetectionPipeline extends OpenCvPipeline {
    private final String model_weights;
    private final String model_config;
    //private final File current_dir;
    private final String class_file_name_dir;
    private final List<String> classes;
    private final List<String> output_layers;
    private List<String> layer_names;
    private Net network;
    private Size size;
    private Integer height;
    private Integer width;
    private Integer channels;
    private Scalar mean;
    private Mat image;
    private Mat blob;
    private List<Mat> outputs;
    private List<Rect2d> boxes;
    private List<Float> confidences;
    private List<Integer> class_ids;
    private String outputFileName;

    private static int argmax(List<Float> array) {
        float max = array.get(0);
        int re = 0;
        for (int i = 1; i < array.size(); i++) {
            if (array.get(i) > max) {
                max = array.get(i);
                re = i;
            }
        }
        return re;
    }
    private void setClasses() {
        try {
            File f = new File(class_file_name_dir);
            Scanner reader = new Scanner(f);
            while (reader.hasNextLine()) {
                String class_name = reader.nextLine();
                classes.add(class_name);
            }
        } catch (FileNotFoundException e) {
            System.out.println(e);
        }
    }
    private void setUnconnectedLayers() {
        for (Integer i : network.getUnconnectedOutLayers().toList()) {
            output_layers.add(layer_names.get(i - 1));
        }
    }
    private void setLayerNames() {
        layer_names = network.getLayerNames();
    }
    private void preprocessImage(Mat input) {
        Mat img = input;
        Mat resizedImage = new Mat();
        Imgproc.resize(img, resizedImage, size);
        height = resizedImage.height();
        width = resizedImage.width();
        channels = resizedImage.channels();
        image = resizedImage;
    }
    private void detectObject() {
        Mat blob_from_image = Dnn.blobFromImage(image, 1.0, size, mean, true, false);
        // recommended scalefactor: 0.00392
        network.setInput(blob_from_image);
        outputs = new ArrayList<Mat>();
        network.forward(outputs, output_layers);
        blob = blob_from_image;
    }
    private void getBoxDimensions() {
        for (Mat output : outputs) {
            for (int i = 0; i < output.height(); i++) {
                Mat row = output.row(i);
                MatOfFloat temp = new MatOfFloat(row);
                List<Float> detect = temp.toList();
                List<Float> score = detect.subList(5, 85);
                int class_id = argmax(score);
                float conf = score.get(class_id);
                if (conf >= 0.4) {
                    int center_x = (int) (detect.get(0) * width);
                    int center_y = (int) (detect.get(1) * height);
                    int w = (int) (detect.get(2) * width);
                    int h = (int) (detect.get(3) * height);
                    int x = (center_x - w / 2);
                    int y = (center_y - h / 2);
                    Rect2d box = new Rect2d(x, y, w, h);
                    boxes.add(box);
                    confidences.add(conf);
                    class_ids.add(class_id);
                }
            }
        }
    }
    private void drawLabels() {
        double[] rgb = new double[]{255, 255, 0};
        Scalar color = new Scalar(rgb);
        MatOfRect2d mat = new MatOfRect2d();
        mat.fromList(boxes);
        MatOfFloat confidence = new MatOfFloat();
        confidence.fromList(confidences);
        MatOfInt indices = new MatOfInt();
        int font = Imgproc.FONT_HERSHEY_PLAIN;
        Dnn.NMSBoxes(mat, confidence, (float) (0.4), (float) (0.4), indices);
        List<Integer> indices_list = indices.toList();
        for (int i = 0; i < boxes.size(); i++) {
            if (indices_list.contains(i)) {
                Rect2d box = boxes.get(i);
                Point x_y = new Point(box.x, box.y);
                Point w_h = new Point(box.x + box.width, box.y + box.height);
                Point text_point = new Point(box.x, box.y - 5);
                Imgproc.rectangle(image, w_h, x_y, color);
                String label = classes.get(class_ids.get(i));
                Imgproc.putText(image, label, text_point, font, 1, color);
            }
        }
    }

    public DarknetObjectDetectionPipeline() {
        //System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        this.outputFileName = outputFileName;
        boxes = new ArrayList<>();
        classes = new ArrayList<>();
        class_ids = new ArrayList<>();
        layer_names = new ArrayList<>();
        confidences = new ArrayList<>();
        double[] means = {0.0, 0.0, 0.0};
        mean = new Scalar(means);
        output_layers = new ArrayList<>();
        int image_size = 416;
        size = new Size(image_size, image_size);
        //current_dir = Environment.getExternalStorageDirectory(); // Change if using EOCV-Sim
        /*
        current_dir = new File(System.getProperty("user.dir") + "/Desktop/BadNewsBotsFTC/");
        model_weights = current_dir + weights_path;
        model_config = current_dir + config_path;
        class_file_name_dir = current_dir + labels_path;
         */

        model_weights = "/src/main/res/ml/yolov3/yolov3-tiny.weights";
        model_config = "/src/main/res/ml/yolov3/yolov3-tiny.cfg";
        class_file_name_dir = "/src/main/res/ml/yolov3/coco-labels.txt";
        network = Dnn.readNet(model_weights, model_config);

        setClasses();
        setLayerNames();
        setUnconnectedLayers();
    }

    @Override
    public Mat processFrame(Mat input) {
        preprocessImage(input);
        detectObject();
        getBoxDimensions();
        drawLabels();
        return image;
    }
    @Override
    public void init(Mat input) {
        preprocessImage(input);
        detectObject();
        getBoxDimensions();
        drawLabels();
    }
}
