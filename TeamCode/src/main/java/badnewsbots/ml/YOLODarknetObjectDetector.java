package badnewsbots.ml;

import android.os.Environment;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
import org.opencv.utils.Converters;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Random;

/*
Alternative to using FTC's tfod is to handle recognitions with OpenCV
Also allows us to use YOLOv3 directly instead of trying to convert it to a tflite
since OpenCV supports loading a darknet model (each darknet model has
a .cfg and a .weights file).

Can also use the full version tensorflow lite (not FTC's lite-lite version)
with a model and an OpenCV pipeline. I'm not sure which of these
mthods is better for simply detecting objects, but I will probably try
all of them and see how the performance and ease of use varies.
FTC's implementation requires using Vuforia, which is good for
localization via VuMarks, but is not as convenient as an OpenCV pipeline,
which allows us to do fast preprocessing and non-ML based CV algorithms.
Either way, being able to load models besides basic object detection NNs,
which FTC's implementation is limited to, is a great feature to have.
Segmentation and other neural network based approaches may prove
necessary at some point.
    -This would be great for other projects like my (Frank's) self-driving car
    project because the FTC version only has object detection, no segmentation.
    This would be epic
*/
// Sources:
// https://www.linkedin.com/pulse/opencv-java-yolo-object-detection-images-svetozar-radoj%C4%8Din?trk=pulse-article_more-articles_related-content-card
public class YOLODarknetObjectDetector {
    private final String weightsPath = Environment.getExternalStorageDirectory() + "/FIRST/yolov3-tiny.weights";
    private final String configPath = Environment.getExternalStorageDirectory() + "/FIRST/yolov3-tiny.cfg";
    private final String labelmapPath = Environment.getExternalStorageDirectory() + "/FIRST/labelmap-yolov3.txt";
    private final Size modelSize = new Size(416, 416);

    Telemetry telemetry;

    private Net net;
    List<String> outputLayers = new ArrayList<>();
    List<String> layerNames = new ArrayList<>();
    List<String> labels = new ArrayList<>();
    ArrayList<Scalar> colors = new ArrayList<>();

    public YOLODarknetObjectDetector(Telemetry telemetry) {
        this.telemetry = telemetry;

        labels = Labels.readLabelsAsList(labelmapPath, telemetry, false);

        net = Dnn.readNetFromDarknet(configPath, weightsPath);

        layerNames = net.getLayerNames();
        for (Integer i : net.getUnconnectedOutLayers().toList()) {
            outputLayers.add(layerNames.get(i - 1));
        }
        // Generate random colors for each class
        Random random = new Random();
        for (int i= 0; i < labels.size(); i++) {
            colors.add(new Scalar( new double[] {random.nextInt(255), random.nextInt(255), random.nextInt(255)}));
        }
    }

    public Mat predict(Mat input) {
        Mat blob = Dnn.blobFromImage(input, 1 / 255.0, modelSize, new Scalar(0, 0, 0), false, false);
        net.setInput(blob);
        // A HashMap is a dictionary - items are stored as (key, value) pairs, not enumerated, order not guaranteed.
        HashMap<String, List> output = forwardImageOverNetwork(input, net, outputLayers);

        // Get boxes, confidences, and classes from the hash map of results
        ArrayList<Rect2d> boxes = (ArrayList<Rect2d>)output.get("boxes");
        ArrayList<Float> confidences = (ArrayList<Float>) output.get("confidences");
        ArrayList<Integer> class_ids = (ArrayList<Integer>)output.get("class_ids");

        // -- Now , do so-called “non-maxima suppression”
        //Non-maximum suppression is performed on the boxes whose confidence is equal to or greater than the threshold.
        // This will reduce the number of overlapping boxes:
        MatOfInt indices =  getBBoxIndicesFromNonMaximumSuppression(boxes,
                confidences);
        //-- Finally, go over indices in order to draw bounding boxes on the image:
        if (boxes != null) {
            drawBoxes(input, indices, boxes, labels, class_ids, colors);
        }
        telemetry.addLine("Prediction completed");
        return input;
    }
    private HashMap<String, List> forwardImageOverNetwork(Mat img, Net net, List<String> outputLayers) {
        HashMap<String, List> result = new HashMap<>();
        result.put("boxes", new ArrayList<Rect2d>());
        result.put("confidences", new ArrayList<Float>());
        result.put("class_ids", new ArrayList<Integer>());

        // -- The input image to a neural network needs to be in a certain format called a blob.
        //  In this process, it scales the image pixel values to a target range of 0 to 1 using a scale factor of 1/255.
        // It also resizes the image to the given size of (416, 416) without cropping
        // Construct a blob from the input image and then perform a forward  pass of the YOLO object detector,
        // giving us our bounding boxes and  associated probabilities:
        // scalefactor 1 / 255.0 to normalize pixel values [0, 1]
        Mat blob = Dnn.blobFromImage(img, 1, new Size(416, 416),
                new Scalar(new double[] {0.0, 0.0, 0.0}), false, false);
        net.setInput(blob);

        // -- the output of the forward() method is a list of Mats with all the goodies
        List<Mat> outputs = new ArrayList<Mat>();

        // Compute forward pass using YOLO's different output layers
        net.forward(outputs, outputLayers);

        // Each output of the network (ie, each row of the Mat from 'outputs') is a vector of the number
        // of classes + 5 elements. The first 4 elements represent (center_x, center_y, width, height).
        // The fifth element represents the confidence level of the box.
        // The remaining elements are the confidence levels (ie object types) associated with each class.
        // The box is assigned to the category corresponding to the highest score of the box:

        for (Mat output : outputs) {
            for (int i = 0; i < output.rows(); i++) {
                Mat row = output.row(i);
                List<Float> detect = new MatOfFloat(row).toList();
                List<Float> score = detect.subList(5, output.cols());
                int class_id = argmax(score);
                float conf = score.get(class_id);
                if (conf >= 0.5) {
                    int center_x = (int) (detect.get(0) * img.cols());
                    int center_y = (int) (detect.get(1) * img.rows());
                    int width = (int) (detect.get(2) * img.cols());
                    int height = (int) (detect.get(3) * img.rows());
                    int x = (center_x - width / 2);
                    int y = (center_y - height / 2);
                    Rect2d box = new Rect2d(x, y, width, height);
                    result.get("boxes").add(box);
                    result.get("confidences").add(conf);
                    result.get("class_ids").add(class_id);
                }
            }
        }
        return result;
    }
    private MatOfInt getBBoxIndicesFromNonMaximumSuppression(ArrayList<Rect2d> boxes, ArrayList<Float> confidences) {
        MatOfRect2d mOfRect = new MatOfRect2d();
        mOfRect.fromList(boxes);
        Mat matOfConfidences = Converters.vector_float_to_Mat(confidences);
        MatOfInt result = new MatOfInt();
        if (matOfConfidences.dims() > 1) {
            MatOfFloat mfConfs = new MatOfFloat(matOfConfidences);
            Dnn.NMSBoxes(mOfRect, mfConfs, (float)(0.6), (float)(0.5), result);
        }
        return result;
    }
    private Mat drawBoxes(Mat img, MatOfInt indices, ArrayList<Rect2d> boxes, List<String> cocoLabels,
                          ArrayList<Integer> class_ids, ArrayList<Scalar> colors) {
        //Scalar color = new Scalar( new double[]{255, 255, 0});
        if (!(indices.dims() == 0)) {
            List indices_list = indices.toList();
            for (int i = 0; i < boxes.size(); i++) {
                if (indices_list.contains(i)) {
                    Rect2d box = boxes.get(i);
                    Point x_y = new Point(box.x, box.y);
                    Point w_h = new Point(box.x + box.width, box.y + box.height);
                    Point text_point = new Point(box.x, box.y - 5);
                    Imgproc.rectangle(img, w_h, x_y, colors.get(class_ids.get(i)), 1);
                    String label = cocoLabels.get(class_ids.get(i));
                    Imgproc.putText(img, label, text_point, Imgproc.FONT_HERSHEY_SIMPLEX, 1, colors.get(class_ids.get(i)), 2);
                }
            }
        }
        return img;
    }
    private static int argmax(List<Float> list) {
        // check list is empty or not
        if (list == null || list.size() == 0) {
            return Integer.MIN_VALUE;
        }

        // create a new list to avoid modification
        // in the original list
        List<Float> sortedlist = new ArrayList<>(list);

        // sort list in natural order
        Collections.sort(sortedlist);

        // last element in the sorted list would be maximum
        return sortedlist.size() - 1;
    }
}


