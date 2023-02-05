package badnewsbots.ml;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;

import java.util.ArrayList;
import java.util.List;

public class ImageSpaceToFieldConverter {
    private Mat cameraMatrix, rotationVector, rotationMatrix,
    translationVector, extrinsicMatrix, projectionMatrix, inverseHomographyMatrix;
    private MatOfDouble homographyMatrix;
    private MatOfDouble distCoeffs;
    private Point point;
    private List<Point> image_points_list;
    private MatOfPoint2f image_points;
    private List<Point3> world_points_list;
    private MatOfPoint3f world_points;
    private List<Mat> matricesToConcat;

    public ImageSpaceToFieldConverter() {
        // Add key image space point values
        image_points_list = new ArrayList<>();
        image_points_list.add(new Point());
        image_points_list.add(new Point());
        image_points_list.add(new Point());
        image_points_list.add(new Point());
        image_points.fromList(image_points_list);
        // key vector space corresponding point values
        world_points_list = new ArrayList<>();
        world_points_list.add(new Point3());
        world_points_list.add(new Point3());
        world_points_list.add(new Point3());
        world_points_list.add(new Point3());
        world_points.fromList(world_points_list);

    }

    public void calculate() {
        Calib3d.solvePnP(world_points, image_points, cameraMatrix, distCoeffs, rotationVector, translationVector);
        Calib3d.Rodrigues(rotationVector, rotationMatrix);
        matricesToConcat = new ArrayList<>();
        matricesToConcat.add(rotationMatrix);
        matricesToConcat.add(translationVector);
        Core.hconcat(matricesToConcat, extrinsicMatrix);
        projectionMatrix = cameraMatrix.mul(extrinsicMatrix);
        Mat.Atable<Double> p11 = projectionMatrix.at(Double.class, 0, 0),
                p12 = projectionMatrix.at(Double.class, 0, 1),
                p14 = projectionMatrix.at(Double.class, 0, 3),
                p21 = projectionMatrix.at(Double.class, 1, 0),
                p22 = projectionMatrix.at(Double.class, 1, 1),
                p24 = projectionMatrix.at(Double.class, 1, 3),
                p31 = projectionMatrix.at(Double.class, 2, 0),
                p32 = projectionMatrix.at(Double.class, 2, 1),
                p34 = projectionMatrix.at(Double.class, 2, 3);
        homographyMatrix = new MatOfDouble(3, 3);
        homographyMatrix.put(0, 0, p11.getV());
        homographyMatrix.put(0, 1, p12.getV());
        homographyMatrix.put(0, 2, p14.getV());
        homographyMatrix.put(1, 0, p21.getV());
        homographyMatrix.put(1, 1, p22.getV());
        homographyMatrix.put(1, 2, p24.getV());
        homographyMatrix.put(2, 0, p31.getV());
        homographyMatrix.put(2, 1, p32.getV());
        homographyMatrix.put(2, 2, p34.getV());
        inverseHomographyMatrix = homographyMatrix.inv();
    }
}
