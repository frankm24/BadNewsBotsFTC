package org.firstinspires.ftc.teamcode.slam;

import android.media.Image;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.vuforia.Vec2F;

import org.apache.commons.math3.stat.descriptive.moment.VectorialCovariance;
import org.checkerframework.checker.units.qual.min;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.ConcurrentModificationException;
import java.util.Iterator;
import java.util.List;


public class Map {
    private double DUPLICATE_THRESHOLD;
    private List<Vector2d> points;
    private List<Vector2d> last_points;
    private DistanceUnit unit;
    private final char CHARACTER_EMPTY = ' ';
    private final char CHARACTER_FILL = '@';
    private final char CHARACTER_POINTER = '^';
    private final double[] point_color = new double[] {31, 0, 0};

    public Map(List<Vector2d> initial_points, DistanceUnit unit, double thresh) {
        points = initial_points;
        last_points = new ArrayList<>();
        this.unit = unit;
        DUPLICATE_THRESHOLD = thresh;
    }

    public void addPoints(List<Vector2d> points) {
        this.points.addAll(points);
        last_points.clear();
        last_points.addAll(points);
    }

    public void deletePoints(List<Vector2d> points) {
        this.points.removeAll(points);
    }

    public List<Vector2d> getPoints() {
        return new ArrayList<>(this.points);
    }


    public void removeDuplicatePoints() {
        List<Vector2d> pointsToRemove = new ArrayList<>();
        List<Vector2d[]> combinations = new ArrayList<>();
        for (Vector2d point0 : points) {
            for (Vector2d point1 : points) {
                combinations.add(new Vector2d[] {point0, point1});
            }
        }
       for (Vector2d[] combination : combinations) {
           Vector2d point0 = combination[0];
           Vector2d point1 = combination[1];
           if (point0.distTo(point1) <= DUPLICATE_THRESHOLD) {
               pointsToRemove.add(point1);
           }
       }
       points.removeAll(pointsToRemove);
    }

    @Deprecated
    public String[] renderASCII(Vector2d reference_position, double range) {
        //defualt range: 21
        List<Vector2d> points_to_render = new ArrayList<>();

        for (Vector2d point : points) {
            if (reference_position.distTo(point) > range) {
                continue;
            }
            Vector2d point_relative = point.minus(reference_position);
            points_to_render.add(point_relative);
        }
        // Make empty 2d char array and fill with empty character
        char[][] display = new char[42][42];
        for (int i = 0; i < display.length; i++) {
            Arrays.fill(display[i], CHARACTER_EMPTY);
        }
        // Set characters corresponding to points around frame of reference
        for (Vector2d point : points_to_render) {
            int rounded_x = (int) Math.round(point.getX());
            int rounded_y = (int) Math.round(point.getY());
            display[rounded_y][rounded_x] = CHARACTER_FILL;

        }
        // Set center point to be 'pointer' character signifying the robot no matter what
        display[11][11] = CHARACTER_POINTER;
        // Convert char[][] to String[]
        String[] display_string = new String[42];
        for (int i = 0; i < display.length; i++) {
            display_string[i] = String.valueOf(display[i]);
        }
        return display_string;
    }
    // For exporting and graphing in python
    public String getPointsAsTXT() {
        StringBuilder points_string = new StringBuilder();
        for (Vector2d point : points) {
            points_string.append(point.getX());
            points_string.append(',');
            points_string.append(point.getY());
            points_string.append('\n');
        }
        return points_string.toString();
    }

    private double norm(double value, double min, double max) {
        return (value - min) / (max - min);
    }
    private double lerp(double norm, double min, double max) {
        return (max - min) * norm + min;
    }
    private double map(double value, double srcMin, double srcMax, double destMin, double destMax) {
        return lerp(norm(value, srcMin, srcMax), destMin, destMax);
    }

    // Currently coded so each pixel == an inch with no scaling, I just want it to work first.
    public Mat renderPointsOntoSquareImage(Vector2d reference_position, int img_side_length) {
        int half_img_side_length = img_side_length/2;
        Mat RGB_map = new Mat(img_side_length, img_side_length, CvType.CV_8UC3);
        Point img_center_point = new Point(half_img_side_length, half_img_side_length);
        List<Vector2d> relative_points_to_render = new ArrayList<>();

        for (Vector2d point : points) {
            Vector2d point_relative = point.minus(reference_position);
            if (point_relative.getX() > half_img_side_length || point_relative.getY() > half_img_side_length) {
                continue;
            }
            relative_points_to_render.add(point_relative);
        }
        for (Vector2d point : relative_points_to_render) {
            double x_rel = point.getX();
            double y_rel = point.getY();
            int mapped_x = (int) Math.round(map(x_rel, -half_img_side_length, half_img_side_length, 0, img_side_length));
            int mapped_y = (int) Math.round(map(y_rel, -half_img_side_length, half_img_side_length, 0, img_side_length));
            RGB_map.put(mapped_y, mapped_y, point_color);
        }
        Imgproc.circle(RGB_map, img_center_point, 5, new Scalar(point_color), Imgproc.FILLED);
        return RGB_map;
    }
}
