package org.firstinspires.ftc.teamcode.slam;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.apache.commons.math3.geometry.Point;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.ConcurrentModificationException;
import java.util.List;


public class Map {
    private double DUPLICATE_THRESHOLD;
    private List<Vector2d> points;
    private List<Vector2d> last_points;
    private DistanceUnit unit;
    private final char CHARACTER_EMPTY = ' ';
    private final char CHARACTER_FILL = '@';
    private final char CHARACTER_POINTER = '^';

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

    /*
    public void removeDuplicatePoints() {
        for (Vector2d currentPoint : points) {
            for (Vector2d point: points) {
                if (Math.abs(point.getX()) <= DUPLICATE_THRESHOLD || Math.abs(point.getY()) <= DUPLICATE_THRESHOLD) {
                    Vector2d new_point = currentPoint.plus(point).div(2);
                    points.remove(point);
                    points.remove(currentPoint);
                    points.add(new_point);
                }
            }
        }
    }
     */
    public String[] renderASCII(Vector2d reference_position, double range) {
        //defualt range: 21
        List<Vector2d> points_to_render = new ArrayList<>();

        for (Vector2d point : points) {
            if (reference_position.distTo(point) > range) {
                continue;
            }
            Vector2d point_relative = point.plus(reference_position);
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
            if (Math.abs(rounded_x) <= range && Math.abs(rounded_y) <= range) {
                display[rounded_y][rounded_x] = CHARACTER_FILL;
            }
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
    public String getPointsAsCSV() {
        StringBuilder points_string = new StringBuilder();
        for (Vector2d point : points) {
            points_string.append('"');
            points_string.append(point.getX());
            points_string.append(',');
            points_string.append(point.getY());
            points_string.append('"');
            points_string.append(',');
        }
        return points_string.toString();
    }
}
