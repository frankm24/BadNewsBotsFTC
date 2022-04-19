package org.firstinspires.ftc.teamcode.slam;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.apache.commons.math3.geometry.Point;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;


public class Map {
    private double DUPLICATE_THRESHOLD;
    private List<Vector2d> points;
    private List<Vector2d> last_points;
    private DistanceUnit unit;

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
        return this.points;
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
}
