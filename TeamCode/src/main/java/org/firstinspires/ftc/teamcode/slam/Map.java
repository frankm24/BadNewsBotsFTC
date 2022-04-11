package org.firstinspires.ftc.teamcode.slam;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.ArrayList;
import java.util.List;


public class Map {
    private List<Pose2d> points;

    public Map(List<Pose2d> initial_points) {
        points = initial_points;
    }

    public Map() {
        points = new ArrayList<>();
    }

    public void addPoints(List<Pose2d> points) {
        this.points.addAll(points);
    }

    public void deletePoints(List<Pose2d> points) {
        this.points.removeAll(points);
    }

    public List<Pose2d> getPoints() {
        return this.points;
    }
}
