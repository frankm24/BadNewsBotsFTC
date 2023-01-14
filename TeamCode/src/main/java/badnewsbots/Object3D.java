package badnewsbots;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import java.util.Arrays;
import java.util.Vector;

public abstract class Object3D {
    private Vector3D position;
    private Vector3D orientation;

    public Object3D(Vector3D position, Vector3D orientation) {
        this.position = position;
        this.orientation = orientation;
    }

    public Vector3D getPosition() {return position;}
    public void setPosition(Vector3D position) {this.position = position;}

    public Vector3D getOrientation() {return orientation;}
    public void setOrientation(Vector3D orientation) {this.orientation = orientation;}

    abstract void intersect();

}
