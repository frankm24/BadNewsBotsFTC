package badnewsbots.ultimategoal;

@Deprecated
public class Points {
    public double distanceFormula(double x1, double y1, double x2, double y2) {
        return Math.sqrt( Math.pow(x2-x1, 2) + Math.pow(y2-y1, 2) );
    }
    public boolean isInDeadzone(double input) {
        if ((input > -0.1) && (input < 0.1)) {
            return true;
        }
        else {
            return false;
        }
    }
}
