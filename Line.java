package org.firstinspires.ftc.teamcode;


public class Line {
    /*
    potential optimization:
    use the formula ax + by = c instead so to avoid slope which will become infinity if the line happens to be vertical
    */
    public double m; //the formula for a line is mx+b
    public double b;
    
    public Line(double m, double b) {
        this.m = m;
        this.b = b;
    }
    
    public Line pointSlope(double[][] point, double m) {
        double b = point[1][0] - m*point[0][0];
        return new Line(m, b);
    } //creates new line using point slope method.
    
    public Line pointPoint(double[][] pointA, double[][] pointB) {
        double[][] delta = new double[2][1];
        delta[0][0] = pointA[0][0] - pointB[0][0];
        delta[1][0] = pointA[1][0] - pointB[1][0];
        double m = delta[1][0]/delta[0][0];
        return pointSlope(pointA, m);
    } //creates new line using two points.
    
    public double[][] intesect(Line a, Line b) {
        //y = m1 * x + b1 & y = m2 * x + b2 solve for x and y in terms of m1 m2 b1 and b2
        double y = (b.b*a.m/b.m-a.b)/(a.m/b.m-1);
        double x = (y - a.b)/a.m;
        return Matricies.vector(x, y);
    } //calculates the intesection point of two lines
    
    
    
    
    
}
