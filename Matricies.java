package org.firstinspires.ftc.teamcode;


public class Matricies {
    private double[][] forward;
    private double[][] distSensor;
    private double[][] distSensorOrientation;
    public double angle;
    
    private static Matricies instance;
    
    public double[][] getForward(double angle) {
        double[][] transform = rotateTransform(angle);
        return multiply(transform, forward);
    }
    
    public double[][] getDistSensorPos(double angle) {
        double[][] transform = rotateTransform(angle);
        return multiply(transform, distSensor);
    }
    
    public double[][] getDistSensorForward(double angle) { //returns a unit vector in the direction that the distance sensor is facing
        double[][] transform = rotateTransform(angle);
        return multiply(transform, distSensorOrientation);
    }
    
    public Matricies(){
        forward = multiply(rotateTransform(-Math.PI/4), vector(0.0d, 1.0d));
        distSensor = multiply(rotateTransform(-Math.PI/4), vector(0.0d, 1.0d));//need distSensor measurement. the robots forward is the +y direction and right is the +x directon.        angle = 0;
        distSensorOrientation = multiply(rotateTransform(-Math.PI/4), vector(1.0d, 0.0d));
    }

    public static Matricies getInstance(){
        if (instance == null) {
            instance = new Matricies();
        }
        return instance;
    }

    public void updateRobotPosition(){

    }

    public static int[][] vector(int x, int y) { //creates a vector using the given x and y components
        int[][] v = new int[2][1];
        v[0][0] = x;
        v[1][0] = y;
        return v;
    }
    
    public static double[][] vector(double x, double y) { //double overload
        double[][] v = new double[2][1];
        v[0][0] = x;
        v[1][0] = y;
        return v;
    }

    public static int[][] multiply(int[][] a, int[][] b) { //Multiplies two matricies using basic matrix multiplycation rules
        int[][] p = new int[a.length][b[0].length];
        for (int row = 0; row < a.length; row++) { //apparently, you use semicol0ns;;;
            for (int col = 0; col < b[0].length; col++) {
                int s = 0;
                for (int i = 0; i < b.length; i++) {
                    s += a[row][i] * b[i][col];
                }
                p[row][col] = s;
            }
        }
        return p; //i think this will work
        //it w0rks!
    }

    public static double[][] multiply(double[][] a, double[][] b) { //overload for double
        double[][] p = new double[a.length][b[0].length];
        for (int row = 0; row < a.length; row++) {
            for (int col = 0; col < b[0].length; col++) {
                double s = 0;
                for (int i = 0; i < b.length; i++) {
                    s += a[row][i] * b[i][col];
                }
                p[row][col] = s;
            }
        }
        return p;
    }
    
    public double[][] add(double[][] a, double[][] b) { //overload this for int as well maybe
        double[][] sum = new double[a.length][a[1].length];
        for (int i = 0; i < a.length; i++) {
            for (int j = 0; j < a[0].length; j++) {
                sum[i][j] = a[i][j] + b[i][j];
            }
        }
        return sum;
    }
    
    public static double[][] rotateTransform(double angle) { //returns a transform matrix that when multiplied by a vector will rotate the vector by the given angle.
        double[][] transform = new double[2][2];
        transform[0][0] = Math.cos(angle);
        transform[0][1] = Math.sin(angle);
        transform[1][0] = -Math.sin(angle);
        transform[1][1] = Math.cos(angle);
        return transform;
    }
    
    public static double[][] scale(double a, double[][] b) {//scales a matrix by a scalar
        double[][] out = new double[b.length][b[0].length];
        for (int i = 0; i < b.length; i++) {
            for (int j = 0; j < b[0].length; j++) {
                out[i][j] = a * b[i][j];
            }
        }
        return out;
    }
    /*
    we can use matricies as vectors and transform them using matrix multiplycation using a traansform matrix such as
    [cos(a) -sin(a)]
    [sin(a)  cos(a)]
    to rotate the vector.
    */
}
