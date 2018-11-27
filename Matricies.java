public class Matricies {
    public float[][] forward = vector(0, 1);
    public float angle = 0;
    private Matricies instance;

    public Matricies(){

    }

    public static Matricies getInstance(){
        if (instance == null) {
            instance = new Matricies();
        }
        return instance;
    }

    public void updateRobotPosition(){

    }

    public static int[][] vector(int x, int y) {
        return {{x}, {y}};
    } //overload this for floats too

    public static int[][] multiplyMatricies(int[][] a, int[][] b) {
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

    public static float[][] multiplyMatricies(float[][] a, float[][] b) { //overload for floats
        float[][] p = new float[a.length][b[0].length];
        for (int row = 0; row < a.length; row++) {
            for (int col = 0; col < b[0].length; col++) {
                float s = 0;
                for (int i = 0; i < b.length; i++) {
                    s += a[row][i] * b[i][col];
                }
                p[row][col] = s;
            }
        }
        return p;
    }

    /*
    we can use matricies as vectors and transform them using matrix multiplycation using a traansform matrix such as
    [cos(a) -sin(a)]
    [sin(a)  cos(a)]
    to rotate the vector.
    */
}