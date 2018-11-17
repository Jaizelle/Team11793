package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;

/* 
this stuff is untested. I dont know how well the color sensor can detect color once it is calibrated. I probably will need to change SATURATION_LOWER_BOUND.
creating this class allows us to clear out a bunch of space in our op modes and offers a compact way to modify the code
as any modifycations made in this class will affect any opmode that uses this class.
also this is the best name i could come up with.
*/
public class ColorSensorStuff {
  public final double CUBE_HUE_UPPER_BOUND = Math.PI/6; //the bounds represent ranges of colors acceptable for a "cube". 
  public final double CUBE_HUE_LOWER_BOUND = 0; //the hue bounds are less important than the saturation bound as all objects except for the cube have any significant saturation whatsoever.
  public final double SATURATION_LOWER_BOUND = .2; //change this
  public final double ROOT = Math.sqrt(3);
  
  private int ambientr = 0; //probably should be private
  private int ambientg = 0;
  private int ambientb = 0;
  
  private ColorSensor colorSensor;
  
  public ColorSensorStuff (ColorSensor cs) {
    colorSensor = cs;
  }
  
  public void calibrate() {
    ambientr = colorSensor.red();
    ambientg = colorSensor.green();
    ambientb = colorSensor.blue();
  }
  
  public double[] getColor() {
    double[] out = new double[2];
    int r = colorSensor.red() - ambientr;
    int g = colorSensor.green() - ambientg;
    int b = colorSensor.blue() - ambientb;
    double x = r + g/2 + b/2;
    double y = ROOT * g / 2 - ROOT * b / 2;
    double h = Math.atan2(x, y);
    double s = Math.hypot(x, y);
    out[0] = h;
    out[1] = s;
    return out;
  }
  
  public boolean detectCuble() { //if the color is within the ranges set by the bounds this method will return true.
    double[] color = getColor();
    double hue = color[0];
    double sat = color[1];
    if (
        sat > SATURATION_LOWER_BOUND && 
        hue > CUBE_HUE_LOWER_BOUND && 
        hue < CUBE_HUE_UPPER_BOUND
        ) {
      return true;
    } else {
      return false;
    }
  }
}
