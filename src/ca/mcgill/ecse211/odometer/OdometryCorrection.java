/*
 /*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;


public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private Odometer odometer;

  // initializing color/light sensor
 static Port lightSensor = LocalEV3.get().getPort("S1");
 static SensorModes ls = new EV3ColorSensor(lightSensor);
 static SampleProvider myColorSample = ls.getMode("Red");
 static float[] sampleColor = new float[ls.sampleSize()];
  
  //counter for black lines along X and Y axis
 int xCounter = -1;
 int yCounter = -1;
 
  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection() throws OdometerExceptions {

    this.odometer = Odometer.getOdometer();

  }

  /**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;

    while (true) {
      correctionStart = System.currentTimeMillis();

      // TODO Trigger correction (When do I have information to correct?)
      // TODO Calculate new (accurate) robot position
      myColorSample.fetchSample(sampleColor,0); // get the sample color light from sensor
      double lightDensity = (sampleColor[0]);
      
      if(lightDensity < 0.36){
    	  Sound.beep(); // beep when cross black line
    	  double theta = odometer.getTheta();
    	  double tileWidth = 30.48;
    	  
    	  if(theta >= 345 && theta < 359.999 || theta >= 0 && theta < 10) {

    		yCounter++;
    		odometer.setY(yCounter*tileWidth);
    	  }
    	  
    	  else if(theta >= 80 && theta < 95){
    		 xCounter++;
    		 odometer.setX(xCounter*tileWidth);
    		 
    		  
    	  }
    	  
    	  else if(theta >= 170 && theta <185){
    		 
      		odometer.setY(yCounter*tileWidth);
      		 yCounter--;
    	  }
    	  
    	  else if ( theta >= 265 && theta < 275) {
     		 
     		 odometer.setX(xCounter*tileWidth);
     		xCounter--;
     		  
     	  }
    	  
    	  
    	  
      }
      // TODO Update odometer with new calculated (and more accurate) vales

     // odometer.setXYT(0.3, 19.23, 5.0);

      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
  }
}
