package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

  private final int bandCenter;
  private final int bandwidth;
  private final int motorLow;
  private final int motorHigh;
  private int distance;

  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {
    this.distance = distance;
    
    // TODO: process a movement based on the us distance passed in (BANG-BANG style)

    double currentdistance = distance;
 
    // from the wall and the counter is used to avoid the gaps
    if (currentdistance > 60){ 
     int x = 0;
     while (currentdistance > 60) {
      x++;
      if (x >= 17) {
       x = 0;
       break;
      } 
      else {
       //  outer motor speed up and  the inner motor slows down
      WallFollowingLab.rightMotor.setSpeed(motorHigh + 100); 
         WallFollowingLab.leftMotor.forward();
         WallFollowingLab.rightMotor.forward();
         WallFollowingLab.leftMotor.setSpeed(motorLow);

     }
    }
     
    }
    
    else if (currentdistance < 40){
     
     //  inner motor speeds up  and  outer motor to speeds down
     WallFollowingLab.rightMotor.setSpeed(motorLow);
      WallFollowingLab.leftMotor.setSpeed(motorHigh + 200); 
      
      WallFollowingLab.rightMotor.forward();
      WallFollowingLab.leftMotor.forward();
 
     
     }
    
     else{
      // forward 
    	 WallFollowingLab.leftMotor.forward();
         WallFollowingLab.rightMotor.forward();
    	 WallFollowingLab.leftMotor.setSpeed(motorHigh); 
         WallFollowingLab.rightMotor.setSpeed(motorHigh);
        
     }
      
  }
  

  @Override
  public int readUSDistance() {
    return this.distance;
  }
}
