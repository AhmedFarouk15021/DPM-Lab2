package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 200;
  private static final int FILTER_OUT = 20;

  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl;

  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0;

    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {

    // rudimentary filter - toss out invalid samples corresponding to null
    // signal.
    // (n.b. this was not included in the Bang-bang controller, but easily
    // could have).
    //
    if (distance >= 255 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
    } else if (distance >= 255) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }

    // TODO: process a movement based on the us distance passed in (P style)
    
    double currentdistance = distance;
    double error = ( bandCenter - currentdistance);
    //dspeed =correction factor + (error * constant from testing)
    double dspeed; 
    
    // Far wall
    if (currentdistance>40){
    	dspeed = proportionalspeed(error);
     int x = 0;
     // delaying the response to aviod the gap
     while (error <- bandWidth) {
      x++;
      if (x >= 10) {
       x = 0;
       break;
      } 
      else {
       // outer motor speed up and inner motor slows down
    	  WallFollowingLab.leftMotor.setSpeed((int) (MOTOR_SPEED - dspeed));  
    	  
      WallFollowingLab.rightMotor.setSpeed ((int) (MOTOR_SPEED + dspeed -20));
      
                 WallFollowingLab.leftMotor.forward();
                 
         WallFollowingLab.rightMotor.forward();
         
     }
    }
     
    }
    //close wall
    else if (currentdistance < 35 ){
    	dspeed = proportionalspeed(error);
     //  inner motor speeds up  outer motor speeds down
   // we added the dspeed because the wheels speed does not change fast enough so adding it makes it faster 
      WallFollowingLab.leftMotor.setSpeed((int) (MOTOR_SPEED + 100 + dspeed)); 
      
 
      WallFollowingLab.leftMotor.setAcceleration(2000); 
      
      WallFollowingLab.rightMotor.setSpeed((int) (MOTOR_SPEED - 70- dspeed));

      WallFollowingLab.leftMotor.forward(); 
      
       WallFollowingLab.rightMotor.forward();
     //}
     
     }
    
     else{
    	 //Moves forward
    	  WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    	  
         WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Start robot moving forward
                  
         WallFollowingLab.leftMotor.forward();
         
         WallFollowingLab.rightMotor.forward();
     }


  }
  
  //speed correction
  //proportional speed: adjusts the speed when the robot gets too close or too far from the wall, the wheel speed changes.
  public double proportionalspeed(double error) {
         double dspeed2;
    if(error<-bandWidth ){ 
    	dspeed2 = ((Math.abs(error)) * 5); 
   }
   else {
	   // when robot is  close turn fast so it does not hit the wall
	   dspeed2 = (((Math.abs(error)) * 8));
   }                                      
    // limit correction
    if(dspeed2 > MOTOR_SPEED) {
    	dspeed2=100;
    }
   
   return dspeed2;
  
  }
  


  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
