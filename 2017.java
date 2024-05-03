//Formatted by Salmaan Khan
//Modified by Amolik Singh
//Further modified/formatted by Daniel Martchenkov (also Amolik's grammar corrected)

package org.usfirst.frc.team6632.robot; 
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Encoder;


public class Robot extends IterativeRobot 
{
  CameraServer server;
  RobotDrive myRobot;
  
  VictorSP vspFrontLeft, vspRearLeft, vspFrontRight, vspRearRight;
  Joystick joyController1;
  JoystickButton joyButtonA, joyButtonB, joyButtonX, joyButtonY, joyButtonStart;
  
  private byte step = 0;
  private byte autonMode = 1;
  private byte mode = 0; //710
  
  // bounce variables and temp variables are to prevent constant input of buttons
  // just press the button once, trigger the event; if button is held, event does
  // not continue to get triggered
  private boolean bounce; // auton selection
  private boolean bounce2; // intake
  private boolean bounce3; // climber
  private boolean dBounce = true; // for drivers
  private boolean temp = false; // switching for back door
  private boolean temp2 = false; // intake
  private boolean temp3 = false; // climber
  private boolean driver = false; // for switching between erfan and michael
  private boolean triggerClimb = false; // if you want to use the triggers to affect climb speed
  private boolean on = false;
  
  //private ADXRS450_Gyro gyroTest;
  
  AnalogInput ai = new AnalogInput(1); // ultrasonic sensor
  
  Spark intakeMotor, climbMotor1, climbMotor2;
  
  private long autonStartTime;
  private long gearBackUpTime;
  
  private double sens;
  private double turn = 0.3;
  private double x = 250;
  
  private Solenoid lowGear;
  private Solenoid highGear;
  
  //////////////////////////////////////////////////////////////////////////MAKE SURE THE FOLLOWING ARE WIRED TO THE CORRECT PINS
  //Encoder sparkyEnc = new Encoder(0, 1, false, Encoder.EncodingType.k4X); // was for platform
  

/////////////////////////////////////////////////////////////////////////////////////
//1. ROBOT INITIALISATION -----------------------------------------------------------
/////////////////////////////////////////////////////////////////////////////////////

  public void robotInit() 
  { 
    intakeMotor = new Spark (5); // intake
    climbMotor1 = new Spark (6); // just pick the pin it's wired into - climber
    climbMotor2 = new Spark (7); // just pick the pin it's wired into - climber
    
    joyController1 = new Joystick(0);
    
    vspFrontLeft = new VictorSP(0);
    vspRearLeft = new VictorSP(1);
    vspFrontRight = new VictorSP(2);
    vspRearRight = new VictorSP(3);
    vspRearRight.setInverted(true);
    vspFrontRight.setInverted(true);
    vspRearLeft.setInverted(true);
    vspFrontLeft.setInverted(true);
    
    joyButtonA = new JoystickButton(joyController1, 1);
    joyButtonB = new JoystickButton(joyController1, 2);
    joyButtonX = new JoystickButton(joyController1, 3);
    joyButtonY = new JoystickButton(joyController1, 4);
    joyButtonStart = new JoystickButton(joyController1, 8);
    
    myRobot = new RobotDrive(vspFrontLeft, vspRearLeft, vspFrontRight, vspRearRight);
    
    myRobot.setSensitivity(0.3);
    
    this.lowGear = new Solenoid(0);
    this.highGear = new Solenoid(1);
    
    //gyroTest = new ADXRS450_Gyro();
    //gyroTest.reset();
    
    CameraServer.getInstance().startAutomaticCapture();
  }
  
  ///////////////////////////////////////////////////////////////////////////////
  //2. AUTONOMOUS INITIALISATION-------------------------------------------------
  ///////////////////////////////////////////////////////////////////////////////

  public void autonomousInit() 
  {
    this.autonStartTime = System.currentTimeMillis();
  }
  
  ///////////////////////////////////////////////////////////////////////////////
  //3. AUTONOMOUS PERIODIC-------------------------------------------------------
  ///////////////////////////////////////////////////////////////////////////////

  public void autonomousPeriodic() 
  {
    this.myRobot.setMaxOutput(0.2);
    
    this.myRobot.setMaxOutput(0.45);
	
	// are we done this step?
	/*if(System.currentTimeMillis() - autonStartTime < 3000)
	{
		this.myRobot.drive(-1.0, (angle / 90.0) * -1.0);
		//this.myRobot.drive(-1.0, 0);
	}
	else {
		this.myRobot.stopMotor();
	}*/

    //double angle = gyroTest.getAngle();
    //SmartDashboard.putString("DB/String 6", angle + "");
    
    if (this.autonMode == 0) //drive to centre gear
    { 
      this.myRobot.setMaxOutput(0.35);
      
      // are we done this step?
      if(System.currentTimeMillis() - autonStartTime < 995) //+0.33
      {
        //this.myRobot.drive(-1.0, (angle / 90.0) * -1.0);
    	  this.myRobot.drive(-1.0, 0.0);
      }
      else
      {
    	  if (System.currentTimeMillis() - autonStartTime > 2000 && System.currentTimeMillis() - autonStartTime < 14000 )
    	  {
	    	  this.lowGear.set(!temp); // low gear
	      	  this.highGear.set(temp);
    	  }
    	  else
    	  {
    		  this.lowGear.set(temp); // low gear
	      	  this.highGear.set(!temp);
    	  }
      }
    }
    else if (this.autonMode == 1) //drive to baseline
    { 
      this.myRobot.setMaxOutput(0.35);
      
      // are we done this step?
      if(System.currentTimeMillis() - autonStartTime < 990) //+0.33
      {
        //this.myRobot.drive(-1.0, (angle / 90.0) * -1.0);
    	  this.myRobot.drive(-1.0, 0.0);
      }
      else
      {
    	  /*this.vspFrontLeft.set(0);
          this.vspRearLeft.set(0);
          this.vspFrontRight.set(0);
          this.vspRearRight.set(0);*/
    	  this.myRobot.stopMotor();
      }
    }
    else if (this.autonMode == 2) //left side gear turn
    { 
      this.myRobot.setMaxOutput(0.35);
      
      // are we done this step?
      if(System.currentTimeMillis() - autonStartTime < 990) //+0.33
      {
        //this.myRobot.drive(-1.0, (angle / 90.0) * -1.0);
    	  this.myRobot.drive(-1.0, 0.0);
      }
      else if (System.currentTimeMillis() - autonStartTime > 1990 && System.currentTimeMillis() - autonStartTime < 1990+x)
      {
    	  this.vspFrontLeft.set(-0.3);
          this.vspRearLeft.set(-0.3);
          this.vspFrontRight.set(-0.3);
          this.vspRearRight.set(-0.3);
      }
      else if (System.currentTimeMillis() - autonStartTime > 1990+x && System.currentTimeMillis() - autonStartTime < 2990+x)
      {
    	  this.myRobot.stopMotor();
      }
      else if (System.currentTimeMillis() - autonStartTime > 2990+x && System.currentTimeMillis() - autonStartTime < 3990+x)
      {
    	  this.myRobot.drive(-1.0, 0.0);
      }
      else if (System.currentTimeMillis() - autonStartTime > 3990+x)
      {
    	  if (System.currentTimeMillis() - autonStartTime > 2000 && System.currentTimeMillis() - autonStartTime < 14000 )
    	  {
	    	  this.lowGear.set(!temp); // low gear
	      	  this.highGear.set(temp);
    	  }
    	  else
    	  {
    		  this.lowGear.set(temp); // low gear
	      	  this.highGear.set(!temp);
    	  }
      }
      else
    	  this.myRobot.stopMotor();
    }
    else if (this.autonMode == 3) //right side gear turn
    { 
      this.myRobot.setMaxOutput(0.35);
      
      // are we done this step?
      if(System.currentTimeMillis() - autonStartTime < 990) //+0.33
      {
        //this.myRobot.drive(-1.0, (angle / 90.0) * -1.0);
    	  this.myRobot.drive(-1.0, 0.0);
      }
      else if (System.currentTimeMillis() - autonStartTime > 1990 && System.currentTimeMillis() - autonStartTime < 1990+x)
      {
    	  this.vspFrontLeft.set(0.3);
          this.vspRearLeft.set(0.3);
          this.vspFrontRight.set(0.3);
          this.vspRearRight.set(0.3);
      }
      else if (System.currentTimeMillis() - autonStartTime > 1990+x && System.currentTimeMillis() - autonStartTime < 2990+x)
      {
    	  this.myRobot.stopMotor();
      }
      else if (System.currentTimeMillis() - autonStartTime > 2990+x && System.currentTimeMillis() - autonStartTime < 3990+x)
      {
    	  this.myRobot.drive(-1.0, 0.0);
      }
      else if (System.currentTimeMillis() - autonStartTime > 3990+x)
      {
    	  if (System.currentTimeMillis() - autonStartTime > 2000 && System.currentTimeMillis() - autonStartTime < 14000 )
    	  {
	    	  this.lowGear.set(!temp); // low gear
	      	  this.highGear.set(temp);
    	  }
    	  else
    	  {
    		  this.lowGear.set(temp); // low gear
	      	  this.highGear.set(!temp);
    	  }
      }
      else
    	  this.myRobot.stopMotor();
    }
    else if (this.autonMode == 4)
    {
    	if (System.currentTimeMillis() - autonStartTime < x)
    	{
    		this.vspFrontLeft.set(0.3);
            this.vspRearLeft.set(0.3);
            this.vspFrontRight.set(0.3);
            this.vspRearRight.set(0.3);
    	}
    	else if (System.currentTimeMillis() - autonStartTime > x && System.currentTimeMillis() - autonStartTime < 1000+x)
    	{
    		this.myRobot.stopMotor();
    	}
    	else if (System.currentTimeMillis() - autonStartTime > 1000+x && System.currentTimeMillis() - autonStartTime < 2000+x)
    	{
    		this.myRobot.drive(-1.0, 0.0);
    	}
    	else
    	{
    		this.myRobot.stopMotor();
    	}
    }
    
    /*else if (this.autonMode == 1) //deliver gear to left peg
    { 
      //double angle = gyroTest.getAngle();
      this.myRobot.setMaxOutput(0.30);
      sample();
      if (step == 0)
      {

          if (System.currentTimeMillis() - autonStartTime < 942)
        	  this.myRobot.drive(-1.0, (angle / 90.0) * -1.0);
          else
        	  step++;
      }
      else if (step == 1)
      {
        this.vspFrontLeft.set(-0.3);
        this.vspRearLeft.set(-0.3);
        this.vspFrontRight.set(-0.3);
        this.vspRearRight.set(-0.3);
        if (angle <= -66)//adjust angle towards peg
          step++;
      }
      else if (step == 2)
      {
        this.myRobot.drive(-1.0, (angle / 90.0) * -1.0); // this needs to stop after delivering the gear
        if (System.currentTimeMillis() - autonStartTime > 3000)
            step++;
      }
    } 
    
    else if (this.autonMode == 2) //deliver gear to right peg
    { 
      //double angle = gyroTest.getAngle();
      this.myRobot.setMaxOutput(0.35);
      sample();
      if (step == 0)
      {
        this.myRobot.drive(1.0, (angle / 90.0) * -1.0);
        if (this.sens > 242)
          step++;
      }
      else if (step == 1)
      {
        this.vspFrontLeft.set(0.3);
        this.vspRearLeft.set(0.3);
        this.vspFrontRight.set(0.3);
        this.vspRearRight.set(0.3);
        if (angle >= 110)//adjust angle towards peg
          step++;
      }
      else if (step == 2)
      {
        this.myRobot.drive(-1.0, (angle / 90.0) * -1.0); // this needs to stop after delivering the gear
      }
    } 
    else if (this.autonMode == 3) //deliver gear to centre peg
    {
      //double angle = gyroTest.getAngle();
      this.myRobot.setMaxOutput(0.35);
      sample();
      if(step == 0)
      {
        if (this.sens < 126) 
        {
          this.myRobot.stopMotor();
          gearBackUpTime = System.currentTimeMillis();
          step++;
        }
        else
          this.myRobot.drive(-1.0, (angle / 90.0) * -1.0); 
      }
      else if (step == 1)
      {
        if (System.currentTimeMillis() - gearBackUpTime >= 1000) // modify this depending on how long it takes for the robot to stop 
        {
          step++;
          gearBackUpTime = System.currentTimeMillis();	
        }
      }
      else if (step==2)
      {
        if(System.currentTimeMillis() - gearBackUpTime <= 1000)
        {
          this.vspFrontLeft.set(-0.3);
          this.vspRearLeft.set(-0.3);
          this.vspFrontRight.set(0.3);
          this.vspRearRight.set(0.3);
        }
        else
          this.myRobot.stopMotor();
      }
    }*/
  }

  ///////////////////////////////////////////////////////////////////////////////
  //4. TELEOP PERIODIC-----------------------------------------------------------
  ///////////////////////////////////////////////////////////////////////////////

  public void teleopPeriodic() 
  {
    // the two drivers are michael and erfan and they prefer different triggers for driving
    // they also prefer different buttons for the intake and the back door
    double power = 0, powerClimb = 0, moveValue, rotateValue;
    boolean curve = false;
    

    //double angle = gyroTest.getAngle();
    //SmartDashboard.putString("DB/String 6", angle + "");
    
    moveValue = joyController1.getRawAxis(1);
    rotateValue = joyController1.getRawAxis(4);
    
    if (driver)
    {
      //////////// MICHAEL CODE
      power = joyController1.getRawAxis(2); // left trigger for speed
      powerClimb = joyController1.getRawAxis(3); // right trigger for climb
    }
    else if (!driver) 
    {
      /////////// ERFAN CODE
      power = joyController1.getRawAxis(3); // right trigger for speed
      powerClimb = joyController1.getRawAxis(2); // left trigger for climb
    }
    
    if (power > 0)
      myRobot.setMaxOutput(0.2 + 0.8 * power); // Increase by power
    else
      myRobot.setMaxOutput(0.25);
    
    if (!curve) 
    {
      if (moveValue > 0)
        myRobot.drive(Math.max(moveValue, Math.abs(rotateValue)), rotateValue);
      else
        myRobot.drive(-(Math.max(Math.abs(moveValue), Math.abs(rotateValue))), rotateValue);
    } 
    else
    {
      if (rotateValue > 0.01) 
      {
        if (moveValue > 0)
        {
          myRobot.drive(Math.max(moveValue, Math.abs(rotateValue)), rotateValue);
          this.vspFrontRight.set(-turn * rotateValue);
          this.vspRearRight.set(-turn * rotateValue);
        }
        else
        {
          myRobot.drive(-(Math.max(Math.abs(moveValue), Math.abs(rotateValue))), rotateValue);
          this.vspFrontRight.set(turn * rotateValue);
          this.vspRearRight.set(turn * rotateValue);
        }
      }
      else if (rotateValue < -0.01)
      {
        if (moveValue > 0)
        {
          myRobot.drive(Math.max(moveValue, Math.abs(rotateValue)), rotateValue);
          this.vspFrontLeft.set(turn * rotateValue);
          this.vspRearLeft.set(turn * rotateValue);
        }
        else 
        {
          myRobot.drive(-(Math.max(Math.abs(moveValue), Math.abs(rotateValue))), rotateValue);
          this.vspFrontLeft.set(-turn * rotateValue);
          this.vspRearLeft.set(-turn * rotateValue);
        }
      } 
      else
      {
        if (moveValue > 0)
          myRobot.drive(Math.max(moveValue, Math.abs(rotateValue)), rotateValue);
        else
          myRobot.drive(-(Math.max(Math.abs(moveValue), Math.abs(rotateValue))), rotateValue);
      }
    }
    
    sample();
    
    // ------- pneumatics -------------------- enables pneumatics
    if(joyButtonA.get()) 
    {
    	if (bounce)
	    {
	      this.lowGear.set(temp); // low gear
	      this.highGear.set(!temp);
	      bounce = false;
	      temp = !temp;
    	}
    }
    else if(!bounce)
    {
      bounce = true;
    }
    
    SmartDashboard.putString("DB/String 2", temp + "");
    
    // ------- intake -------------------- turns on and off intake
    /*if(intake && bounce2) 
    {
      if (temp2)
        intakeMotor.setSpeed(1.0);
      else if (!temp2)
        intakeMotor.setSpeed(0.0);
      bounce2 = false;
      temp2 = !temp2;
    }
    else if(!bounce2)
    {
      bounce2 = true;
    }*/
    
    // ------- climber -------------------- turns on and off the climber
    if(joyButtonB.get()) 
    	on = true;
    else if (joyButtonY.get())
    	on = false;
    
    if (on)
    {
   	 climbMotor1.setSpeed(0.5+0.5*powerClimb);//switch  values based on wiring
     climbMotor2.setSpeed(0.5-0.5*powerClimb);//switch  values based on wiring
    }
    else
    {

      	 climbMotor1.setSpeed(0.0);//turns off motor
        climbMotor2.setSpeed(0.0);//turns off motor
    }
    /*
    if
    {
    	
    }
      if (temp3)
      {
       
        triggerClimb = true;
      }
      else if (!temp3)
      {
        climbMotor1.setSpeed(0.0);
        climbMotor2.setSpeed(0.0);
        triggerClimb = false;
      }
      bounce3 = false;
      temp3 = !temp3;
    }
    else if(!bounce3)
    {
      bounce3 = true;
    }
    
    if (triggerClimb) // if Button Y was pressed, constantly update the climber's power based on triggers
    {
      climbMotor1.setSpeed(0.5+0.5*powerClimb);
      climbMotor2.setSpeed(-0.5-0.5*powerClimb);
    }*/
    
  }

  ///////////////////////////////////////////////////////////////////////////////
  //5. SAMPLE--------------------------------------------------------------------
	//Description: It gets the distance for the ultrasonic sensor in centimetres
  ///////////////////////////////////////////////////////////////////////////////

  public void sample() 
  {
    double voltage = ai.getVoltage(); // reads the range on the ultrasonic sensor 
    double distance = Math.round(voltage / (0.00977));
    SmartDashboard.putString("DB/String 1", Math.round(distance) + "");
    sens = Math.round(distance);
  } 
  public void testPeriodic() 
  {
    LiveWindow.run();
  }

  ///////////////////////////////////////////////////////////////////////////////
  //6. DISABLED PERIODIC---------------------------------------------------------
  ///////////////////////////////////////////////////////////////////////////////

  public void disabledPeriodic() 
  {
    if (joyButtonStart.get()) 
    {      //button 8 is "Start"
      //gyroTest.reset();
    }
    if (joyButtonB.get()) 
    {
      if (dBounce) 
      {
        dBounce = false;
        driver = !driver;
      }
    } 
    else if (!dBounce) 
    {
      dBounce = true;
    }
    double autonSelector = ((this.joyController1.getRawAxis(0)) + 1) / 2.0;
    
    if (driver)
    	SmartDashboard.putString("DB/String 4", "Michael");
    else
    	SmartDashboard.putString("DB/String 4", "Erfan");
    
    if (autonSelector > 0.95)
    {
      if (bounce)
      {
        mode++; 
        bounce = false;
      }
    }
    else if (autonSelector < 0.05)
    {
      if (bounce)
      {
        mode--; 
        bounce = false;
      }
    }
    else
      bounce = true;
    if (mode < 0)
      mode = 9;
    if (mode > 9)
      mode = 0;
    SmartDashboard.putString("DB/String 0", mode +" ");
    sample(); 
    this.autonMode = mode;       
  }   
}
