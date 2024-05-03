package org.usfirst.frc.team6632.robot; 
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.buttons.*;
import edu.wpi.first.wpilibj.livewindow.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import com.kauailabs.navx.frc.*;

public class Robot extends IterativeRobot 
{
	@SuppressWarnings("deprecation")
	RobotDrive myRobot;
	AHRS gyro;
	
	//Speed controllers for drive motors
	VictorSP vspFrontLeft, vspRearLeft, vspFrontRight, vspRearRight;
	
	//Joystick variables
	Joystick joyController1;
	JoystickButton joyButtonA, joyButtonB, joyButtonX, joyButtonY, joyButtonStart, joyButtonLB, joyButtonRB;
	
	//Speed controllers for climb motors
    Spark climbMotor1, climbMotor2; 
    
    PowerDistributionPanel pdp;
	
	//Drive speed info
	private double currentVelocity = 0.0d;
	private double targetVelocity = 0.0d;
	private long lastLoopTime = 0;
	
	//Pneumatics
	private Solenoid cubeIn, cubeOut;
	private Solenoid clawOpen, clawClose;
	private Solenoid clawRaise, clawLower;
	
	private boolean cubePush = false;
	private boolean clawGrab = false;
	private boolean clawButtonPressed = false;
	private long clawButtonPressTime = 0;
	private boolean clawRaised = false;
	private boolean clawRaisePressed = false;
	private long clawRaisePressTime = 0;
	private long cubeOutTime = 0;
	private long cubeButtonPressTime = 0;
	private boolean cubeButtonPressed = false;
	
	//Current state (driver, autonMode)
	private int currentDriver = 0;
	private int autonState = 0;
	private byte autonMode = 1;
	private byte mode = 0; //710
	private String fmsGameData;
	private double currentAngle;
	private int sideState = 0;

	// bounce, dBounce, cubePush and clawGrab variables are to prevent constant input of buttons
	// just press the button once, trigger the event; if button is held, event does
	// not continue to get triggered
	private boolean bounce; // auton selection
	private boolean dBounce = true; // for drivers
	
	private long autonStartTime;
	
	public static final String BASELINE = "Baseline";
	public static final String SWITCH_CENTRE = "Switch_Centre";
	public static final String SWITCH_RIGHT = "Switch_Right";
	public static final String SWITCH_LEFT = "Switch_Left";
	public static String autoMode = SWITCH_CENTRE;
	
	

	/////////////////////////////////////////////////////////////////////////////////////
	//1. ROBOT INITIALISATION -----------------------------------------------------------
	/////////////////////////////////////////////////////////////////////////////////////

	private void initSparks() {
		climbMotor1 = new Spark(5);
		climbMotor2 = new Spark(4);
	}
	
	@SuppressWarnings("deprecation")
	private void initDriveVSPs() {
		vspFrontLeft = new VictorSP(6);  //blue
		vspRearLeft = new VictorSP(7);   //purple
		vspFrontRight = new VictorSP(8); //gray
		vspRearRight = new VictorSP(9);  //white
		
		vspRearRight.setInverted(true);
		vspFrontRight.setInverted(true);
		vspRearLeft.setInverted(false); //true on main bot, false on test
		vspFrontLeft.setInverted(false); //true on main bot, false on test
		
		myRobot = new RobotDrive(vspFrontLeft, vspRearLeft, vspFrontRight, vspRearRight);
		myRobot.setSensitivity(0.3);
	}
	
	private void initController() {
		joyController1 = new Joystick(0);
		
		joyButtonA = new JoystickButton(joyController1, 1);
		joyButtonB = new JoystickButton(joyController1, 2);
		joyButtonX = new JoystickButton(joyController1, 3);
		joyButtonY = new JoystickButton(joyController1, 4);
		joyButtonStart = new JoystickButton(joyController1, 8);
		joyButtonLB = new JoystickButton(joyController1, 5);
		joyButtonRB = new JoystickButton(joyController1, 6);
	}
	
	private void initPnuematics() {
		cubeIn = new Solenoid(5);
		cubeOut = new Solenoid(4);
		clawOpen = new Solenoid(0);
		clawClose = new Solenoid(1);
		clawRaise = new Solenoid(2);
		clawLower = new Solenoid(3);
	}
	
	private void initCamera() {
		CameraServer.getInstance().startAutomaticCapture();
	}
	
	public void robotInit() { 
		initSparks();
		initDriveVSPs();
		initController();
		initPnuematics();
		initCamera();
		
		gyro = new AHRS(SerialPort.Port.kMXP);
		currentAngle = gyro.getAngle();
		pdp = new PowerDistributionPanel();
	}

	/////////////////////////////////////////////////////////////////////////////////////
	//2. AUTONOMOUS MODE ----------------------------------------------------------------
	/////////////////////////////////////////////////////////////////////////////////////

	public void autonomousInit() {
		autonStartTime = System.currentTimeMillis();
		fmsGameData = DriverStation.getInstance().getGameSpecificMessage();
		currentAngle = gyro.getAngle();
		autonState = 0;
		sideState = 0;
	}
	
	public void switchAutonStatesCenter(int state, char c) {
		if(c == 'L') {
			switch(state) {
			case 1:
				currentAngle -= 45;
				break;
			case 3:
				currentAngle += 45;
				break;
			}
		} else if(c == 'R') {
			switch(state) {
			case 1:
				currentAngle += 45;
				break;
			case 3:
				currentAngle -= 45;
				break;
			}
		}
		
	}
	
	public void switchAutonStatesSide(int state, char c) {
		if(c == 'L') {
			switch(state) {
			case 1:
				currentAngle -= 45;
				break;
			case 3:
				currentAngle += 45;
				break;
			}
		} else if(c == 'R') {
			switch(state) {
			case 1:
				currentAngle += 45;
				break;
			case 3:
				currentAngle -= 45;
				break;
			}
		}
		
	}
	
	public void switchAutonStatesLeft(int state, char c) {
		switch(state) {
		case 1:
			currentAngle -= 45;
			break;
		case 3:
			currentAngle += 45;
			break;
		}
	}
	
	private double getTurnAmount() {
		double turnAmount = (currentAngle - gyro.getAngle()) * 0.02d;
		if(Math.abs(turnAmount) < 0.04d) {
			turnAmount = 0.0d;
		}
		if(turnAmount > 0.5d) {
			turnAmount = 0.5d;
		}
		if(turnAmount < -0.5d) {
			turnAmount = -0.5d;
		}
		return turnAmount;
	}

	public void autonomousPeriodic() {
		boolean mySwitch = false;
		boolean myScale = false;
		checkAutonState();
		while(fmsGameData.length() == 0) {
			fmsGameData = DriverStation.getInstance().getGameSpecificMessage();
		}
		//////////////CHECKS IF SWITCH BELONGS TO US////////////
		if(autoMode.equals(SWITCH_CENTRE)) {
			mySwitch = true;
		}if(autoMode.equals(SWITCH_LEFT) && fmsGameData.charAt(0)=='L'){
			mySwitch = true;
		}if(autoMode.equals(SWITCH_RIGHT) && fmsGameData.charAt(0)=='R'){
			mySwitch = true;
		}
		//////////////CHECKS IF SCALE BELONGS TO US////////////
		if(autoMode.equals(SWITCH_LEFT) && fmsGameData.charAt(1)=='L'){
			myScale = true;
		}if(autoMode.equals(SWITCH_RIGHT) && fmsGameData.charAt(1)=='R'){
			myScale = true;
		}
		///////////////END CHECK////////////////////////

		///////////////ACTUAL AUTO CODE/////////////
		//NOTE AS OF RN SWITCH_LEFT and SWITCH RIGHT ARE THE SAME, EXCEPT FOR THE ANGLE; IT GOES FROM +/-45 to +/-45
		if(autoMode.equals(BASELINE)) {
			if(System.currentTimeMillis() - autonStartTime < 3000) {
				myRobot.arcadeDrive(-0.5d, 0, false);
			}else if(System.currentTimeMillis() - autonStartTime < 4000) {
				myRobot.arcadeDrive(-0.1d, 0, false);
			}
		}else if(autoMode.equals(SWITCH_LEFT)) {//Drives Straight, stops; checks if switch belongs to us then delivers; else SCALLEES
			if(System.currentTimeMillis() - autonStartTime < 2200) {
				myRobot.arcadeDrive(-0.5d, getTurnAmount(), false);
			}else if(System.currentTimeMillis() - autonStartTime < 2500) {
				myRobot.arcadeDrive(0.0d, 0, false);
			}
			else if(mySwitch){
				if(System.currentTimeMillis() - autonStartTime < 3500) {
					if(sideState == 0){
						currentAngle+=45;
						sideState = 1;
					}
					myRobot.arcadeDrive(-0.0d, getTurnAmount(), false);
				}else if(System.currentTimeMillis() - autonStartTime < 5000) {
					if(sideState == 1){
						currentAngle+=45;
						sideState = 2;
					}
					myRobot.arcadeDrive(-0.0d, getTurnAmount(), false);
				}else if(System.currentTimeMillis() - autonStartTime < 6000) {
					myRobot.arcadeDrive(-0.5d, 0, false);
				}else if(System.currentTimeMillis() - autonStartTime < 6100) {
					myRobot.arcadeDrive(0.0d, 0, false);
					setClawGrab(true);
				}else{
					setCubeOut(true);
				}
			}
			else if(myScale){
				if(System.currentTimeMillis() - autonStartTime < 4800) {
					myRobot.arcadeDrive(-0.5d, getTurnAmount(), false);
				}else if(System.currentTimeMillis() - autonStartTime < 6000) {
					if(sideState == 0){
						currentAngle+=45;
						sideState = 1;
					}
					myRobot.arcadeDrive(0d, getTurnAmount(), false);
				}else if(System.currentTimeMillis() - autonStartTime < 7000) {
					if(sideState == 1){
						currentAngle+=45;
						sideState = 2;
					}
					myRobot.arcadeDrive(0d, getTurnAmount(), false);
				}else if(System.currentTimeMillis() - autonStartTime < 9000) {
					myRobot.arcadeDrive(-0.0d, 0, false);
					climbMotor1.setSpeed(1);
					climbMotor2.setSpeed(-1);
				}else if(System.currentTimeMillis() - autonStartTime < 9100) {
					climbMotor1.setSpeed(0.15);
					climbMotor2.setSpeed(-0.15);
					myRobot.arcadeDrive(0.0d, 0, false);
					if(myScale) {
						setClawGrab(true);
					}
				}else{
					if(myScale) {
						setCubeOut(true);
					}
				}
			}
		}else if(autoMode.equals(SWITCH_RIGHT)) {//Drives Straight, stops; checks if switch belongs to us then delivers; else SCALEEEES
			if(System.currentTimeMillis() - autonStartTime < 2200) {
				myRobot.arcadeDrive(-0.5d, getTurnAmount(), false);
			}else if(System.currentTimeMillis() - autonStartTime < 2500) {
				myRobot.arcadeDrive(0.0d, 0, false);
			}
			else if(mySwitch){
				if(System.currentTimeMillis() - autonStartTime < 3500) {
					if(sideState == 0){
						currentAngle-=45;
						sideState = 1;
					}
					myRobot.arcadeDrive(-0.0d, getTurnAmount(), false);
				}else if(System.currentTimeMillis() - autonStartTime < 5000) {
					if(sideState == 1){
						currentAngle-=45;
						sideState = 2;
					}
					myRobot.arcadeDrive(-0.0d, getTurnAmount(), false);
				}else if(System.currentTimeMillis() - autonStartTime < 6000) {
					myRobot.arcadeDrive(-0.5d, 0, false);
				}else if(System.currentTimeMillis() - autonStartTime < 6100) {
					myRobot.arcadeDrive(0.0d, 0, false);
					setClawGrab(true);
				}else{
					setCubeOut(true);
				}
			}
			else if(myScale){
				if(System.currentTimeMillis() - autonStartTime < 5000) {
					myRobot.arcadeDrive(-0.5d, getTurnAmount(), false);
				}else if(System.currentTimeMillis() - autonStartTime < 6000) {
					if(sideState == 0){
						currentAngle-=45;
						sideState = 1;
					}
					myRobot.arcadeDrive(-0.1d, getTurnAmount(), false);
				}else if(System.currentTimeMillis() - autonStartTime < 7000) {
					if(sideState == 1){
						currentAngle-=45;
						sideState = 2;
					}
					myRobot.arcadeDrive(-0.1d, getTurnAmount(), false);
				}else if(System.currentTimeMillis() - autonStartTime < 9000) {
					myRobot.arcadeDrive(-0.0d, 0, false);
					climbMotor1.setSpeed(1);
					climbMotor2.setSpeed(-1);
				}else if(System.currentTimeMillis() - autonStartTime < 9100) {
					climbMotor1.setSpeed(0.15);
					climbMotor2.setSpeed(-0.15);
					myRobot.arcadeDrive(0.0d, 0, false);
					if(myScale) {
						setClawGrab(true);
					}
				}else{
					if(myScale) {
						setCubeOut(true);
					}
				}
			}

		}else{//DEFAULT CENTRE AUTO
			if(System.currentTimeMillis() - autonStartTime < 300) {
				myRobot.arcadeDrive(-0.5d, getTurnAmount(), false);//Drive straight and compensate
				SmartDashboard.putString("DB/String 3", "1");
			} else if(System.currentTimeMillis() - autonStartTime < 1000) {
				myRobot.arcadeDrive(0d, 0.0d, false);//Stop
				SmartDashboard.putString("DB/String 3", "2");
			} else if(System.currentTimeMillis() - autonStartTime < 3000) {
				if(autonState < 1) {//set turn amount and increment stage
					autonState = 1;
					switchAutonStatesCenter(1, fmsGameData.charAt(0));
				}
				myRobot.arcadeDrive(-0.25d, getTurnAmount(), false);
				SmartDashboard.putString("DB/String 3", "2");
			} else if(System.currentTimeMillis() - autonStartTime < 3600) {
				myRobot.arcadeDrive(-0.5d, getTurnAmount(), false);
				SmartDashboard.putString("DB/String 3", "3");
			} else if(System.currentTimeMillis() - autonStartTime < 4000) {
				myRobot.arcadeDrive(0d, 0.0d, false);
				SmartDashboard.putString("DB/String 3", "3");
			} else if(System.currentTimeMillis() - autonStartTime < 6000) {
				if(autonState < 3) {
					autonState = 3;
					switchAutonStatesCenter(3, fmsGameData.charAt(0));
				}
				myRobot.arcadeDrive(-0.25d, getTurnAmount(), false);
				SmartDashboard.putString("DB/String 3", "4");
			} else if(System.currentTimeMillis() - autonStartTime < 6800) {
				myRobot.arcadeDrive(-0.5d, getTurnAmount(), false);
				SmartDashboard.putString("DB/String 3", "5");
			} else if(System.currentTimeMillis() - autonStartTime < 6900) {
				myRobot.arcadeDrive(0,0, false);
				setClawGrab(true);
			} else {
				setCubeOut(true);
			}
		}
		

		SmartDashboard.putString("DB/String 1", ""+gyro.getAngle());
		SmartDashboard.putString("DB/String 2", ""+currentAngle);
	}

	public void teleopInit() {
		lastLoopTime = System.currentTimeMillis();
	}
	
	/////////////////////////////////////////////////////////////////////////////////////
	//3. TELEOP MODE  -------------------------------------------------------------------
	/////////////////////////////////////////////////////////////////////////////////////
	
	private void toggleClawRaised() {
		clawRaised = !clawRaised;
		setClawRaised(clawRaised);
	}
	
	private void setClawRaised(boolean b) {
		clawRaisePressTime = System.currentTimeMillis();
		clawRaised = b;
		clawRaise.set(clawRaised);
		clawLower.set(!clawRaised);
	}
	
	private void checkClawRaiseButton(JoystickButton b) {
		if(b.get() && !clawRaisePressed && (System.currentTimeMillis() - clawRaisePressTime) > 200) {
			clawRaisePressed = true;
			toggleClawRaised();
		} else if(!b.get()) {
			clawRaisePressed = false;
		}
	}
	
	private void toggleClawGrab() {
		clawGrab = !clawGrab;
		setClawGrab(clawGrab);
	}
	
	private void setClawGrab(boolean b) {
		clawButtonPressTime = System.currentTimeMillis();
		clawGrab = b;
		clawOpen.set(!clawGrab);
		clawClose.set(clawGrab);
	}
	
	private void checkClawGrabButton(JoystickButton b) {
		if(b.get() && !clawButtonPressed && (System.currentTimeMillis() - clawButtonPressTime) > 200) {
			clawButtonPressed = true;
			toggleClawGrab();
		} else if(!b.get()) {
			clawButtonPressed = false;
		}
	}
	
	private void setCubeOut(boolean b) {
		cubeOut.set(!b);
		cubeIn.set(b);
		cubeOutTime = System.currentTimeMillis();
	}
	
	private void checkCubeEjectButton(JoystickButton b) {
		if(b.get() && !cubeButtonPressed) {
			cubeButtonPressed = true;
			clawButtonPressTime = System.currentTimeMillis();
			cubeButtonPressTime = System.currentTimeMillis();
			setClawGrab(true);
			
		} else if(!b.get()) {
			cubeButtonPressed = false;
		}
	}
	
	private void checkCubeEjectReady() {
		if(System.currentTimeMillis() - cubeButtonPressTime > 100 && System.currentTimeMillis() - cubeButtonPressTime < 500) {
			setCubeOut(true);
		}
	}
	
	private void checkCubeEjectTimeout() {
		if(System.currentTimeMillis() - cubeOutTime > 500) {
			setCubeOut(false);
		}
	}
	
	private void setClimbMotors() {
		if(joyButtonLB.get()) {//down
			climbMotor1.setSpeed(-.8);
			climbMotor2.setSpeed(.8);
		}
		else if(joyButtonRB.get()) {//up
			climbMotor1.setSpeed(1);
			climbMotor2.setSpeed(-1);
		}
		else {//stay
			climbMotor1.setSpeed(0.15);
			climbMotor2.setSpeed(-0.15);
		}
		if(joyButtonY.get()){//down fast
			climbMotor1.setSpeed(-1);
			climbMotor2.setSpeed(1);
		}
	}
	
	private void runDriveMotors() {
		//input collection
		/*	axisInputs[0] is the forwards values, range [-1, 1]
			axisInputs[1] is the rotation value, range [-1, 1]
			axisInputs[2] is the drive power (speed), range [0, 1]
			axisInputs[3] is the climb power (speed), range [0, 1]*/
		double[] axisInputs = new double[4];
		/*	There are multiple drivers
			data for axisInputs comes from a joy-stick axis
			the axis number is given in this array*/
		final int[][] driverAxisInputMaps = {
				{ 1, 4, 2, 3 },
				{ 1, 4, 2, 3 }
		};
		for(int i = 0; i < driverAxisInputMaps[currentDriver].length; i++) {
			axisInputs[i] = joyController1.getRawAxis(driverAxisInputMaps[currentDriver][i]);
		}
		//calculate robot motor output power
		//There is a base threshold power that is active when the trigger is not pressed

		final double baseThresholdPower = 0.4;

		//Calculate drive speed by taking the max of the absolute values of the forward speed and the turn speed
		//this is to prevent the robot from not turning due to too low an input speed

		//final double baseTurnPower = 0.3d;
		//double maxTurnPower = Math.abs(currentVelocity) * (baseTurnPower - 1.0d) + 1.0d;
		double baseTurnPower = 0.5d;
		double turnAmount = axisInputs[1];//(baseTurnPower + ((1.0d - baseTurnPower) * axisInputs[2])) * axisInputs[1];
//.6 * axisInputs[1] * (baseThresholdPower + ((1.0d - baseThresholdPower) * axisInputs[2]));

		//double robotDriveValue = Math.max(Math.abs(axisInputs[0]), Math.abs(turnAmount)) * Math.signum(axisInputs[0]);
		targetVelocity = (baseThresholdPower + ((1.0d - baseThresholdPower) * axisInputs[2])) * axisInputs[0];

		//myRobot.setMaxOutput(baseThresholdPower + (1.0d - baseThresholdPower) * axisInputs[2]);

		long loopDelta = System.currentTimeMillis() - lastLoopTime;
		lastLoopTime = System.currentTimeMillis();
		final double driveScaleValue = 0.005d;

		if(Math.abs(targetVelocity - currentVelocity) < 0.05d) {

		} else if(targetVelocity > currentVelocity) {
			currentVelocity += driveScaleValue * loopDelta;
		} else if(targetVelocity < currentVelocity) {
			currentVelocity -= driveScaleValue * loopDelta;
		}
		myRobot.arcadeDrive(currentVelocity, turnAmount, false);
		
		SmartDashboard.putString("DB/String 5", ""+pdp.getCurrent(0));
		SmartDashboard.putString("DB/String 6", ""+pdp.getCurrent(1));
		SmartDashboard.putString("DB/String 7", ""+pdp.getCurrent(2));
		SmartDashboard.putString("DB/String 8", ""+pdp.getCurrent(3));
		SmartDashboard.putString("DB/String 9", ""+pdp.getCurrent(13));
	}

	public void teleopPeriodic() {
		
		runDriveMotors();
		
		setClimbMotors();
		
		//Pnuematics, you can change piston button configuration here
		checkClawGrabButton(joyButtonX);
		checkCubeEjectButton(joyButtonB);
		checkClawRaiseButton(joyButtonA);
		checkCubeEjectReady();
		checkCubeEjectTimeout();
		SmartDashboard.putString("DB/String 1", ""+gyro.getAngle());
		SmartDashboard.putString("DB/String 2", ""+currentAngle);
	}
	
	public void testPeriodic() 
	{
		LiveWindow.run();
	}
	public void disabledInit(){
	}

	///////////////////////////////////////////////////////////////////////////////
	//4. DISABLED PERIODIC---------------------------------------------------------
	///////////////////////////////////////////////////////////////////////////////
	public void disabledPeriodic() 
	{
		checkAutonState();
	}
	public void autonomousDisabled() {
		checkAutonState();
	}
	public void checkAutonState(){
		/*if(SmartDashboard.getBoolean("DB/Button 1", true)){
			SmartDashboard.putString("DB/String 4", "CENTRE AUTO");
			autoMode = SWITCH_CENTRE;
		}
		else{
			SmartDashboard.putString("DB/String 4", "BASELINE AUTO");
			autoMode = BASELINE;
		}*/
		if(SmartDashboard.getBoolean("DB/Button 1", true)){
			SmartDashboard.putString("DB/String 4", "Left Priority SWITCH");
			autoMode = SWITCH_LEFT;
		}
		else if(SmartDashboard.getBoolean("DB/Button 2", true)){
			SmartDashboard.putString("DB/String 4", "Centre SWITCH AUTO");
			autoMode = SWITCH_CENTRE;
		}
		else if(SmartDashboard.getBoolean("DB/Button 3", true)){
			SmartDashboard.putString("DB/String 4", "Right Priority SWITCH");
			autoMode = SWITCH_RIGHT;
		}
		else if (!((SmartDashboard.getBoolean("DB/Button 1", true) && SmartDashboard.getBoolean("DB/Button 2", true)) && SmartDashboard.getBoolean("DB/Button 3", true))){
			SmartDashboard.putString("DB/String 4", "Baseline AUTO");
			autoMode = BASELINE;
		}
	}
}
