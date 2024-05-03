// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Servo;

import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.cameraserver.CameraServer;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

 /**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the manifest
 * file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final WPI_TalonSRX leftMotor = new WPI_TalonSRX(1);
  private final WPI_TalonSRX leftRearMotor = new WPI_TalonSRX(2);

  private final WPI_VictorSPX leftHookMotor = new WPI_VictorSPX(8);
  private final WPI_VictorSPX rightHookMotor = new WPI_VictorSPX(9);

  private final WPI_TalonSRX rightMotor = new WPI_TalonSRX(3);
  private final WPI_TalonSRX rightRearMotor = new WPI_TalonSRX(4);

  private final WPI_TalonSRX armMotor = new WPI_TalonSRX(7);

  private final WPI_TalonSRX rightIntakeMotor = new WPI_TalonSRX(5);
  private final WPI_TalonSRX leftIntakeMotor = new WPI_TalonSRX(6);

  private final Servo rearDoor = new Servo(0);       
  // private final Servo pusher = new Servo(1);

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(leftMotor, rightMotor);

  private final XboxController m_controller = new XboxController(0);

  private final Timer m_timer = new Timer();

  private final Encoder encoderLeft = new Encoder(0, 1);
  private final Encoder encoderRight = new Encoder(2,3);

  public boolean hookTop = false;
  public boolean backDoor = false;
  public boolean intake = false;
  public boolean reverse = false;
  public boolean hook = false;
  public int num;

  // private static final double whd = 6;  //inches

  //private static final double cpr = 7/4; //if am-2861a
  // private static final double cpr = 360; //if am-3132
  // private static final double cpr = 5; //if am-3314a
  // private static final double cpr = 1024; //if am-3445
  // private static final double cpr = 64; //if am-4027



  public Robot() {
    SendableRegistry.addChild(m_robotDrive, leftMotor);
    SendableRegistry.addChild(m_robotDrive, rightMotor);
    
    //armMotor.configSupplyCurrentLimit(null);
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    leftMotor.setInverted(true);
    leftRearMotor.setInverted(true);

    leftRearMotor.follow(leftMotor);
    rightRearMotor.follow(rightMotor);

    leftHookMotor.setInverted(true);

    

    // encoderLeft.setReverseDirection(true);

    // encoderLeft.setDistancePerPulse(Math.PI*whd/cpr); 
    // encoderRight.setDistancePerPulse(Math.PI*whd/cpr); 

    // encoderLeft.reset();
    // encoderRight.reset();

    CameraServer.startAutomaticCapture();
    

  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.restart();
    rearDoor.set(0);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.arcadeDrive(-0.5, 0.0, false);
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
    // double distance = encoderLeft.getDistance();
    

    // if (distance<92){
    //   m_robotDrive.arcadeDrive(0.5, 0);

    // }


  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    rearDoor.set(0 );
    // armMotor.set(1.0); 
 
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    // robot drive code
    // REMOVE THE "/2" FROM BOTH LEFT AND RIGHT
    m_robotDrive.arcadeDrive(m_controller.getLeftY() , -m_controller.getRightX()); // 

    // Climer code

    // if(m_timer.get()>125){
    //   if (m_controller.getPOV() == 180 || hook){
    //     hook = true;
    //     leftHookMotor.set(-0.5);
    //     rightHookMotor.set(-0.5);
        

    //   }
    // }
    
    SmartDashboard.putBoolean("hookTop", hookTop);
    // if (m_controller.getYButtonPressed()) {
    //   if (hookTop) {
    //     leftHookMotor.set(-1);
    //     rightHookMotor.set(-1);
    //   } else {
    //     leftHookMotor.set(1);
    //     rightHookMotor.set(1);
    //   }

    // } else if (m_controller.getYButtonReleased()) {
    //   leftHookMotor.stopMotor();
    //   rightHookMotor.stopMotor();
    //   hookTop = !hookTop;
     
    // }



    // end of climber code


      


    // Arm Code

    if (m_controller.getXButtonPressed()) {
      armMotor.set(-1.0);
      num = 1;

    } 
    else if (m_controller.getXButtonReleased()){
      armMotor.stopMotor();
    }

    if (m_controller.getBButtonPressed()) {
      armMotor.set(1.0);
      num=0;
    }
    else if(m_controller.getBButtonReleased()){
      armMotor.stopMotor();
    }

    if (num == 1){
      if (m_controller.getLeftBumperPressed()){
        armMotor.set(0.7);
      }
    }
    else if (num == 0){
      if(m_controller.getLeftBumperPressed()){
        armMotor.set(-0.7);
      }
    }
    // else  {
    //   armMotor.stopMotor();
    // }

    // if (m_controller.getBButtonPressed()) {
    //   armMotor.set(-1.0);

    // } else if (m_controller.getBButtonReleased()) {
    //   armMotor.stopMotor();
    // }

    // arm code end


    // intake code

    // if (m_controller.getRightTriggerAxis()>0.2 ) {
    //   if (intake){
    //   leftIntakeMotor.set(.75);
    //   rightIntakeMotor.set(.75);  
    //   }
    //  else 
    //  {
    //   leftIntakeMotor.stopMotor();
    //   rightIntakeMotor.stopMotor();
    //  } 
    //  intake = !intake;
    // }

    leftIntakeMotor.set(m_controller.getLeftTriggerAxis()-m_controller.getRightTriggerAxis());
    rightIntakeMotor.set((m_controller.getLeftTriggerAxis()-m_controller.getRightTriggerAxis()));

    

  

    //shooter starts

    // if (m_controller.getStartButtonPressed() ) {
    //   if (reverse){
    //   leftIntakeMotor.set(-.75);
    //   rightIntakeMotor.set(-.75);  
    //   }
    //  else 
    //  {
    //   leftIntakeMotor.stopMotor();
    //   rightIntakeMotor.stopMotor();
      
    //  } 
    //  reverse = !reverse;
    // }
    
    
   
    // Back door code 

    if (m_controller.getRightBumperPressed()) {
      if (backDoor){
        
        rearDoor.set(1);
      }
     else{
      rearDoor.set(0);
    }
    backDoor = !backDoor;
  }

  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  
}

