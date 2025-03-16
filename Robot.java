// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private final DifferentialDrive m_robotDrive;
  private final Joystick m_leftStick;
  private final Joystick m_rightStick;


  SparkMax m_rightMotor = new SparkMax(4, MotorType.kBrushed);
  SparkMax m_leftothermotor = new SparkMax(10, MotorType.kBrushed);
  SparkMax m_leftMotor = new SparkMax(5, MotorType.kBrushed);
  SparkMax m_rightotherMotor = new SparkMax(6, MotorType.kBrushed);
  SparkMax m_intake = new SparkMax(8, MotorType.kBrushed);
  // SparkMax m_arm = new SparkMax(17, MotorType.kBrushed);
  PS4Controller PS41 = new PS4Controller(0);
//  SparkMax m_arm2 = new SparkMax(18, MotorType.kBrushless);
  
  

  
  /** Called once at the beginning of the robot program. */
  public Robot(){
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    m_robotDrive = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);
    m_leftStick = new Joystick(9);
    m_rightStick = new Joystick(1); 

   // m_robotDrive.arcadeDrive(0.5, 0);

    SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor);
  }



  @Override
public void autonomousPeriodic() {
  //m_robxotDrive.arcadeDrive(1, 0);
  m_leftMotor.set(-0.5);
  m_rightMotor.set(0.5);
  m_leftothermotor.set(-0.5);
  m_rightotherMotor.set(-0.5);
  // m_rightothermotor.set(0.5);
//   double x=0;
//    x=x+1;
//   if (x==5) {
//   m_leftMotor.set(0.5);
//   m_rightMotor.set(0.5);
//  // m_rightothermotor.set(0.5);

    
//   }
}
 

  

  @Override
  public void teleopPeriodic() {
   // m_robotDrive.tankDrive(m_leftMotor, m_rightMotor);
    // m_leftMotor.set(-PS41.getLeftY() + PS41.getLeftX());
    // m_rightmotor.set(PS41.getLeftY() + PS41.getLeftX());
    // m_leftothermotor.set(PS41.getLeftY() + PS41.getLeftX());
    // m_rightothermotor.set(PS41.getLeftY() + PS41.getLeftX());
  double forward = PS41.getLeftY();
  double turn = PS41.getRightX();
  //  m_leftMotor.set(PS41.getLeftY());
  //  m_rightMotor.set(-PS41.getRightY());
  //  m_leftothermotor.set(PS41.getLeftY());
  //  m_rightotherMotor.set(PS41.getRightY());
  // m_leftothermotor.set(0.5);

  m_leftMotor.set((forward - turn));
  m_rightMotor.set(-forward-turn);
  m_leftothermotor.set(forward-turn);
  m_rightotherMotor.set(forward+turn);
  
    //-PS41.getLeftX()
   // m_leftothermotor.set(-PS41.getLeftY());
    //m_rightothermotor.set(PS41.getLeftY());
   // m_arm.set(PS41.getL2Axis() + 1);


    // if (PS41.getSquareButtonReleased()) {
    //   m_intake.set(0.9);
      
    // } else if (PS41.getR2ButtonReleased()) {
    //   m_intake.set(-0.9);
      
    // }
    // m_intake.set(0);
    // m_intake.set(0.5);
  // m_intake.set(PS41.getL2Axis()+1);
  double intake = -PS41.getR2Axis() - 1;
  m_intake.set(((-PS41.getR2Axis()-1)*0.25));
  



    }
}