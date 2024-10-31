/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/



/*		There are encoders on talon's 16, 12, 10
 * 
 * 		 ======|							 |======
 * 		|											|
 * 		|											|
 * 		|											|
 * 		|											|
 * 		|[FrontLeft-12]				 [FrontRight-10]|
 * 		|[RearLeft-13]			  	 [ RearRight-11]|
 * 		|											|
 * 		|[climbMotor-18]							|
 * 		|											|
 * 		|							[leftArmMotor-16|
 * 		|						   [rightArmMotor-17|
 * 		============================================
 * 
 * 
 * IMPORTANT: ROBOT SELF-DESTRUCT.EXE CLASS MUST EXECUTE WHEN Will COMES TO ROBOTICS
 * IMPORTANT: EMERGENCY_STOP.EXE MUST EXECUTE WHEN Will TOUCHES THE ROBOT
 * iMPORTANT: ROBOT_DELETE_CODEMUST EXECUTE WHEN Will THINKS ABOUT ROBOTICS
 * IMPORTANT: ROBOT MUST EXECUTE SELF-DESTRUCT.EXE WHEN JOHN PULLS OUT HIS JUUL
 */




package org.usfirst.frc.team2641.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
//import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix.motorcontrol.can.*;
import com.kauailabs.navx.frc.AHRS;


/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends IterativeRobot {
	WPI_TalonSRX frontLeftMotor; //left drivetrain motors
	WPI_TalonSRX rearLeftMotor;
	
	WPI_TalonSRX frontRightMotor; //right drivetrain motors
	WPI_TalonSRX rearRightMotor;
	
	WPI_TalonSRX leftIntakeMotor; //intake motors on opposite sides
	WPI_TalonSRX rightIntakeMotor;
	
	WPI_TalonSRX leftArmMotor; //motor to shoot up a hook and reel robot up
	WPI_TalonSRX rightArmMotor;
	WPI_TalonSRX climbMotor; //raise/lower the intake arm
	
	Encoder climbingArmEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);//climbing arm's encoder
	
	Joystick gamePad0 = new Joystick(0); //for driving and climbing - its the yellow one
	Joystick gamePad1 = new Joystick(1); //for other stuff - the one without the tape
	double position = 0.0;
	boolean flag1, flag2, flag3, flag4, flag5, flag6;
	int counter;
	
	AHRS ahrs;
	
	int bisectAlgorithmCounter = 0, num = 5;
	double omega1, omega2, omegaTemp = 0;	//teleop arm stuff
	double power1, power2, powerTemp;
	

	@Override
	public void robotInit() {
		frontLeftMotor = new WPI_TalonSRX(12); //use the device IDs as number!
		rearLeftMotor = new WPI_TalonSRX(13);  //joysticks
		
		frontRightMotor = new WPI_TalonSRX(10); //right joystick
		rearRightMotor = new WPI_TalonSRX(11);
		
		
		leftIntakeMotor = new WPI_TalonSRX(14); //lb, lt, rb, rt
		rightIntakeMotor = new WPI_TalonSRX(15);
		
		leftArmMotor = new WPI_TalonSRX(16); //a, y
		rightArmMotor = new WPI_TalonSRX(17);
		
		climbMotor = new WPI_TalonSRX(18); //a, b
		CameraServer.getInstance().startAutomaticCapture(); //should launch the camera
		CameraServer.getInstance().startAutomaticCapture();
		
		frontLeftMotor.setSelectedSensorPosition(0, 0, 0);
		frontRightMotor.setSelectedSensorPosition(0, 0, 0);
		leftArmMotor.setSelectedSensorPosition(0, 0, 0);
		
		ahrs = new AHRS(SerialPort.Port.kMXP, AHRS.SerialDataType.kProcessedData, (byte)4);
	}
	
	@Override
	public void teleopInit() {
		frontLeftMotor.setSelectedSensorPosition(0, 0, 0);
		frontRightMotor.setSelectedSensorPosition(0, 0, 0);
		ahrs.reset();
	}
	
	@Override
	public void autonomousInit() {
		frontLeftMotor.setSelectedSensorPosition(0, 0, 0);
		frontRightMotor.setSelectedSensorPosition(0, 0, 0);
		ahrs.reset();
	}

	@Override
	public void teleopPeriodic() {
		SmartDashboard.putNumber("Yaw", ahrs.getYaw());
		SmartDashboard.putNumber("TotalYaw", ahrs.getAngle());
		
		SmartDashboard.putNumber("Arm sensor position", leftArmMotor.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Arm sensor velocity", leftArmMotor.getSelectedSensorVelocity(0));
		
		SmartDashboard.putNumber("Left motor position", frontLeftMotor.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Left motor velocity", frontLeftMotor.getSelectedSensorVelocity(0));
		
		SmartDashboard.putNumber("Right motor position", frontRightMotor.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Right motor velocity", frontRightMotor.getSelectedSensorVelocity(0));		
		
		double reduction = 1.666; //arbitrary testing value - 666 = 60% power
		double pivotreduction = reduction/0.7853;//higher the value the more right-leaning
		
		double rightStickValue = (gamePad0.getRawAxis(5)/pivotreduction);
		double leftStickValue = gamePad0.getRawAxis(1)/reduction;
		double value = 1/reduction;
		double lowvalue = 1/(reduction * 2);
		double highvalue = 1/(reduction / 2);
		double armMax = 0.60;
		double armValue = (gamePad1.getRawAxis(1)*armMax);
		double cancelOutGravity;//Power necessary to keep lever steady at certain angle
		
		
		/*
		System.out.println("Reduced Left Stick: " + leftStickValue);
		System.out.println("Reduced Right Stick: " + rightStickValue);
		System.out.println("Yellow A button value: " + gamePad0.getRawButton(1));
		System.out.println("Yellow B button value: " + gamePad0.getRawButton(2));
		System.out.println("Regular LB: " + gamePad1.getRawButton(4));
		System.out.println("Regular Left Trigger: " + gamePad1.getRawAxis(2));
		System.out.println("Regular RB: " + gamePad1.getRawButton(5));
		System.out.println("Regular Right Trigger: " + gamePad1.getRawAxis(3));
		*/
		System.out.println("Climbing Arm Encoder Distance: " + climbingArmEncoder.getDistance());
		System.out.println("Climbing Arm Encoder Distance Per Pulse: " + climbingArmEncoder.getDistancePerPulse());
		
		if (leftStickValue > 0.01 || leftStickValue < -0.01) {
			frontLeftMotor.set(-leftStickValue);
			rearLeftMotor.set(-leftStickValue);
		}
		else {
			frontLeftMotor.set(0);
			rearLeftMotor.set(0);
		}

		if (rightStickValue > 0.01 || rightStickValue < -0.01) {
			frontRightMotor.set(rightStickValue);
			rearRightMotor.set(rightStickValue);
		}
		else {
			frontRightMotor.set(0);
			rearRightMotor.set(0);
		}
		
		if(gamePad0.getRawButton(1)) { //hook up - A button
			climbMotor.set(value);
		}
		else if(gamePad0.getRawAxis(3) == 1) { //pull the robot up - RT button //still b
			climbMotor.set(-value);
		}
		else {
			climbMotor.set(0);
		}
		
		if(gamePad1.getRawButton(5)) { //low intake - LB button
			leftIntakeMotor.set(lowvalue);
			rightIntakeMotor.set(-lowvalue); //opposite values for opposite motors
		}
		else if(gamePad1.getRawAxis(2) == 1) { //high intake - Left Trigger
			leftIntakeMotor.set(highvalue);
			rightIntakeMotor.set(-highvalue); //opposite values for opposite motors
		}
		else if(gamePad1.getRawButton(6)) { //low outake - RB button
			leftIntakeMotor.set(-lowvalue); //opposite values for opposite motors
			rightIntakeMotor.set(lowvalue);
		}
		else if(gamePad1.getRawAxis(3) == 1) { //high intake - Right Trigger
			leftIntakeMotor.set(-highvalue); //opposite values for opposite motors
			rightIntakeMotor.set(highvalue);
		}
		else {
			leftIntakeMotor.set(0);
			rightIntakeMotor.set(0);
		}
		
		//if(gamePad1.getRawButton(1)) { //lower the arm at some value - A 
			//leftArmMotor.set(-armvalue);
			//rightArmMotor.set(-armvalue);
			//position = leftArmMotor.getSelectedSensorPosition(0);
			//bisectAlgorithmCounter = 0;
		//}
		if(armValue < -0.01) { //raise the arm by same value - Y
			leftArmMotor.set(-armValue);
			rightArmMotor.set(-armValue);
			position = leftArmMotor.getSelectedSensorPosition(0);
			bisectAlgorithmCounter = 0;
		}
		/*else//the leftArmMotor has the encoder
		{
			if(secantAlgorithmCounter == 0){
				//System.out.println(secantAlgorithmCounter);
				omega1 = leftArmMotor.getSelectedSensorVelocity(0);
				power1 = 0.8;
			}
			else if(secantAlgorithmCounter == 0)
			{
				omega2 = leftArmMotor.getSelectedSensorVelocity(0);
				power2 = 0.6;
			}
		}*/
		/*else
		{
			bisectAlgorithmCounter++;
			
			if(bisectAlgorithmCounter % num == 0)
			{
				omegaTemp = leftArmMotor.getSelectedSensorVelocity(0);
				if(bisectAlgorithmCounter == 0)
				{
					power1 = 0;
					leftArmMotor.set(0);
					rightArmMotor.set(0);
				}
				if(bisectAlgorithmCounter == num)
				{
					omega1 = omegaTemp;
					power2 = 1;
					leftArmMotor.set(1);
					rightArmMotor.set(1);
				}
				if(bisectAlgorithmCounter == (2 * num))
				{
					omega2 = omegaTemp;
				}
				if(bisectAlgorithmCounter > (2 * num))
				{
					if(bisectAlgorithmCounter % (2 * num) == num)
					{
						powerTemp = (power1 + power2)/2;
						leftArmMotor.set(powerTemp);
						rightArmMotor.set(powerTemp);
					}
				}
				if(bisectAlgorithmCounter % (2 * num) == 0)
				{
					
				}
			}
		}
			/*if(position > some value)//some value will always be less than cancelOutGravity to ensure that once the lever rests at that value it will continue to stay their throughout the teleop loop
			{
				cancelOutGravity = A*Math.cos(theta + b);//A is a fit parameter (AKA the desired angle), theta is 90-the angle between Fg and the lever (see board), b is the angle when the lever is at rest (resting on bumper)
				leftArmMotor.set(cancelOutGravity);
				rightArmMotor.set(cancelOutGravity);
			}*/
	}//////////////////////////////////////////////////////////////////////////////////
		/*if(position > some value)//some value will always be less than cancelOutGravity to ensure that once the lever rests at that value it will continue to stay their throughout the teleop loop
		{
			cancelOutGravity = A*Math.cos(theta + b);//A is a fit parameter (AKA the desired angle), theta is 90-the angle between Fg and the lever (see board), b is the angle when the lever is at rest (resting on bumper)
			leftArmMotor.set(cancelOutGravity);
			rightArmMotor.set(cancelOutGravity);
		}*//////////////////////////////////////////////////////////////////////////////////
		/*else if(leftArmMotor.getSelectedSensorPosition(0) > position) {
			leftArmMotor.set(0.5);
			rightArmMotor.set(0.5);
			System.out.println("here");
			System.out.println(leftArmMotor.getSelectedSensorPosition(0));
		}*/
		//else if(rightArmMotor.getSelectedSensorPosition(0) > position) {
			//leftArmMotor.set(0);
			//rightArmMotor.set(0);
		//}
		/*else {
			leftArmMotor.set(0);
			rightArmMotor.set(0);
		}*/
	
	
	public void autonomousPeriodic() {
		/* pulse per inch ---> 18.72/1400 = .013
		 * 
		 * 126 in/.013 = 9692 to get into shooting range of the switch if going only forward
		 * 
		 * from middle to blue box: 120-20 = 100/.013 = 7692
		 */
		SmartDashboard.putNumber("Left motor position", frontLeftMotor.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Left motor velocity", frontLeftMotor.getSelectedSensorVelocity(0));
		
		SmartDashboard.putNumber("Right motor position", frontRightMotor.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Right motor velocity", frontRightMotor.getSelectedSensorVelocity(0));
		
		double reduction = 1.666;			//speed of the robot
		double pivotreduction = reduction/0.7853;	//this allows one side to move at  a slower speed than the other
		DriverStation.Alliance color = DriverStation.getInstance().getAlliance();
		
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		if(color == DriverStation.Alliance.Blue){ //the superior nested loop works
			if(DriverStation.getInstance().getLocation() == 1) { //blue 1 is right side
				if(gameData.length() > 0)
				{
					if(gameData.charAt(0) == 'L')
					{		
						if(frontLeftMotor.getSelectedSensorPosition(0) > -12693 && flag1)
						{
							frontLeftMotor.set(reduction);
							rearLeftMotor.set(reduction);
							
							frontRightMotor.set(pivotreduction);
							rearRightMotor.set(pivotreduction);
						}
						if(frontLeftMotor.getSelectedSensorPosition(0) <= -12693 && flag1)
						{
							flag1 = false;
							ahrs.reset();
						}
					}
					else
					{
						if(frontLeftMotor.getSelectedSensorPosition(0) > -12693 && flag1)
						{
							frontLeftMotor.set(reduction);
							rearLeftMotor.set(reduction);
							
							frontRightMotor.set(pivotreduction);
							rearRightMotor.set(pivotreduction);
						}
						if(frontLeftMotor.getSelectedSensorPosition(0) <= -12693 && flag1)
						{
							flag1 = false;
							ahrs.reset();
						}
						  if(ahrs.getYaw() < 90.1 && flag2 && !flag1)
						  {
								//This will point turn the robot to the left
								//Left side moves
								frontLeftMotor.set(-reduction);
								rearLeftMotor.set(-reduction);
								//right side less speed
								frontRightMotor.set(pivotreduction);
								rearRightMotor.set(pivotreduction);
						  }
						  if(ahrs.getYaw() >= 90.1 && flag2 && !flag1)
						  {
							  flag2 = false;
							  ahrs.reset();
						  }
						  
						  if(frontLeftMotor.getSelectedSensorPosition(0) > -4000 && flag3 && !flag2)
						  {
								frontLeftMotor.set(reduction);
								rearLeftMotor.set(reduction);
								
								frontRightMotor.set(pivotreduction);
								rearRightMotor.set(pivotreduction);
						  }
						  if(frontLeftMotor.getSelectedSensorPosition(0) <=-4000 && flag3 && !flag2)
						  {
							  flag3 = false;
							  ahrs.reset();
						  }
						  
						  if(counter < 150 && flag4 && !flag3)
							  climbMotor.set(0.2);
						  if(counter >= 150 && flag4 && !flag3)
						  {
							  flag4 = false;
							  counter = 0;
						  }
						  
						  if(counter < 150 && flag5 && !flag4)
						  {
								leftIntakeMotor.set(-1/(1.666*2)); //opposite values for opposite motors
								rightIntakeMotor.set(1/(1.666*2));
						  }
					}
				}
			}
			else if(DriverStation.getInstance().getLocation() == 2) { //blue 2 is middle
				 if(gameData.length() > 0)
	                {
						  if(gameData.charAt(0) == 'L')
						  {
							  if(frontLeftMotor.getSelectedSensorPosition(0) > -6538 && flag1)//pulse variable < -7692)//num of pulses to get just in front of blue box
							  {
								  frontLeftMotor.set(reduction);
								  rearLeftMotor.set(reduction);
									
								  frontRightMotor.set(pivotreduction);
								  rearRightMotor.set(pivotreduction);
							  }
							  if(frontLeftMotor.getSelectedSensorPosition(0) <= -6538 && flag1)
							  {
								  flag1 = false;
								  ahrs.reset();
							  }
							  
							  if(ahrs.getYaw() < 90.1 && flag2 && !flag1) {	
									//This will point turn the robot to the left
									//Left side moves
									frontLeftMotor.set(-reduction);
									rearLeftMotor.set(-reduction);
									//right side less speed
									frontRightMotor.set(pivotreduction);
									rearRightMotor.set(pivotreduction);
							  }
							  if(ahrs.getYaw() >= 90.1 && flag2 && !flag1)
							  {
								  flag2 = false;
								  ahrs.reset();
							  }
							  
							  if(frontLeftMotor.getSelectedSensorPosition(0) > -11078 && flag3 && !flag2)
							  {
								  frontLeftMotor.set(reduction);
								  rearLeftMotor.set(reduction);
									
								  frontRightMotor.set(pivotreduction);
								  rearRightMotor.set(pivotreduction);
							  }
							  if(frontLeftMotor.getSelectedSensorPosition(0) <= -11078 && flag3 && !flag2)
							  {
								  flag3 = false;
								  ahrs.reset();
							  }
							  
							  if(ahrs.getYaw() > -90.1 && flag4 && !flag3)
							  {
									//This will point turn the robot to the right
									//Left side moves
									frontLeftMotor.set(reduction);
									rearLeftMotor.set(reduction);
									//right side less speed
									frontRightMotor.set(-pivotreduction);
									rearRightMotor.set(-pivotreduction);
							  }
							  if(ahrs.getYaw() <= -90.1 && flag4 && !flag3)
							  {
								  flag4 = false;
								  ahrs.reset();
							  }
							  
							  if(frontLeftMotor.getSelectedSensorPosition(0) > -2769 && flag5 && !flag4)
							  {
								  frontLeftMotor.set(reduction);
								  rearLeftMotor.set(reduction);
									
								  frontRightMotor.set(pivotreduction);
								  rearRightMotor.set(pivotreduction);
							  }
							  if(frontLeftMotor.getSelectedSensorPosition(0) <= -2769 && flag5 && !flag4)
							  {
								  flag5 = false;
								  ahrs.reset();
							  }
						  }
							  
	                } else {
						  if(frontLeftMotor.getSelectedSensorPosition(0) > -6538 && flag1)//pulse variable < -7692)//num of pulses to get just in front of blue box
						  {
							  frontLeftMotor.set(reduction);
							  rearLeftMotor.set(reduction);
								
							  frontRightMotor.set(pivotreduction);
							  rearRightMotor.set(pivotreduction);
						  }
						  if(frontLeftMotor.getSelectedSensorPosition(0) <= -6538 && flag1)
						  {
							  flag1 = false;
							  ahrs.reset();
						  }
						  
						  if(ahrs.getYaw() > -90.1 && flag2 && !flag1) {	
								//This will point turn the robot to the right
								//Left side moves
								frontLeftMotor.set(reduction);
								rearLeftMotor.set(reduction);
								//right side less speed
								frontRightMotor.set(-pivotreduction);
								rearRightMotor.set(-pivotreduction);
						  }
						  if(ahrs.getYaw() <= -90.1 && flag2 && !flag1)
						  {
							  flag2 = false;
							  ahrs.reset();
						  }
						  
						  if(frontLeftMotor.getSelectedSensorPosition(0) > -11078 && flag3 && !flag2)
						  {
							  frontLeftMotor.set(reduction);
							  rearLeftMotor.set(reduction);
								
							  frontRightMotor.set(pivotreduction);
							  rearRightMotor.set(pivotreduction);
						  }
						  if(frontLeftMotor.getSelectedSensorPosition(0) <= -11078 && flag3 && !flag2)
						  {
							  flag3 = false;
							  ahrs.reset();
						  }
						  
						  if(ahrs.getYaw() < 90.1 && flag4 && !flag3)
						  {
								//This will point turn the robot to the left
								//Left side moves
								frontLeftMotor.set(-reduction);
								rearLeftMotor.set(-reduction);
								//right side less speed
								frontRightMotor.set(pivotreduction);
								rearRightMotor.set(pivotreduction);
						  }
						  if(ahrs.getYaw() >= 90.1 && flag4 && !flag3)
						  {
							  flag4 = false;
							  ahrs.reset();
						  }
						  
						  if(frontLeftMotor.getSelectedSensorPosition(0) > -2769 && flag5 && !flag4)
						  {
							  frontLeftMotor.set(reduction);
							  rearLeftMotor.set(reduction);
								
							  frontRightMotor.set(pivotreduction);
							  rearRightMotor.set(pivotreduction);
						  }
						  if(frontLeftMotor.getSelectedSensorPosition(0) <= -2769 && flag5 && !flag4)
						  {
							  flag5 = false;
							  ahrs.reset();
						  }
					}
		
			}
			else if(DriverStation.getInstance().getLocation() == 3) { //blue 3 is left side
				if(gameData.charAt(0) == 'L')
				{		
					if(frontLeftMotor.getSelectedSensorPosition(0) > -12693 && flag1)
					{
						frontLeftMotor.set(reduction);
						rearLeftMotor.set(reduction);
						
						frontRightMotor.set(pivotreduction);
						rearRightMotor.set(pivotreduction);
					}
					if(frontLeftMotor.getSelectedSensorPosition(0) <= -12693 && flag1)
					{
						flag1 = false;
						ahrs.reset();
					}
				}
				else
				{
					if(frontLeftMotor.getSelectedSensorPosition(0) > -12693 && flag1)
					{
						frontLeftMotor.set(reduction);
						rearLeftMotor.set(reduction);
						
						frontRightMotor.set(pivotreduction);
						rearRightMotor.set(pivotreduction);
					}
					if(frontLeftMotor.getSelectedSensorPosition(0) <= -12693 && flag1)
					{
						flag1 = false;
						ahrs.reset();
					}
					  if(ahrs.getYaw() > -90.1 && flag2 && !flag1)
					  {
							//This will point turn the robot to the left
							//Left side moves
							frontLeftMotor.set(-reduction);
							rearLeftMotor.set(-reduction);
							//right side less speed
							frontRightMotor.set(pivotreduction);
							rearRightMotor.set(pivotreduction);
					  }
					  if(ahrs.getYaw() <= -90.1 && flag2 && !flag1)
					  {
						  flag2 = false;
						  ahrs.reset();
					  }
					  
					  if(frontLeftMotor.getSelectedSensorPosition(0) > -4000 && flag3 && !flag2)
					  {
							frontLeftMotor.set(reduction);
							rearLeftMotor.set(reduction);
							
							frontRightMotor.set(pivotreduction);
							rearRightMotor.set(pivotreduction);
					  }
					  if(frontLeftMotor.getSelectedSensorPosition(0) <=-4000 && flag3 && !flag2)
					  {
						  flag3 = false;
						  ahrs.reset();
					  }
					  
					  if(counter < 150 && flag4 && !flag3)
						  climbMotor.set(0.2);
					  if(counter >= 150 && flag4 && !flag3)
					  {
						  flag4 = false;
						  counter = 0;
					  }
					  
					  if(counter < 150 && flag5 && !flag4)
					  {
							leftIntakeMotor.set(-1/(1.666*2)); //opposite values for opposite motors
							rightIntakeMotor.set(1/(1.666*2));
					  }
				}
			}
		}
		else if(color == DriverStation.Alliance.Red) {
			if(DriverStation.getInstance().getLocation() == 1) { //red 1 is right side
				if(gameData.length() > 0)
				{
					if(gameData.charAt(0) == 'L')
					{		
						if(frontLeftMotor.getSelectedSensorPosition(0) > -12693 && flag1)
						{
							frontLeftMotor.set(reduction);
							rearLeftMotor.set(reduction);
							
							frontRightMotor.set(pivotreduction);
							rearRightMotor.set(pivotreduction);
						}
						if(frontLeftMotor.getSelectedSensorPosition(0) <= -12693 && flag1)
						{
							flag1 = false;
							ahrs.reset();
						}
					}
					else
					{
						if(frontLeftMotor.getSelectedSensorPosition(0) > -12693 && flag1)
						{
							frontLeftMotor.set(reduction);
							rearLeftMotor.set(reduction);
							
							frontRightMotor.set(pivotreduction);
							rearRightMotor.set(pivotreduction);
						}
						if(frontLeftMotor.getSelectedSensorPosition(0) <= -12693 && flag1)
						{
							flag1 = false;
							ahrs.reset();
						}
						  if(ahrs.getYaw() < 90.1 && flag2 && !flag1)
						  {
								//This will point turn the robot to the left
								//Left side moves
								frontLeftMotor.set(-reduction);
								rearLeftMotor.set(-reduction);
								//right side less speed
								frontRightMotor.set(pivotreduction);
								rearRightMotor.set(pivotreduction);
						  }
						  if(ahrs.getYaw() >= 90.1 && flag2 && !flag1)
						  {
							  flag2 = false;
							  ahrs.reset();
						  }
						  
						  if(frontLeftMotor.getSelectedSensorPosition(0) > -4000 && flag3 && !flag2)
						  {
								frontLeftMotor.set(reduction);
								rearLeftMotor.set(reduction);
								
								frontRightMotor.set(pivotreduction);
								rearRightMotor.set(pivotreduction);
						  }
						  if(frontLeftMotor.getSelectedSensorPosition(0) <=-4000 && flag3 && !flag2)
						  {
							  flag3 = false;
							  ahrs.reset();
						  }
						  
						  if(counter < 150 && flag4 && !flag3)
							  climbMotor.set(0.2);
						  if(counter >= 150 && flag4 && !flag3)
						  {
							  flag4 = false;
							  counter = 0;
						  }
						  
						  if(counter < 150 && flag5 && !flag4)
						  {
								leftIntakeMotor.set(-1/(1.666*2)); //opposite values for opposite motors
								rightIntakeMotor.set(1/(1.666*2));
						  }
					}
				}
			}
			else if(DriverStation.getInstance().getLocation() == 2) { //red 2 is middle
				 if(gameData.length() > 0)
	                {
						  if(gameData.charAt(0) == 'L')
						  {
							  if(frontLeftMotor.getSelectedSensorPosition(0) > -6538 && flag1)//pulse variable < -7692)//num of pulses to get just in front of blue box
							  {
								  frontLeftMotor.set(reduction);
								  rearLeftMotor.set(reduction);
									
								  frontRightMotor.set(pivotreduction);
								  rearRightMotor.set(pivotreduction);
							  }
							  if(frontLeftMotor.getSelectedSensorPosition(0) <= -6538 && flag1)
							  {
								  flag1 = false;
								  ahrs.reset();
							  }
							  
							  if(ahrs.getYaw() < 90.1 && flag2 && !flag1) {	
									//This will point turn the robot to the left
									//Left side moves
									frontLeftMotor.set(-reduction);
									rearLeftMotor.set(-reduction);
									//right side less speed
									frontRightMotor.set(pivotreduction);
									rearRightMotor.set(pivotreduction);
							  }
							  if(ahrs.getYaw() >= 90.1 && flag2 && !flag1)
							  {
								  flag2 = false;
								  ahrs.reset();
							  }
							  
							  if(frontLeftMotor.getSelectedSensorPosition(0) > -11078 && flag3 && !flag2)
							  {
								  frontLeftMotor.set(reduction);
								  rearLeftMotor.set(reduction);
									
								  frontRightMotor.set(pivotreduction);
								  rearRightMotor.set(pivotreduction);
							  }
							  if(frontLeftMotor.getSelectedSensorPosition(0) <= -11078 && flag3 && !flag2)
							  {
								  flag3 = false;
								  ahrs.reset();
							  }
							  
							  if(ahrs.getYaw() > -90.1 && flag4 && !flag3)
							  {
									//This will point turn the robot to the right
									//Left side moves
									frontLeftMotor.set(reduction);
									rearLeftMotor.set(reduction);
									//right side less speed
									frontRightMotor.set(-pivotreduction);
									rearRightMotor.set(-pivotreduction);
							  }
							  if(ahrs.getYaw() <= -90.1 && flag4 && !flag3)
							  {
								  flag4 = false;
								  ahrs.reset();
							  }
							  
							  if(frontLeftMotor.getSelectedSensorPosition(0) > -2769 && flag5 && !flag4)
							  {
								  frontLeftMotor.set(reduction);
								  rearLeftMotor.set(reduction);
									
								  frontRightMotor.set(pivotreduction);
								  rearRightMotor.set(pivotreduction);
							  }
							  if(frontLeftMotor.getSelectedSensorPosition(0) <= -2769 && flag5 && !flag4)
							  {
								  flag5 = false;
								  ahrs.reset();
							  }
						  }
							  
	                } else {
						  if(frontLeftMotor.getSelectedSensorPosition(0) > -6538 && flag1)//pulse variable < -7692)//num of pulses to get just in front of blue box
						  {
							  frontLeftMotor.set(reduction);
							  rearLeftMotor.set(reduction);
								
							  frontRightMotor.set(pivotreduction);
							  rearRightMotor.set(pivotreduction);
						  }
						  if(frontLeftMotor.getSelectedSensorPosition(0) <= -6538 && flag1)
						  {
							  flag1 = false;
							  ahrs.reset();
						  }
						  
						  if(ahrs.getYaw() > -90.1 && flag2 && !flag1) {	
								//This will point turn the robot to the right
								//Left side moves
								frontLeftMotor.set(reduction);
								rearLeftMotor.set(reduction);
								//right side less speed
								frontRightMotor.set(-pivotreduction);
								rearRightMotor.set(-pivotreduction);
						  }
						  if(ahrs.getYaw() <= -90.1 && flag2 && !flag1)
						  {
							  flag2 = false;
							  ahrs.reset();
						  }
						  
						  if(frontLeftMotor.getSelectedSensorPosition(0) > -11078 && flag3 && !flag2)
						  {
							  frontLeftMotor.set(reduction);
							  rearLeftMotor.set(reduction);
								
							  frontRightMotor.set(pivotreduction);
							  rearRightMotor.set(pivotreduction);
						  }
						  if(frontLeftMotor.getSelectedSensorPosition(0) <= -11078 && flag3 && !flag2)
						  {
							  flag3 = false;
							  ahrs.reset();
						  }
						  
						  if(ahrs.getYaw() < 90.1 && flag4 && !flag3)
						  {
								//This will point turn the robot to the left
								//Left side moves
								frontLeftMotor.set(-reduction);
								rearLeftMotor.set(-reduction);
								//right side less speed
								frontRightMotor.set(pivotreduction);
								rearRightMotor.set(pivotreduction);
						  }
						  if(ahrs.getYaw() >= 90.1 && flag4 && !flag3)
						  {
							  flag4 = false;
							  ahrs.reset();
						  }
						  
						  if(frontLeftMotor.getSelectedSensorPosition(0) > -2769 && flag5 && !flag4)
						  {
							  frontLeftMotor.set(reduction);
							  rearLeftMotor.set(reduction);
								
							  frontRightMotor.set(pivotreduction);
							  rearRightMotor.set(pivotreduction);
						  }
						  if(frontLeftMotor.getSelectedSensorPosition(0) <= -2769 && flag5 && !flag4)
						  {
							  flag5 = false;
							  ahrs.reset();
						  }
					}
		
			}
			else if(DriverStation.getInstance().getLocation() == 3) { //red 3 is left side
				if(gameData.charAt(0) == 'L')
				{		
					if(frontLeftMotor.getSelectedSensorPosition(0) > -12693 && flag1)
					{
						frontLeftMotor.set(reduction);
						rearLeftMotor.set(reduction);
						
						frontRightMotor.set(pivotreduction);
						rearRightMotor.set(pivotreduction);
					}
					if(frontLeftMotor.getSelectedSensorPosition(0) <= -12693 && flag1)
					{
						flag1 = false;
						ahrs.reset();
					}
				}
				else
				{
					if(frontLeftMotor.getSelectedSensorPosition(0) > -12693 && flag1)
					{
						frontLeftMotor.set(reduction);
						rearLeftMotor.set(reduction);
						
						frontRightMotor.set(pivotreduction);
						rearRightMotor.set(pivotreduction);
					}
					if(frontLeftMotor.getSelectedSensorPosition(0) <= -12693 && flag1)
					{
						flag1 = false;
						ahrs.reset();
					}
					  if(ahrs.getYaw() > -90.1 && flag2 && !flag1)
					  {
							//This will point turn the robot to the left
							//Left side moves
							frontLeftMotor.set(-reduction);
							rearLeftMotor.set(-reduction);
							//right side less speed
							frontRightMotor.set(pivotreduction);
							rearRightMotor.set(pivotreduction);
					  }
					  if(ahrs.getYaw() <= -90.1 && flag2 && !flag1)
					  {
						  flag2 = false;
						  ahrs.reset();
					  }
					  
					  if(frontLeftMotor.getSelectedSensorPosition(0) > -4000 && flag3 && !flag2)
					  {
							frontLeftMotor.set(reduction);
							rearLeftMotor.set(reduction);
							
							frontRightMotor.set(pivotreduction);
							rearRightMotor.set(pivotreduction);
					  }
					  if(frontLeftMotor.getSelectedSensorPosition(0) <=-4000 && flag3 && !flag2)
					  {
						  flag3 = false;
						  ahrs.reset();
					  }
					  
					  if(counter < 150 && flag4 && !flag3)
						  climbMotor.set(0.2);
					  if(counter >= 150 && flag4 && !flag3)
					  {
						  flag4 = false;
						  counter = 0;
					  }
					  
					  if(counter < 150 && flag5 && !flag4)
					  {
							leftIntakeMotor.set(-1/(1.666*2)); //opposite values for opposite motors
							rightIntakeMotor.set(1/(1.666*2));
					  }
				}
		}
		//*arm.stay.up.exe execute NOW
		//
		
		
		if(frontLeftMotor.getSelectedSensorPosition(0) > -10000) {
			frontLeftMotor.set(reduction);
			rearLeftMotor.set(reduction);
		}
		if(frontRightMotor.getSelectedSensorPosition(0) < 10000) {
			frontRightMotor.set(-pivotreduction);
			rearRightMotor.set(-pivotreduction);
		}
		
		double angle = 90.0;
		
		System.out.println("Motor Power: " + reduction);
		System.out.println("Sensor Position: " + frontLeftMotor.getSelectedSensorPosition(0));
			
		//if the angle is greater or less than zero, then the robot will pivot
		if(ahrs.getYaw() > 0.1) {
			//This will pivot the robot to the left
			//right side moves
			frontRightMotor.set(-reduction);
			rearRightMotor.set(-reduction);
			//left side less speed
			frontLeftMotor.set(pivotreduction);
			rearLeftMotor.set(pivotreduction);
		}
		else if (ahrs.getYaw() < -0.1) {	
			//This will pivot the robot to the right
			//Left side moves
			frontLeftMotor.set(reduction);
			rearLeftMotor.set(reduction);
			//right side less speed
			frontRightMotor.set(-pivotreduction);
			rearRightMotor.set(-pivotreduction);
		}	
	
		/*import java.first.264q
		  Drive forward when the joystick is go
		  go score when it is tele-op
		  /*
		   //import WPIlib
		    * robo go
		    */
		/*	Encoder rightDriveEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
			Encoder leftDriveEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
			rightDriveEncoder.setMaxPeriod(0.1);
			leftDriveEncoder.setMaxPeriod(0.1);
			rightDriveEncoder.setDistancePerPulse(6.0);//this is distance per rotation, set to 6 in. (diameter of wheel, 1 rotation)
			leftDriveEncoder.setDistancePerPulse(6.0);
			rightDriveEncoder.setMinRate(2);//minimum rate of the device before the hardware reports it stopped
			leftDriveEncoder.setMinRate(2);
			
			double maxSpeed = 6;
			
			
			rightDriveEncoder.reset();//reset distance to 0
			leftDriveEncoder.reset();//same as above
			//diameter of wheels is 6 in.
			//1 ft. is 2 rotations (encoders are on the outer gear shaft)
			while(rightDriveEncoder.getDistance() < 2)
			{
				System.out.println("");
				
				frontLeftMotor.set(leftDriveEncoder.getDistancePerPulse()/maxSpeed);
				rearLeftMotor.set(leftDriveEncoder.getDistancePerPulse()/maxSpeed);
				
				frontRightMotor.set(rightDriveEncoder.getDistancePerPulse()/maxSpeed);
				rearRightMotor.set(rightDriveEncoder.getDistancePerPulse()/maxSpeed);
			}
			frontLeftMotor.set(0); 
			rearLeftMotor.set(0);
			frontRightMotor.set(0);
			rearRightMotor.set(0);
			*/
	}


	
	public void testPeriodic() {
	     LiveWindow.setEnabled(true);
	}
}
