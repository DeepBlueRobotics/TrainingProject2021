// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.lib.MotorControllerFactory;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Motors extends SubsystemBase {

  private final CANSparkMax motor = MotorControllerFactory.createSparkMax(Constants.Motor.kSparkMax);
  private final CANEncoder enc = motor.getEncoder();
  private final CANPIDController pid = motor.getPIDController();

  private final double gearRatio = 2;
  private final double diameter = 3;
  private final double conversion = (Math.PI * diameter) / gearRatio;
  private final double lengthOfArm = 20; // in inches

  

  /** Creates a new ExampleSubsystem. */
  public Motors() {
    enc.setPosition(0);
    // encoder measures input shaft
    enc.setPositionConversionFactor(conversion);
  }

  public void run(double height)
  {
    
    //pid.setReference(string pulled, ControlType.kPosition);
  }

  public CANEncoder getEnc()
  {
    return enc;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
