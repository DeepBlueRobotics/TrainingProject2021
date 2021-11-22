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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Motors extends SubsystemBase {

  private final CANSparkMax motor = MotorControllerFactory.createSparkMax(Constants.Motor.kSparkMax);
  private final CANEncoder enc = motor.getEncoder();
  private final CANPIDController pid = motor.getPIDController();

  private final double gearRatio = 2;
  //private final double diameter = 3;
  private final double conversion = (2*Math.PI) / gearRatio;
  private final double lengthOfArm = 20; // in inches
  double ff;
  double p;
  double i;
  double d;

  /** Creates a new ExampleSubsystem. */
  public Motors() {
    motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    enc.setPosition(0);
    pid.setOutputRange(-5.9/12, 5.9/12);
    // encoder measures input shaft
    p = pid.getP();
    i = pid.getI();
    d = pid.getD();
    ff = pid.getFF();
    SmartDashboard.putNumber("P", p);
    SmartDashboard.putNumber("I", i);
    SmartDashboard.putNumber("D", d);
    SmartDashboard.putNumber("FF", ff);

    enc.setPositionConversionFactor(conversion);
  }

  public void run(double height)
  {
    
    pid.setReference(Math.asin(height/lengthOfArm), ControlType.kPosition);
  }

  public CANEncoder getEnc()
  {
    return enc;
  }

  // Input is in degrees
  public void setInitialPosition(double position) {
    position = Math.toRadians(position);
    enc.setPosition(position);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double np = SmartDashboard.getNumber("P", 0);
    double ni = SmartDashboard.getNumber("I", 0);
    double nd = SmartDashboard.getNumber("D", 0);
    double nFF = SmartDashboard.getNumber("FF", 0);
    if(np != p)
    {
      p = np;
      pid.setP(p);
    }
   if(ni != i) {
    i = ni;
    pid.setI(i);
   }
   if(nd != d) {
     d = nd;

    pid.setD(d);
   }
   if(nFF != ff) {
     ff = nFF;
     pid.setFF(ff);
   }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
