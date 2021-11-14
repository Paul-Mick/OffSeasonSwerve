// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Optimizer extends SubsystemBase {
  private Drive drive;
  private double initAngle = drive.getLeftForwardEncoder();
  private double diffAngle;
  /** Creates a new Optimizer. */
  public Optimizer() {}

  public void changeAngle(double wantAngle){
    diffAngle = wantAngle - initAngle;

    double setAngle = wantAngle;

    if(diffAngle < 0){
      diffAngle += 360;
    }

    if(diffAngle <= 180){
      setAngle = wantAngle;
    } 
    else{
      setAngle = 360 - wantAngle;
    }

    if(((wantAngle + 180) - initAngle) < setAngle) { 
      setAngle = wantAngle + 180;
    }



    
  }







  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
