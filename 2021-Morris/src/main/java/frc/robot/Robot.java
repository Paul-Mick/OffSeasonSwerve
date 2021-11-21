package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Peripherals;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  File f;
  BufferedWriter bw;
  FileWriter fw;
  double startTime;

  private final Peripherals peripherals = new Peripherals();
  private Drive drive = new Drive(peripherals);

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    drive.init();
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    try {
      if(bw != null){ 
        bw.close();
      }
      if(fw != null){
        fw.close();
      }
     
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    startTime = Timer.getFPGATimestamp();
    try{
      f = new File("/home/lvuser/Navx/NavxValues" +  (int) startTime + ".csv");
      if(!f.exists()){
        f.createNewFile();  
      }
      fw = new FileWriter(f);
      bw = new BufferedWriter(fw);
      bw.write("started collecting \n" );
      
      
      }
      catch (Exception e){
        e.printStackTrace();
      }

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
       try{
      bw.write(Timer.getFPGATimestamp() - startTime + ",");
      bw.write( peripherals.getNavxYaw() + ",");
      bw.write( peripherals.getNavxRoll() + ",");
      bw.write( peripherals.getNavxPitch() + "\n");
    } catch (Exception e){
      e.printStackTrace();
    }
    SmartDashboard.putNumber("Controller Y", OI.getDriverLeftY());
    SmartDashboard.putNumber("Controller X", OI.getDriverLeftX());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
