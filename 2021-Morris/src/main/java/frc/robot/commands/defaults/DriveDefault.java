package frc.robot.commands.defaults;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.OI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Peripherals;

public class DriveDefault extends CommandBase {
  /** Creates a new DriveDefault. */
  private static Drive drive;
  // private final Peripherals peripherals = new Peripherals();
  

  public DriveDefault(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    addRequirements(drive);
    // navx.softResetAngle();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // navx.softResetAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // drive.getAngleMotorAngle(0);
    // drive.postAbsoluteEncoder();
    // SmartDashboard.putNumber("Controller Angle", drive.getJoystickAngle(OI.getDriverLeftY(), OI.getDriverLeftX()));
    // SmartDashboard.putNumber("Left Front Abs Encoder", drive.getLeftForwardEncoder());
    drive.swerveDrive();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}