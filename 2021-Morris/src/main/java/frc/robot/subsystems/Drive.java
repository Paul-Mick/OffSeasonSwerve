package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.commands.defaults.DriveDefault;

public class Drive extends SubsystemBase {
    private final OI OI = new OI();

    private final WPI_TalonFX leftForwardMotor = new WPI_TalonFX(6);
    private final WPI_TalonFX leftForwardAngleMotor = new WPI_TalonFX(1);
    private final WPI_TalonFX leftBackMotor = new WPI_TalonFX(4);
    private final WPI_TalonFX leftBackAngleMotor = new WPI_TalonFX(3);
    private final WPI_TalonFX rightForwardMotor = new WPI_TalonFX(2);
    private final WPI_TalonFX rightForwardAngleMotor = new WPI_TalonFX(5);
    private final WPI_TalonFX rightBackMotor = new WPI_TalonFX(8);
    private final WPI_TalonFX rightBackAngleMotor = new WPI_TalonFX(7);

    private Peripherals peripherals;

    private double adjustedX = 0.0;
    private double adjustedY = 0.0;

    private CANCoder backRightAbsoluteEncoder = new CANCoder(2);
    private CANCoder frontLeftAbsoluteEncoder = new CANCoder(1);
    private CANCoder frontRightAbsoluteEncoder = new CANCoder(5);
    private CANCoder backLeftAbsoluteEncoder = new CANCoder(3);

    private final SwerveModule leftFront = new SwerveModule(2, leftForwardAngleMotor, leftForwardMotor, 0, frontLeftAbsoluteEncoder);
    private final SwerveModule leftBack = new SwerveModule(3, leftBackAngleMotor, leftBackMotor, 0, backLeftAbsoluteEncoder);
    private final SwerveModule rightFront = new SwerveModule(1, rightForwardAngleMotor, rightForwardMotor, 0, frontRightAbsoluteEncoder);
    private final SwerveModule rightBack = new SwerveModule(4, rightBackAngleMotor, rightBackMotor, 0, backRightAbsoluteEncoder);

    double initAngle;
    double setAngle;
    double diffAngle;

    public Drive(Peripherals peripherals) {
        this.peripherals = peripherals;
    }

    public double getLeftForwardEncoder() {
        return leftFront.getAbsolutePosition();
    }

    public double getLeftBackEncoder() {
        return leftBack.getAbsolutePosition();
    }

    public double getRightForwardEncoder() {
        return rightFront.getAbsolutePosition();
    }

    public double getRightBackEncoder() {
        return rightBack.getAbsolutePosition();
    }

    public double getAdjustedY(double originalX, double originalY){
        double adjustedY = originalY * Math.sqrt((1-(Math.pow(originalX, 2))/2));
        return adjustedY;
    }

    public double getAdjustedX(double originalX, double originalY){
        double adjustedX = originalX * Math.sqrt((1-(Math.pow(originalY, 2))/2));
        return adjustedX;
    }

    public void init() {
        leftFront.init();
        leftBack.init();
        rightBack.init();
        rightFront.init();
        peripherals.zeroNavx();

        setDefaultCommand(new DriveDefault(this));
    }

    public void swerveDrive() {
        double originalX = OI.getDriverLeftX();
        double originalY = -OI.getDriverLeftY();
        double turn = OI.getDriverRightX();
        double navxOffset = peripherals.getNavxAngle();
        double xPower = getAdjustedX(originalX, originalY);
        double yPower = getAdjustedY(originalX, originalY);

        leftFront.moduleDrive( xPower, yPower, turn, navxOffset);
        rightFront.moduleDrive( xPower, yPower, turn, navxOffset);
        leftBack.moduleDrive(xPower, yPower, turn, navxOffset);
        rightBack.moduleDrive(xPower, yPower, turn, navxOffset);

    }

    private Command DriveDefault() {
        return null;
    }
 

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
}