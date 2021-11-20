package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.defaults.DriveDefault;

public class Drive extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    private final WPI_TalonFX leftForwardMotor = new WPI_TalonFX(6);
    private final WPI_TalonFX leftForwardAngleMotor = new WPI_TalonFX(1);
    private final WPI_TalonFX leftBackMotor = new WPI_TalonFX(4);
    private final WPI_TalonFX leftBackAngleMotor = new WPI_TalonFX(3);
    private final WPI_TalonFX rightForwardMotor = new WPI_TalonFX(2);
    private final WPI_TalonFX rightForwardAngleMotor = new WPI_TalonFX(5);
    private final WPI_TalonFX rightBackMotor = new WPI_TalonFX(8);
    private final WPI_TalonFX rightBackAngleMotor = new WPI_TalonFX(7);

    private final Peripherals peripherals = new Peripherals();

    private double adjustedX = 0.0;
    private double adjustedY = 0.0;

    private CANCoder backRightAbsoluteEncoder = new CANCoder(2);
    private CANCoder frontLeftAbsoluteEncoder = new CANCoder(1);
    private CANCoder frontRightAbsoluteEncoder = new CANCoder(5);
    private CANCoder backLeftAbsoluteEncoder = new CANCoder(3);

    private final SwerveModule leftFront = new SwerveModule(1, leftForwardAngleMotor, leftForwardMotor, 0, frontLeftAbsoluteEncoder);
    private final SwerveModule leftBack = new SwerveModule(2, leftBackAngleMotor, leftBackMotor, 0, backLeftAbsoluteEncoder);
    private final SwerveModule rightFront = new SwerveModule(1, rightForwardAngleMotor, rightForwardMotor, 0, frontRightAbsoluteEncoder);
    private final SwerveModule rightBack = new SwerveModule(2, rightBackAngleMotor, rightBackMotor, 0, backRightAbsoluteEncoder);

    private double kF = 0;
    private double kP = 0.01;
    private double kI = 0;
    private double kD = 0;

    double initAngle;
    double setAngle;
    double diffAngle;


    public Drive() {

    }

    public void setAnglePid(double targetAngle, double navxOffset, double percent) {
        // targetAngle = targetAngle + 90 + navxOffset;
        SmartDashboard.putNumber("Target", targetAngle);
        leftFront.setAnglePID(targetAngle);
        leftBack.setAnglePID(targetAngle);
        rightFront.setAnglePID(targetAngle);
        rightBack.setAnglePID(targetAngle);
        setForwardBackMotors(percent);
    }

    public void setDriveMotorPercents(double percent){
        leftForwardMotor.set(ControlMode.PercentOutput, percent);
        leftForwardAngleMotor.set(ControlMode.PercentOutput, percent);
        leftBackMotor.set(ControlMode.PercentOutput, percent);
        leftBackAngleMotor.set(ControlMode.PercentOutput, percent);
        rightForwardMotor.set(ControlMode.PercentOutput, percent);
        rightForwardAngleMotor.set(ControlMode.PercentOutput, percent);
        rightBackMotor.set(ControlMode.PercentOutput, percent);
        rightBackAngleMotor.set(ControlMode.PercentOutput, percent);

    }

    public void setForwardBackMotors(double percent) {
        leftFront.setDriveMotors(-percent);
        rightFront.setDriveMotors(-percent);
        rightBack.setDriveMotors(-percent);
        leftBack.setDriveMotors(-percent);
    }

    public double getJoystickAngle(double joystickUp, double joystickSide) {
        if(Math.abs(joystickUp) < 0.05 && Math.abs(joystickSide) < 0.005) {
            return 0.0;
        }
        double joystickAngle = Math.atan2(-joystickUp, joystickSide);
        return joystickAngle;
    }

    public double getLeftForwardEncoder() {
        // frontLeftAbsoluteEncoder.setPosition(50);
        // System.out.println("Encoder: " + frontLeftAbsoluteEncoder.getAbsolutePosition());
        return leftFront.getAbsolutePosition();
    }

    public double getLeftBackEncoder() {
        // frontLeftAbsoluteEncoder.setPosition(50);
        // System.out.println("Encoder: " + frontLeftAbsoluteEncoder.getAbsolutePosition());
        return leftBack.getAbsolutePosition();
    }

    public double getRightForwardEncoder() {
        // frontLeftAbsoluteEncoder.setPosition(50);
        // System.out.println("Encoder: " + frontLeftAbsoluteEncoder.getAbsolutePosition());
        return rightFront.getAbsolutePosition();
    }

    public double getRightBackEncoder() {
        // frontLeftAbsoluteEncoder.setPosition(50);
        // System.out.println("Encoder: " + frontLeftAbsoluteEncoder.getAbsolutePosition());
        return rightBack.getAbsolutePosition();
    }

    public void setLeftForwardAngleEncoder(double targetAngle){
        leftForwardAngleMotor.setSelectedSensorPosition(targetAngle);
    }

    public void setLeftBackAngleEncoder(double targetAngle){
        leftBackAngleMotor.setSelectedSensorPosition(targetAngle);
    }

    public void setRightForwardAngleEncoder(double targetAngle){
        rightForwardAngleMotor.setSelectedSensorPosition(targetAngle);
    }

    public void setRightBackAngleEncoder(double targetAngle){
        rightBackAngleMotor.setSelectedSensorPosition(targetAngle);
    }

    public void postMotorEncoders() {
        SmartDashboard.putNumber("LeftForwardMotor", getRightBackEncoder());
    }

    public void postAngleMotorValues() {
        SmartDashboard.putNumber("LeftForwardAngle", radiansToDegrees(ticsToRadians(rightBackAngleMotor.getSelectedSensorPosition())));
        SmartDashboard.putNumber("LFTICS", leftForwardAngleMotor.getSelectedSensorPosition());
        // SmartDashboard.putNumber("RightForwardAngle", 360 * (rightForwardAngleMotor.getSelectedSensorPosition())/3700);
        // SmartDashboard.putNumber("LeftBackAngle", 360 * (leftBackAngleMotor.getSelectedSensorPosition())/3700);
        // SmartDashboard.putNumber("RightBackAngle", 360 * (rightBackAngleMotor.getSelectedSensorPosition())/3700);
    }

    public void postIntegratedAngleValues(){
        SmartDashboard.putNumber("Left Angle Integrated", leftForwardAngleMotor.getSelectedSensorPosition());
    }

    public double radiansToTics(double radians) {
        double outputTics = 0.5 * radians * (36833/Math.PI);
        return outputTics;
    }

    public double ticsToRadians(double tics) {
        double outputRadians = 2 * tics * (Math.PI/36833);
        return outputRadians;
    }

    public double radiansToDegrees(double radians) {
        double outputDegrees = 180 * radians/Math.PI;
        return outputDegrees;
    }

    public double degreesToRadians(double degrees) {
        double outputRadians = Math.PI * degrees/180;
        return outputRadians;
    }

    public double getDriveMotorPercent(double joystickUp, double joystickSide) {
        if(Math.abs(joystickUp) < 0.1) {
            joystickUp = 0;
        }
        if(Math.abs(joystickSide) < 0.1) {
            joystickSide = 0;
        }

        System.out.println("Adjusted Y: " + joystickUp);


        joystickUp = Math.abs(joystickUp);
        joystickSide = Math.abs(joystickSide);

        adjustedX = joystickSide * Math.sqrt((1-(Math.pow(joystickUp, 2))/2));
        adjustedY = joystickUp * Math.sqrt((1-(Math.pow(joystickSide, 2))/2));

        SmartDashboard.putNumber("Adj Y", adjustedY);

        double upSquared = Math.pow(adjustedY, 2);
        double sideSquared = Math.pow(adjustedX, 2);
        double currentMotorPercent = Math.sqrt(upSquared + sideSquared);

        SmartDashboard.putNumber("UP", upSquared);
        SmartDashboard.putNumber("Side", sideSquared);
        // double currentMotorPercent = (Math.sqrt(Math.abs(Math.pow(joystickUp, 2)) + Math.abs(Math.pow(joystickSide, 2))))/Math.sqrt(2);
        SmartDashboard.putNumber("Current Motor Percent", currentMotorPercent);
        return currentMotorPercent;
    }

    public void init() {
        setDefaultCommand(new DriveDefault(this));

        leftFront.init();
        leftBack.init();
        rightBack.init();
        rightFront.init();
        peripherals.zeroNavx();
    }

    private Command DriveDefault() {
        return null;
    }

    private void rightNoFlip(double wantAngle, double percent){
        double setAngle = wantAngle;
        setAnglePid(setAngle, 0, percent);
    }

    private void leftNoFlip(double wantAngle, double percent){
        double setAngle = wantAngle - degreesToRadians(360);
        setAnglePid(setAngle, 0 , percent);
    }

    private void rightAndFlip(double wantAngle, double percent){
        double setAngle = wantAngle + 180;
        setAnglePid(setAngle, 0 , -percent);
    }

    private void leftAndFlip(double wantAngle, double percent){
        double setAngle = (wantAngle + 180) - degreesToRadians(360);
        setAnglePid(setAngle, 0 , -percent);
    }

    public void driveOptimizer(double wantAngle, double percent){
        initAngle = ticsToRadians(leftFront.getAngleMotorEncoder());
        diffAngle = Math.abs(wantAngle - initAngle);
        double anglePos = radiansToDegrees(ticsToRadians(leftForwardAngleMotor.getSelectedSensorPosition()));   

        System.out.println("Position: " + anglePos);
        double caseOneAngle = wantAngle;
        double caseTwoAngle = wantAngle - degreesToRadians(360);
        double caseThreeAngle = wantAngle + 180;
        double caseFourAngle = (wantAngle + 180) - degreesToRadians(360);

        double distanceOne = Math.abs(90-caseOneAngle);
        double distanceTwo = Math.abs(90-caseTwoAngle);
        double distanceThree = Math.abs(90-caseThreeAngle);
        double distanceFour = Math.abs(90-caseFourAngle);

        if((distanceOne < distanceTwo) && (distanceOne < distanceThree) && (distanceOne < distanceFour)){
            setAnglePid(caseOneAngle, 0 , percent);
        }
        if((distanceTwo < distanceOne) && (distanceTwo < distanceThree) && (distanceTwo < distanceFour)){
            setAnglePid(caseTwoAngle, 0 , percent);
        }
        if((distanceThree < distanceOne) && (distanceThree < distanceTwo) && (distanceThree < distanceFour)){
            setAnglePid(caseThreeAngle, 0 , -percent);
        }
        if((distanceFour < distanceOne) && (distanceFour < distanceTwo) && (distanceFour < distanceThree)){
            setAnglePid(caseFourAngle, 0 , -percent);
        }



        // if(diffAngle > (degreesToRadians(90)) && (wantAngle > initAngle)){
        //     setAngle = wantAngle + degreesToRadians(180);
        //     setAnglePid(setAngle, 0 , -percent);
        //     SmartDashboard.putBoolean("Case 1", true);
        //     SmartDashboard.putBoolean("Case 4", false);
        //     SmartDashboard.putBoolean("Case 2", false);
        //     SmartDashboard.putBoolean("Case 3", false);
        //   }
        //   else if(diffAngle > degreesToRadians(90) && (wantAngle < initAngle)){
        //     setAngle = wantAngle + degreesToRadians(180);
        //     setAnglePid(setAngle, 0, -percent);
        //     SmartDashboard.putBoolean("Case 2", true);
        //     SmartDashboard.putBoolean("Case 1", false);
        //     SmartDashboard.putBoolean("Case 3", false);
        //     SmartDashboard.putBoolean("Case 4", false);
        //   }
        //   else if(diffAngle < degreesToRadians(90) && (wantAngle < initAngle)){
        //     setAnglePid(wantAngle, 0, percent);
        //     SmartDashboard.putBoolean("Case 3", true);
        //     SmartDashboard.putBoolean("Case 2", false);
        //     SmartDashboard.putBoolean("Case 1", false);
        //     SmartDashboard.putBoolean("Case 4", false);
        //   }
        //   else if(diffAngle < degreesToRadians(90) && (wantAngle > initAngle)){
        //     setAnglePid(wantAngle, 0, percent);
        //     SmartDashboard.putBoolean("Case 4", true);
        //     SmartDashboard.putBoolean("Case 1", false);
        //     SmartDashboard.putBoolean("Case 3", false);
        //     SmartDashboard.putBoolean("Case 2", false);
        //   }
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