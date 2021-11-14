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

    private double adjustedX = 0.0;
    private double adjustedY = 0.0;

    private CANCoder backRightAbsoluteEncoder = new CANCoder(2);
    private CANCoder frontLeftAbsoluteEncoder = new CANCoder(1);
    private CANCoder frontRightAbsoluteEncoder = new CANCoder(5);
    private CANCoder backLeftAbsoluteEncoder = new CANCoder(3);

    private double kF = 0;
    private double kP = 0.01;
    private double kI = 0;
    private double kD = 0;

    public Drive() {
    }

    public void setDriveMotorPercents(double percent) {
        leftForwardMotor.set(ControlMode.PercentOutput, percent);
        leftBackMotor.set(ControlMode.PercentOutput, percent);
        rightForwardMotor.set(ControlMode.PercentOutput, percent);
        rightBackMotor.set(ControlMode.PercentOutput, percent);

        leftForwardAngleMotor.set(ControlMode.PercentOutput, percent);
        leftBackAngleMotor.set(ControlMode.PercentOutput, percent);
        rightForwardAngleMotor.set(ControlMode.PercentOutput, percent);
        rightBackAngleMotor.set(ControlMode.PercentOutput, percent);
    }

    public void setAnglePid(double targetAngle, double navxOffset) {
        SmartDashboard.putNumber("Target", targetAngle);
        leftForwardAngleMotor.set(ControlMode.Position, ((targetAngle)/360) * 37000);
        leftBackAngleMotor.set(ControlMode.Position, ((targetAngle)/360) * 37000);
        rightForwardAngleMotor.set(ControlMode.Position, ((targetAngle)/360) * 37000);
        rightBackAngleMotor.set(ControlMode.Position, ((targetAngle)/360) * 37000);
    }

    public double getJoystickAngle(double joystickUp, double joystickSide) {
        if(Math.abs(joystickUp) < 0.05 && Math.abs(joystickSide) < 0.005) {
            return 0.0;
        }
        double joystickAngle = Math.atan2(-joystickUp, joystickSide);
        joystickAngle = (joystickAngle * 180/Math.PI) + 180;
        return joystickAngle;
    }

    public void setForwardBackMotors(double percent) {
        leftForwardMotor.set(ControlMode.PercentOutput, percent);
        // leftBackMotor.set(ControlMode.PercentOutput, percent);
        // rightForwardMotor.set(ControlMode.PercentOutput, percent);
        // rightBackMotor.set(ControlMode.PercentOutput, percent);
    }

    public void setAngleMotors(double percent) {
        leftForwardAngleMotor.set(ControlMode.PercentOutput, percent);
        leftBackAngleMotor.set(ControlMode.PercentOutput, percent);
        rightForwardAngleMotor.set(ControlMode.PercentOutput, percent);
        rightBackAngleMotor.set(ControlMode.PercentOutput, percent);
    }

    public double getLeftForwardEncoder() {
        // frontLeftAbsoluteEncoder.setPosition(50);
        // System.out.println("Encoder: " + frontLeftAbsoluteEncoder.getAbsolutePosition());
        return frontLeftAbsoluteEncoder.getAbsolutePosition();
    }

    public double getLeftBackEncoder() {
        // frontLeftAbsoluteEncoder.setPosition(50);
        // System.out.println("Encoder: " + frontLeftAbsoluteEncoder.getAbsolutePosition());
        return backLeftAbsoluteEncoder.getAbsolutePosition();
    }

    public double getRightForwardEncoder() {
        // frontLeftAbsoluteEncoder.setPosition(50);
        // System.out.println("Encoder: " + frontLeftAbsoluteEncoder.getAbsolutePosition());
        return frontRightAbsoluteEncoder.getAbsolutePosition();
    }

    public double getRightBackEncoder() {
        // frontLeftAbsoluteEncoder.setPosition(50);
        // System.out.println("Encoder: " + frontLeftAbsoluteEncoder.getAbsolutePosition());
        return backRightAbsoluteEncoder.getAbsolutePosition();
    }

    public void postMotorEncoders() {
        SmartDashboard.putNumber("LeftForwardMotor", getLeftForwardEncoder());
    }

    public void postAngleMotorValues() {
        SmartDashboard.putNumber("LeftForwardAngle", 360 * (leftForwardAngleMotor.getSelectedSensorPosition())/37000);
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

        leftForwardAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

        // leftForwardAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        leftForwardAngleMotor.configPeakOutputForward(0.7);
        leftForwardAngleMotor.configPeakOutputReverse(-0.7);
        leftForwardAngleMotor.configVoltageCompSaturation(11.7);
        leftForwardAngleMotor.enableVoltageCompensation(true);
        leftForwardAngleMotor.setSensorPhase(true);
        leftForwardAngleMotor.selectProfileSlot(0, 0);
        leftForwardAngleMotor.config_kF(0, 0.0);
        leftForwardAngleMotor.config_kP(0, 0.085);
        leftForwardAngleMotor.config_kI(0, 0.0009);
        leftForwardAngleMotor.config_kD(0, 0.9);

        leftForwardMotor.setNeutralMode(NeutralMode.Brake);
        rightForwardMotor.setNeutralMode(NeutralMode.Brake);
        leftBackMotor.setNeutralMode(NeutralMode.Brake);
        rightBackMotor.setNeutralMode(NeutralMode.Brake);

        // frontLeftAbsoluteEncoder.set(0);

        leftForwardAngleMotor.setSelectedSensorPosition(getLeftForwardEncoder() * 37000/360);
        leftBackAngleMotor.setSelectedSensorPosition(getLeftBackEncoder() * 37000/360);
        rightForwardAngleMotor.setSelectedSensorPosition(getRightForwardEncoder() * 37000/360);
        rightBackAngleMotor.setSelectedSensorPosition(getRightBackEncoder() * 37000/360);

    }

    private Command DriveDefault() {
        return null;
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