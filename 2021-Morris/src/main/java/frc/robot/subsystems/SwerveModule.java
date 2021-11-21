package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;

public class SwerveModule extends SubsystemBase {
	private final double mZeroOffset;

	private final TalonFX angleMotor;
    private final TalonFX driveMotor;
    
    private double turnVectorX = 0;
    private double turnVectorY = 0;

    private double turnPower = 0.5;

    private final CANCoder absoluteEncoder;

    public SwerveModule(int moduleNumber, TalonFX mAngleMotor, TalonFX mDriveMotor, double zeroOffset, CANCoder mAbsoluteEncoder) {
        // sets up the module by defining angle motor and drive motor
        angleMotor = mAngleMotor;
        driveMotor = mDriveMotor;

        mZeroOffset = zeroOffset;

        absoluteEncoder = mAbsoluteEncoder;

        angleMotor.configPeakOutputForward(1);
        angleMotor.configPeakOutputReverse(-1);
        angleMotor.configVoltageCompSaturation(11.7);
        angleMotor.enableVoltageCompensation(true);
        angleMotor.setSensorPhase(true);
        angleMotor.selectProfileSlot(0, 0);
        angleMotor.config_kF(0, 0.0);
        angleMotor.config_kP(0, 0.5);
        angleMotor.config_kI(0, 0);
        angleMotor.config_kD(0, 0);

        driveMotor.setNeutralMode(NeutralMode.Brake);

        if(moduleNumber == 1) {
            turnVectorX = turnPower * -Math.sqrt(2)/2.0;
            turnVectorY = turnPower * -Math.sqrt(2)/2.0;
        }
        if(moduleNumber == 2) {
            turnVectorX = turnPower * (Math.sqrt(2))/2.0;
            turnVectorY = turnPower * -Math.sqrt(2)/2.0;
        }
        if(moduleNumber == 3) {
            turnVectorX = turnPower * Math.sqrt(2)/2.0;
            turnVectorY = turnPower * Math.sqrt(2)/2.0;
        }
        if(moduleNumber == 4) {
            turnVectorX = turnPower * -Math.sqrt(2)/2.0;
            turnVectorY = turnPower * Math.sqrt(2)/2.0;
        }
    }

    public double getJoystickAngle(double joystickUp, double joystickSide) {
        double joystickAngle = Math.atan2(-joystickUp, joystickSide);
        return joystickAngle;
    }

    public void setAnglePID(double targetAngle, double motorPercent){
        angleMotor.set(ControlMode.Position, (radiansToTics((targetAngle))));
        setDriveMotors(motorPercent);
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

    public void setDriveMotors(double percent) {
        driveMotor.set(ControlMode.PercentOutput, percent);
    }

    public void init() {
        angleMotor.setSelectedSensorPosition(radiansToTics(degreesToRadians(absoluteEncoder.getAbsolutePosition())));
        angleMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 0 ,0));
        driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 0 , 0));
    }

    public double getAngleMotorEncoder(){
        return angleMotor.getSelectedSensorPosition();
    }

    public double getAbsolutePosition() {
        return absoluteEncoder.getAbsolutePosition();
    }

    public double getModulePosition(){
        return ticsToRadians(getAngleMotorEncoder());
    }

    public void vectorCalculations(double targetX, double targetY, double turnPercent, double navxOffset) {
        double targetAngle = Math.atan2(targetY, targetX);
        targetAngle = targetAngle + degreesToRadians(90) + degreesToRadians(navxOffset);

        double hypotenuse = Math.sqrt(Math.pow(targetX, 2) + Math.pow(targetY, 2));

        double navxAdjustedX = hypotenuse * Math.cos(targetAngle);
        double navxAdjustedY = hypotenuse * Math.sin(targetAngle);

        double turnX = turnPercent * turnVectorX;
        double turnY = turnPercent * turnVectorY;

        double adjustedVectorX = turnX + navxAdjustedX;
        double adjustedVectorY = turnY + navxAdjustedY;

        double motorPercent = Math.sqrt(Math.pow(adjustedVectorX, 2) + Math.pow(adjustedVectorY, 2));
        double adjustedAngle = Math.atan2(adjustedVectorY, adjustedVectorX);

        double initAngle = getModulePosition();
        double boundedInitAngle = initAngle%Math.toRadians(360);

        double caseOneAngle = adjustedAngle;
        if(adjustedAngle > boundedInitAngle){
            caseOneAngle = adjustedAngle - Math.toRadians(360);
        }
        //Case one moves clockwise
        double caseTwoAngle = adjustedAngle;
        if(adjustedAngle < boundedInitAngle){
         caseTwoAngle = adjustedAngle + Math.toRadians(360);

        }
        //Case two moves counterclockwise
        double caseThreeAngle = adjustedAngle + Math.toRadians(180);
        double caseFourAngle = (adjustedAngle + Math.toRadians(180)) - Math.toRadians(360);

        double distanceOne = Math.abs(boundedInitAngle - caseOneAngle);
        double distanceTwo = Math.abs(boundedInitAngle - caseTwoAngle);
        double distanceThree = Math.abs(boundedInitAngle - caseThreeAngle);
        double distanceFour = Math.abs(boundedInitAngle - caseFourAngle);

        if(motorPercent > 0.1){
            if((distanceOne < distanceTwo) && (distanceOne < distanceThree) && (distanceOne < distanceFour)){
                setAnglePID((caseOneAngle - boundedInitAngle + initAngle), motorPercent);
            }
            if((distanceTwo < distanceOne) && (distanceTwo < distanceThree) && (distanceTwo < distanceFour)){
                setAnglePID((caseTwoAngle - boundedInitAngle + initAngle), motorPercent);
            }
            if((distanceThree < distanceOne) && (distanceThree < distanceTwo) && (distanceThree < distanceFour)){
                setAnglePID((caseThreeAngle - boundedInitAngle + initAngle),  -motorPercent);
            }
            if((distanceFour < distanceOne) && (distanceFour < distanceTwo) && (distanceFour < distanceThree)){
                setAnglePID((caseFourAngle - boundedInitAngle + initAngle),  -motorPercent);
            }
        } 
        else{
            driveMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    public void moduleDrive(double x, double y, double turn, double offset) {
        vectorCalculations(x, y,turn, offset);
    }

}
