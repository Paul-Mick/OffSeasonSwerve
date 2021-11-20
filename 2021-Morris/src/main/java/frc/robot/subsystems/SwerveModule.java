// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
    private final int mModuleNumber;
    
    private final OI OI = new OI();

	private final double mZeroOffset;

	private final TalonFX angleMotor;
    private final TalonFX driveMotor;
    
    private double turnVectorX = 0;
    private double turnVectorY = 0;

    private double turnPower = 1;

    private final CANCoder absoluteEncoder;

    public SwerveModule(int moduleNumber, TalonFX mAngleMotor, TalonFX mDriveMotor, double zeroOffset, CANCoder mAbsoluteEncoder) {
        mModuleNumber = moduleNumber;

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
        // p = 0.085, i = 0.0009, d = 0.9
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
        // if(Math.abs(joystickUp) < 0.05 && Math.abs(joystickSide) < 0.005) {
        //     return 0.0;
        // }
        double joystickAngle = Math.atan2(-joystickUp, joystickSide);
        // System.out.println("Angle: " + joystickAngle);
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

    public double getAbsolutePosition() {
        return absoluteEncoder.getAbsolutePosition();
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

    public double getModulePosition(){
        return ticsToRadians(getAngleMotorEncoder());
    }

    public void vectorCalculations(double targetX, double targetY, double navxOffset, double turnPercent) {
        // navxOffset = 0;
        double targetAngle = Math.atan2(targetY, targetX);
        targetAngle = targetAngle + degreesToRadians(90) + degreesToRadians(navxOffset);

        double hypotenuse = Math.sqrt(Math.pow(targetX, 2) + Math.pow(targetY, 2));

        double navxAdjustedX = hypotenuse * Math.cos(targetAngle);
        double navxAdjustedY = hypotenuse * Math.sin(targetAngle);

        // System.out.println()

        double turnX = turnPercent * turnVectorX;
        double turnY = turnPercent * turnVectorY;

        // System.out.println("Adj X: " + turnX + " Adj Y : " + turnY);


        double adjustedVectorX = turnX + navxAdjustedX;
        double adjustedVectorY = turnY + navxAdjustedY;

        double motorPercent = Math.sqrt(Math.pow(adjustedVectorX, 2) + Math.pow(adjustedVectorY, 2));
        double adjustedAngle = Math.atan2(adjustedVectorY, adjustedVectorX);

        double initAngle = getModulePosition();
        double boundedInitAngle = initAngle%Math.toRadians(360);
        // if(boundedInitAngle < 0){
        //     boundedInitAngle += Math.toRadians(360);
        // }

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

       
       // setAnglePID(adjustedAngle, motorPercent);
    }

    public void swerveModule(double navxOffset, double driveMotorPercent, double turnPercent) {
        // System.out.println("NAVX: " + navxOffset);
        // System.out.println("Turn: " + OI.getDriverRightX())
        // System.out.println("Turn Percent: " + turnPercent);
        if(OI.driverController.getAButton()) {
            // drive.setAngleMotors(0.2);
            // driveOptimizer(90, 0);
            setAnglePID(degreesToRadians(90), 0);
        }
        else if(OI.driverController.getBButton()) {
            // drive.setForwardBackMotors((0.2));
            // driveOptimizer(181, 0);
            setAnglePID(degreesToRadians(180), 0);
        }
        else if(OI.driverController.getYButton()) {
            // driveOptimizer(270, 0);
            setAnglePID(degreesToRadians(270), 0);
        }
        else if(OI.driverController.getXButton()) { 
            // driveOptimizer(1, 0);
            setAnglePID(degreesToRadians(0), 0);
        }
        else if(OI.getDriverLeftY() != 0 || OI.getDriverLeftX() != 0){
            vectorCalculations(OI.getDriverLeftX(), -OI.getDriverLeftY(), navxOffset, turnPercent);
        // drive.setForwardBackMotors(drive.getDriveMotorPercent(OI.getDriverLeftY(), OI.getDriverLeftX()));
        }
        else {
        //  drive.setDriveMotorPercents(0);
        }
    }

}
