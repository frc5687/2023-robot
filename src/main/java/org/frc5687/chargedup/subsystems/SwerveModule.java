/* Team 5687  */
package org.frc5687.chargedup.subsystems;
import static org.frc5687.chargedup.Constants.SwerveModule.*;

import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.chargedup.Constants;


import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenixpro.BaseStatusSignalValue;
import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.controls.VelocityTorqueCurrentFOC;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.math.util.Units;


// Swerve Module Code Created in the shadow of
// the death of diffy chargedup by Linus Krenkel
// using VelocityFOC and PositionVoltage 
public class SwerveModule {

    private final OutliersTalon _driveMotor;
    private final OutliersTalon _turningMotor;
    private final CANCoder _encoder;

   
    private Translation2d _positionVector;

    private SwerveModuleState _goal;

    private final StatusSignalValue<Double> _driveVelocityRotationsPerSec;
    private final StatusSignalValue<Double> _drivePositionRotations;
    private final StatusSignalValue<Double> _turningVelocityRotationsPerSec;
    private final StatusSignalValue<Double> _turningPositionRotations;

    private final BaseStatusSignalValue[] _signals;

    private final VelocityTorqueCurrentFOC _velocityTorqueCurrentFOC;
    private final PositionVoltage _positionVoltage;

    private double _rotPerMet = 0.0;
    private double _gearRatio;
    private double _metPerRot;


    
    public SwerveModule(
        SwerveModule.ModuleConfiguration config,
        int steeringMotorID,
        int driveMotorID,
        int encoderPort
        ){
           
            _driveMotor = new OutliersTalon(driveMotorID, config.canBus, "Drive");
            _turningMotor = new OutliersTalon(steeringMotorID, config.canBus, "Steer");

            _driveMotor.configure(Constants.SwerveModule.CONFIG);
            _turningMotor.configure(Constants.SwerveModule.CONFIG);
            _driveMotor.setTorqueCurrentFOCRate(1000);
            _turningMotor.setTorqueCurrentFOCRate(1000);

            _velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0.0);
            _positionVoltage = new PositionVoltage(0.0);

         
            _goal = new SwerveModuleState(0.0, getCanCoderAngle());


            _encoder = new CANCoder(encoderPort, config.canBus);
            CANCoderConfiguration CANfig = new CANCoderConfiguration();
            // set units of the CANCoder to radians, with velocity being radians per second
            CANfig.sensorCoefficient = 2 * Math.PI / 4096.0;
            CANfig.unitString = "rad";
            CANfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
            CANfig.sensorTimeBase = SensorTimeBase.PerSecond;
            CANfig.magnetOffsetDegrees = Constants.SwerveModule.CAN_OFFSET;
            _encoder.configAllSettings(CANfig);

            _rotPerMet = (_gearRatio * _metPerRot);
            _gearRatio = Constants.SwerveModule.GEAR_RATIO_DRIVE;
            _metPerRot = 2 * Math.PI * Units.inchesToMeters(Constants.SwerveModule.WHEEL_RADIUS);

            _positionVector = config.position;
            
            _drivePositionRotations = _driveMotor.getPosition();
            _driveVelocityRotationsPerSec = _driveMotor.getVelocity();
            _turningPositionRotations = _turningMotor.getPosition();
            _turningVelocityRotationsPerSec = _turningMotor.getVelocity();

            _driveVelocityRotationsPerSec.setUpdateFrequency(1 / kDt);
            _drivePositionRotations.setUpdateFrequency(1 / kDt);

           _turningVelocityRotationsPerSec.setUpdateFrequency(1 / kDt);
           _turningPositionRotations.setUpdateFrequency(1 / kDt);

           _signals = new BaseStatusSignalValue[4];
           _signals[0] = _driveVelocityRotationsPerSec;
           _signals[1] = _drivePositionRotations;
           _signals[2] = _turningVelocityRotationsPerSec;
           _signals[3] = _turningPositionRotations;
           //_controlState = ControlState.OFF;
         
        }
        // public ControlState getControlState() {
        //     return _controlState;
        // }
        // public void setControlState(ControlState state) {
        //     _controlState = state;
        // }
        public BaseStatusSignalValue[] getSignals() {
            return _signals;
        }
        public void setIdealState( SwerveModuleState state){
            if (state.speedMetersPerSecond < 0.1 ){
            StopAll();
            } else {
            state = SwerveModuleState.optimize(state, getCanCoderAngle());
            _goal = state;
            setModuleState(_goal);
            }
        }
        public void setModuleState(SwerveModuleState state){
            double speed = state.speedMetersPerSecond * _rotPerMet;
            double position = state.angle.getDegrees();
            _driveMotor.setControl(_velocityTorqueCurrentFOC.withVelocity(speed));
            _turningMotor.setControl(_positionVoltage.withPosition(position));

        }
        public SwerveModuleState getState(){
            return new SwerveModuleState(getWheelVelocity(), getCanCoderAngle());
        }
        public void StopAll(){
            _driveMotor.stopMotor();
            _turningMotor.stopMotor();
        }
        public double getEncoderAngledouble(){
            return _encoder.getAbsolutePosition();
        }
        
        public Rotation2d getCanCoderAngle() {
            return Rotation2d.fromDegrees(_encoder.getAbsolutePosition());
        }
        public double getDriveRPM(){
            return OutliersTalon.rotationsPerSecToRPM(_driveVelocityRotationsPerSec.getValue(), 1.0);
        }
        public double getTurningRPM(){
            return OutliersTalon.rotationsPerSecToRPM(_turningVelocityRotationsPerSec.getValue(), 1.0);
        }
        public double getWheelVelocity() {
            return getWheelAngularVelocity() * Constants.SwerveModule.WHEEL_RADIUS; // Meters per sec.
        }
        public double getWheelAngularVelocity(){
            return Units.rotationsPerMinuteToRadiansPerSecond(getDriveRPM() / Constants.SwerveModule.GEAR_RATIO_DRIVE);
        }
        public Translation2d getModuleLocation() {
            return _positionVector;
        }
        public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getWheelDistance(), getCanCoderAngle());
    }
    public double getWheelDistance() {
        return (getDistance())
                * WHEEL_RADIUS;
    }
    public double getDistance(){
        return _drivePositionRotations.getValue() * (Math.PI * 2.0);
    }
    public void resetEncoders() {
        _driveMotor.setRotorPosition(0);
        _turningMotor.setRotorPosition(0);
    }
     public void updateDashboard() {
        //        SmartDashboard.putNumber(_name + "/leftVoltage", _leftFalcon.getMotorOutputVoltage());
        //        SmartDashboard.putNumber(_name + "/rightVoltage",
        // _rightFalcon.getMotorOutputVoltage());
        //SmartDashboard.putNumber(_name + "/leftNextCurrent", getLeftNextCurrent());
        //SmartDashboard.putNumber(_name + "/rightNextCurrent", getRightNextCurrent());
        //        SmartDashboard.putNumber(_name + "/leftSupplyCurrent",
        // _leftFalcon.getSupplyCurrent());
        //        SmartDashboard.putNumber(_name + "/rightSupplyCurrent",
        // _rightFalcon.getSupplyCurrent());
        // SmartDashboard.putNumber(_name + "/leftStatorCurrent", getLeftCurrent());
        // SmartDashboard.putNumber(_name + "/rightStatorCurrent", getRightCurrent());
        // SmartDashboard.putNumber(_name + "/referenceAngleGoal", getReferenceModuleAngle());
        // SmartDashboard.putNumber(_name + "/moduleAngle", getModuleAngle());
        // SmartDashboard.putNumber(_name + "/moduleAngleABS", getABSEncoderAngle());

        // SmartDashboard.putNumber(_name + "/moduleAngVel", getAzimuthAngularVelocity());

        // SmartDashboard.putNumber(_name + "/velocityWheel", getWheelVelocity());
        // SmartDashboard.putNumber(_name + "/referenceWheelVelocity", getReferenceWheelVelocity());
        //
        //        SmartDashboard.putString(_name + "/KMatrix",
        // _moduleControlLoop.getController().getK().toString());
        //
        //        SmartDashboard.putNumber(_name + "/estimatedModuleAngle", getPredictedAzimuthAngle());
        //        SmartDashboard.putString(_name + "/refernce", _reference.toString());
    }

        

    public static class ModuleConfiguration {
        public String moduleName = "";

        public Translation2d position = new Translation2d();

        public double encoderOffset = 0.0;
        public boolean encoderInverted = false;

        public String canBus = "rio";
    }
    // public enum ControlState {
    //     OFF(0),
    //     STATE_CONTROL(1),
    //     CHARACTERIZATION(2);
    //     private final int _value;

    //     ControlState(int value) {
    //         _value = value;
    //     }

    //     public int getValue() {
    //         return _value;
    //     }
    // }


}
