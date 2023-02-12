package org.frc5687.chargedup.subsystems;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.RobotMap;
import org.frc5687.chargedup.util.OutliersContainer;
import org.frc5687.lib.drivers.OutliersTalon;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import static org.frc5687.chargedup.Constants.EndEffector.*;

public class EndEffector extends OutliersSubsystem {
   
    private final TalonSRX _wrist;
    private final TalonSRX _gripper;
    
    private final DutyCycleEncoder _wristEncoder;
    private final DutyCycleEncoder _gripperEncoder;

    private final PIDController _wristController;
    private final PIDController _gripperController;

    public EndEffector(OutliersContainer container) {
        super(container);

        _wrist = new TalonSRX(RobotMap.CAN.TalonSRX.WRIST);
        _gripper = new TalonSRX(RobotMap.CAN.TalonSRX.GRIPPER);
        _wrist.setInverted(Constants.EndEffector.WRIST_INVERTED);
        _gripper.setInverted(Constants.EndEffector.GRIPPPER_INVERTED);
        
        _wristEncoder = new DutyCycleEncoder(RobotMap.DIO.ENCODER_WRIST);
        _wristEncoder.setDistancePerRotation(2.0 * Math.PI);
        _gripperEncoder = new DutyCycleEncoder(RobotMap.DIO.ENCODER_GRIPPER);
        _gripperEncoder.setDistancePerRotation(2.0 * Math.PI);

        _wristController = new PIDController(
            WRIST_kP,
            WRIST_kI,
            WRIST_kD
            // new TrapezoidProfile.Constraints(WRIST_VEL, WRIST_ACCEL)
        );
            
        _gripperController = new PIDController(
            GRIPPER_kP,
            GRIPPER_kI,
            GRIPPER_kD
            // new TrapezoidProfile.Constraints(GRIPPER_VEL, GRIPPER_ACCEL)
        );
        _gripperController.setIntegratorRange(-GRIPPER_I_ZONE, GRIPPER_I_ZONE);
    }  
    @Override
    public void updateDashboard() {
        metric("wrist angle deg", Units.radiansToDegrees(getWristAngleRadians()));
        metric("wrist angle rad", getWristAngleRadians());
        /*metric("gripper angle", Units.radiansToDegrees(getGripperAngleRadians()));*/

        // metric("wrist setpoint", _wristController.getGoal().position);
        metric("position error", _wristController.getPositionError());
        metric("Is Roller Stalled", isRollerStalled());
        metric("Roller Voltage", _gripper.getMotorOutputVoltage());
        metric("Roller Current", _gripper.getStatorCurrent());
    }
    public void setWristSpeed(double demand){
        _wrist.set(ControlMode.PercentOutput, demand);
    }
    
    public void setRollerSpeed(double demand){
        if (!isRollerStalled()) {
            _gripper.set(ControlMode.PercentOutput, demand);
        } else {
            _gripper.set(ControlMode.PercentOutput, 0);
        }
    }

    public boolean isRollerStalled(){ 
        return Math.abs(_gripper.getStatorCurrent()) > Constants.EndEffector.GRIPPER_STALL_CURRENT;
    }

    public double getWristAngleRadians(){
         return _wristEncoder.getDistance() % (2.0 * Math.PI); // - Constants.EndEffector.WRIST_OFFSET;
    }
    /*public double getGripperAngleRadians(){
        return _gripperEncoder.getDistance() % (2.0 * Math.PI); // - Constants.EndEffector.GRIPPER_OFFSET;
    }*/
    
    public void setWristSetpointDegrees(double degrees){
        _wristController.setSetpoint(Units.degreesToRadians(degrees));
    }

    public void setWristSetpointRadians(double radians){
        _wristController.setSetpoint(radians);
    }

    /*public void setGripperSetpointDegrees(double degrees){
        _gripperController.setSetpoint(Units.degreesToRadians(degrees));
    }*/

    /*public void setGripperSetpointRadians(double radians) {
        _gripperController.setSetpoint(radians);
    }*/
    public double getWristControllerOutput(){
      return  _wristController.calculate(getWristAngleRadians()); 
    }
    /*public double getGripperControllerOutput(){
        return _gripperController.calculate(getGripperAngleRadians());
    }*/
}
