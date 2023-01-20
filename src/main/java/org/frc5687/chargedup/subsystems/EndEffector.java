package org.frc5687.chargedup.subsystems;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.RobotMap;
import org.frc5687.chargedup.util.OutliersContainer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class EndEffector extends OutliersSubsystem {
   
    private final TalonSRX _wrist;
    private final TalonSRX _gripper;
    
    private final DutyCycleEncoder _wristEncoder;
    private final DutyCycleEncoder _gripperEncoder;


    public EndEffector(OutliersContainer container) {
        super(container);

        _wrist = new TalonSRX(RobotMap.CAN.TalonSRX.WRIST);
        _gripper = new TalonSRX(RobotMap.CAN.TalonSRX.GRIPPER);
        
        _wristEncoder = new DutyCycleEncoder(RobotMap.DIO.ENCODER_WRIST);
        _wristEncoder.setDistancePerRotation(2.0 * Math.PI);
        _gripperEncoder = new DutyCycleEncoder(RobotMap.DIO.ENCODER_GRIPPER);
        _gripperEncoder.setDistancePerRotation(2.0 * Math.PI);
    }  
    @Override
    public void updateDashboard() {
        
    }
    public void setWristSpeed(double demand){
        _wrist.set(ControlMode.PercentOutput, demand);
    }
    
    public void setGripperSpeed(double demand){
        _gripper.set(ControlMode.PercentOutput, demand);
    }

    public double getWristAngle(){
         return _wristEncoder.getDistance() % (2.0 * Math.PI) - Constants.EndEffector.WRIST_OFFSET;
    }
    public double getGripperAngle(){
        return _gripperEncoder.getDistance() % (2.0 * Math.PI) - Constants.EndEffector.GRIPPER_OFFSET;
    }
    
}
