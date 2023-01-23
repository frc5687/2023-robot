package org.frc5687.chargedup.subsystems;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.RobotMap;
import org.frc5687.chargedup.util.OutliersContainer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

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

    private final ProfiledPIDController _wristController;
    private final ProfiledPIDController _gripperController;

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

        _wristController = new ProfiledPIDController(
            WRIST_kP,
            WRIST_kI,
            WRIST_kD,
            new TrapezoidProfile.Constraints(WRIST_VEL, WRIST_ACCEL)
        );
            
        _gripperController = new ProfiledPIDController(
            GRIPPER_kP,
            GRIPPER_kI,
            GRIPPER_kD,
            new TrapezoidProfile.Constraints(GRIPPER_VEL, GRIPPER_ACCEL)
        );
    }  
    @Override
    public void updateDashboard() {
        metric("wrist angle", Units.radiansToDegrees(getWristAngleRadians()));
        metric("gripper angle", Units.radiansToDegrees(getGripperAngleRadians()));
    }
    public void setWristSpeed(double demand){
        _wrist.set(ControlMode.PercentOutput, demand);
    }
    
    public void setGripperSpeed(double demand){
        _gripper.set(ControlMode.PercentOutput, demand);
    }

    public double getWristAngleRadians(){
         return _wristEncoder.getDistance() % (2.0 * Math.PI) - Constants.EndEffector.WRIST_OFFSET;
    }
    public double getGripperAngleRadians(){
        return _gripperEncoder.getDistance() % (2.0 * Math.PI) - Constants.EndEffector.GRIPPER_OFFSET;
    }
    
    public void setWristSetpointDegrees(double degrees){
        _wristController.setGoal(Units.degreesToRadians(degrees));
    }

    public void setGripperSetpointDegrees(double degrees){
        _gripperController.setGoal(Units.degreesToRadians(degrees));
    }
    public double getWristControllerOutput(){
      return  _wristController.calculate(getWristAngleRadians()); 
    }
    public double getGripperControllerOutput(){
        return _gripperController.calculate(getGripperAngleRadians());
    }
}
