package org.frc5687.chargedup.commands;
import static org.frc5687.chargedup.util.SuperStructureSetpoints.*;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.Auto.AutoGroundPickupCone;
import org.frc5687.chargedup.subsystems.*;
import org.frc5687.chargedup.util.SuperStructureSetpoints.Setpoint;

public class AutoGroundIntake extends OutliersCommand {
     private Arm _arm;
     private EndEffector _wrist;
     private Elevator _elevator;
     private OI _oi;
     private CubeShooter _shooter;

     


    
    public AutoGroundIntake(Arm arm, EndEffector endEffector, Elevator elevator, CubeShooter cubeShooter, OI oi){
        
    _arm = arm;
    _wrist = endEffector;
    _elevator = elevator;
    _oi = oi;
    _shooter = cubeShooter;
    addRequirements(_arm, _elevator, _wrist);
    }

    @Override
    public void initialize() {
        new AutoGroundPickupCone(_elevator, _arm, _wrist, _shooter, _oi);
    }

    @Override
    public void execute() {
        
    }
    @Override
    public boolean isFinished() {
       return _oi.getButtonNine();
        
    }
    @Override
    public void end(boolean interrupted) {
        Setpoint _idleSetpoint = idleConeSetpoint;
        new AutoSetSuperStructurePosition(_elevator, _wrist, _arm, _idleSetpoint);
        _shooter.setWristAngle(Constants.CubeShooter.IDLE_ANGLE);
    }
    
}
