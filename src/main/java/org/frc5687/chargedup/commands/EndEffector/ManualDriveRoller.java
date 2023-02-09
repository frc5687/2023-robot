package org.frc5687.chargedup.commands.EndEffector;

import org.frc5687.chargedup.subsystems.EndEffector;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.OutliersCommand;

public class ManualDriveRoller extends OutliersCommand {
    
    private EndEffector _gripper;
    private OI _oi;

    public ManualDriveRoller(EndEffector gripper, OI oi) {
      _gripper = gripper;  
      _oi = oi;
      addRequirements(_gripper);
   }
     
    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        if (_oi.getIntakeIn()){
            _gripper.setRollerSpeed(1);
        } else if (_oi.getIntakeOut()){
            _gripper.setRollerSpeed(-1);
        } else {
            _gripper.setRollerSpeed(0);
        }
        // metric("Gripper Speed", speed);
    }
    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
