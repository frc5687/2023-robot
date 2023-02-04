package org.frc5687.chargedup.commands.Elevator;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.Elevator;

import edu.wpi.first.math.util.Units;

public class AutoExtendElevator extends OutliersCommand {
    private Elevator _extArm;
    private double _extDistance;

    public AutoExtendElevator(Elevator extArm, double extDistance){
        _extArm = extArm;
        _extDistance = extDistance;
        addRequirements(_extArm);
    }
    @Override
    public void initialize() {
        super.initialize();
        //Setpoint in Radians
        // error("Starting Extend Arm Command");
    }
    
    @Override
    public void execute() {
        super.execute();
        _extArm.setExtArmLengthMeters(_extDistance);

    }
    @Override
    public boolean isFinished(){
        // error("Finished Extending");
        // error("Difference between setpoint and position: " + (_extDistance - _extArm.getExtArmMeters()));
        // error("Value is: " + (Math.abs(_extDistance - _extArm.getExtArmMeters()) < Constants.ExtendingArm.EXT_ARM_TOLERANCE));
        return Math.abs(_extDistance - _extArm.getExtArmMeters()) < Constants.ExtendingArm.EXT_ARM_TOLERANCE;
    }

    public void end(boolean interrupted){
        super.end(interrupted);
    }
}
