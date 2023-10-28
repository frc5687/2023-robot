package org.frc5687.chargedup.commands;

import org.frc5687.chargedup.OI;

public class RumbleGamepad extends OutliersCommand {
    private OI _oi;
    private long _timeout;
    
    public RumbleGamepad(OI oi){
        _oi = oi;
    }
    @Override
    public void initialize() {
    _timeout = System.currentTimeMillis() + 250;

    }
    @Override
    public void execute() {
     _oi.RumbleDriver();

    }
    @Override
    public boolean isFinished() {
      return System.currentTimeMillis() > _timeout;  
    }
    @Override
    public void end(boolean interrupted) {
        _oi.stopRumbleDriver();
    }
    
}
