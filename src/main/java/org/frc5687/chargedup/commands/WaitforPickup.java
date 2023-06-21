package org.frc5687.chargedup.commands;

import org.frc5687.chargedup.OI;

import edu.wpi.first.wpilibj.DriverStation;

public class WaitforPickup extends OutliersCommand {
    private OI _oi;
    private long _timeout;

    public WaitforPickup(OI oi){
        _oi = oi;


    }
    @Override
    public void initialize() {
        _timeout = System.currentTimeMillis() + 1000;
        System.out.println("starting wait");
        
    }
    @Override
    public void execute() {
        
    }
    @Override
    public boolean isFinished() {
        System.out.println("stowing ground intake");
        return (_oi.getButtonNine() && System.currentTimeMillis() > _timeout);
    }
    @Override
    public void end(boolean interrupted) {
        
}}
