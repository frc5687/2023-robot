package org.frc5687.chargedup.commands;

import org.frc5687.chargedup.subsystems.Elevator;

public class Rickroll extends OutliersCommand {
   private Elevator _music;
    
    @Override
    public void initialize() {
        
        super.initialize();
    }
    @Override
    public void execute() {
       _music.Rickroll();




    }
}
