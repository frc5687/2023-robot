package org.frc5687.chargedup.commands.Auto;

import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.DriveTrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class DriveForTime extends OutliersCommand{
    private final DriveTrain _driveTrain;
    private long _timeout;
    

    public DriveForTime(DriveTrain drivetrain, long timeout){
        _driveTrain = drivetrain;
        _timeout = timeout;
        addRequirements(_driveTrain);
    }
   
    @Override
    public void initialize() {
        super.initialize();
        _timeout = System.currentTimeMillis() + _timeout;
        error("Starting Drive");
    }
    @Override
    public void execute() {
        super.execute();
        _driveTrain.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(2.0, 0,0, _driveTrain.getHeading()));
    }
    @Override
    public boolean isFinished() {
        return _timeout < System.currentTimeMillis();
}
    @Override
    public void end(boolean interrupted) {
        _driveTrain.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0,0, _driveTrain.getHeading()));
        error("ending drive");
        super.end(interrupted);
        _driveTrain.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0,0, _driveTrain.getHeading()));
 
    }
}