package org.frc5687.chargedup.commands;

import org.frc5687.chargedup.subsystems.DriveTrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SetCenterOfRotation extends OutliersCommand {
    private final DriveTrain _driveTrain;
    private Translation2d _corTranslation2d;

    public SetCenterOfRotation(DriveTrain driveTrain, Translation2d corTranslation2d){
        _driveTrain = driveTrain;
        _corTranslation2d = corTranslation2d;
    }

    @Override
    public void initialize() {
        if (_driveTrain.getHeading().getRadians() > new Rotation2d(Math.PI/2).getRadians() && 
        _driveTrain.getHeading().getRadians() < new Rotation2d(Math.PI).getRadians()){
            _corTranslation2d = new Translation2d(_corTranslation2d.getX(), _corTranslation2d.getY());
        } 
        else if (_driveTrain.getHeading().getRadians() > new Rotation2d(Math.PI).getRadians() && 
        _driveTrain.getHeading().getRadians() < new Rotation2d((3*Math.PI)/2).getRadians()){
            _corTranslation2d = new Translation2d(_corTranslation2d.getX() - 2*_corTranslation2d.getX(), _corTranslation2d.getY());
        } 
        else if (_driveTrain.getHeading().getRadians() > new Rotation2d((3*Math.PI)/2).getRadians() && 
        _driveTrain.getHeading().getRadians() < new Rotation2d(2*Math.PI).getRadians()){
            _corTranslation2d = new Translation2d(_corTranslation2d.getX() - 2*_corTranslation2d.getX(), _corTranslation2d.getY() - 2*_corTranslation2d.getY());
        }
        else if (_driveTrain.getHeading().getRadians() > new Rotation2d(2*Math.PI).getRadians() && 
        _driveTrain.getHeading().getRadians() < new Rotation2d(Math.PI/2).getRadians()){
            _corTranslation2d = new Translation2d(_corTranslation2d.getX(), _corTranslation2d.getY() - 2*_corTranslation2d.getY());
        }
    }

    @Override
    public void execute() {
        _driveTrain.setCenterOfRotation(_corTranslation2d);
    }


}
