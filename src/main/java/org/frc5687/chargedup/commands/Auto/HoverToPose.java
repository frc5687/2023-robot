package org.frc5687.chargedup.commands.Auto;

import static org.frc5687.chargedup.Constants.DriveTrain.DRIVE_POSE_KINEMATIC_LIMITS;
import static org.frc5687.chargedup.Constants.DriveTrain.KINEMATIC_LIMITS;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.CubeShooter;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.subsystems.Lights;
import org.frc5687.chargedup.subsystems.Lights.AnimationType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class HoverToPose extends OutliersCommand {

    private final DriveTrain _driveTrain;
    private final CubeShooter _cubeShooter;

    private Pose2d _offsetPose;
    private Transform2d _offset;

    private final Lights _lights;

    public HoverToPose(DriveTrain driveTrain, CubeShooter shooter, Lights lights) {
        _driveTrain = driveTrain;
        _cubeShooter = shooter;
        _lights = lights;
        addRequirements(_driveTrain, _lights);
    }

    @Override
    public void initialize() {
        super.initialize();
        _driveTrain.setKinematicLimits(DRIVE_POSE_KINEMATIC_LIMITS);
        _lights.switchAnimation(AnimationType.LARSON);
        _offset = new Transform2d(new Translation2d(0.3, 0), new Rotation2d());
        _offsetPose = _driveTrain.getHoverGoal().transformBy(_offset);
    }

    @Override
    public void execute() {
    if(_driveTrain.getEstimatedPose().getX() - _offsetPose.getX() > 0.05 && 
        _driveTrain.getEstimatedPose().getY() - _offsetPose.getY() > 0.05){
        _driveTrain.setVelocityPose(_offsetPose, _cubeShooter.isCubeDetected());
    }else{
        _driveTrain.setVelocityPose(_driveTrain.getHoverGoal(), _cubeShooter.isCubeDetected());
    }
    }

    @Override
    public void end(boolean interrupted) {
        _driveTrain.setKinematicLimits(KINEMATIC_LIMITS);
        _driveTrain.setMaintainHeading(_driveTrain.getHeading());
        super.end(interrupted);
    }
}
