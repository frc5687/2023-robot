package org.frc5687.chargedup.commands.Auto;

import static org.frc5687.chargedup.Constants.DriveTrain.DRIVE_POSE_KINEMATIC_LIMITS;
import static org.frc5687.chargedup.Constants.DriveTrain.KINEMATIC_LIMITS;

import edu.wpi.first.math.geometry.Pose2d;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.subsystems.Lights;
import org.frc5687.chargedup.subsystems.Lights.AnimationType;

public class HoverToPose extends OutliersCommand {

    private final DriveTrain _driveTrain;
    private Pose2d _destPose;
    private final OI _oi;
    private final Lights _lights;

    public HoverToPose(DriveTrain driveTrain, Lights lights, OI oi) {
        _driveTrain = driveTrain;
        _oi = oi;
        _lights = lights;
        addRequirements(_driveTrain, _lights);
    }

    @Override
    public void initialize() {
        super.initialize();
        _driveTrain.setKinematicLimits(DRIVE_POSE_KINEMATIC_LIMITS);
        _lights.switchAnimation(AnimationType.LARSON);
    }

    @Override
    public void execute() {
        _driveTrain.setVelocityPose(_driveTrain.getHoverGoal());
    }

    @Override
    public void end(boolean interrupted) {
        _driveTrain.setKinematicLimits(KINEMATIC_LIMITS);
        super.end(interrupted);
    }
}
