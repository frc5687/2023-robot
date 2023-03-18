package org.frc5687.chargedup.commands.CubeShooter;

import edu.wpi.first.math.Pair;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.CubeShooter;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.subsystems.EndEffector;

public class IdleWrist extends OutliersCommand {

    private final CubeShooter _cubeShooter;
    private final DriveTrain _driveTrain;
    private final EndEffector _endEffector;
    public IdleWrist(CubeShooter shooter, DriveTrain driveTrain, EndEffector endEffector) {
        _cubeShooter = shooter;
        _driveTrain = driveTrain;
        _endEffector = endEffector;
        addRequirements(_cubeShooter);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        if (_cubeShooter.isCubeDetected()) {
            Pair<Double, Double> params = _cubeShooter.getShootingParameters(_driveTrain.getDistanceToGoal(), _endEffector.getLevelGoal());
            _cubeShooter.setWristAngle(params.getSecond());
        } else {
            _cubeShooter.setWristAngle(Constants.CubeShooter.IDLE_ANGLE);
        }
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
