package org.frc5687.chargedup.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.subsystems.EndEffector;
import org.frc5687.chargedup.util.Nodes;
import org.frc5687.chargedup.util.SuperStructureSetpoints;

/**
 * This command tells the arm and drivetrain to go to a specific setpoint and location based on a button press
 * For instance if they press Node 1 High, it sets the setpoint to be arm place high and drivetrain setpoint to node 1
 */
public class SetRobotGoal extends OutliersCommand{
    private DriveTrain _driveTrain;
    private EndEffector _endEffector;
    private Nodes.Node _node;
    private Nodes.SuperStructurePosition _superStructurePosition;
    public SetRobotGoal(DriveTrain driveTrain, EndEffector endEffector, Nodes.Node node, Nodes.SuperStructurePosition position) {
        _driveTrain = driveTrain;
        _endEffector = endEffector;
        _node = node;
        _superStructurePosition = position;
    }

    @Override
    public void initialize() {
        super.initialize();
        boolean isRedAlliance = _driveTrain.isRedAlliance();
        Pose2d pose;
        switch (_node) {
            case ONE:
                pose = isRedAlliance ? Constants.Auto.RED_FIRST_GOAL : Constants.
            case TWO:
            case THREE:
            case FOUR:
            case FIVE:
            case SIX:
            case SEVEN:
            case EIGHT:
            case NINE:
        }
        _endEffector.setSuperStructureSetpoint(_setpoint);
        _driveTrain.setHoverGoal(_pose);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
