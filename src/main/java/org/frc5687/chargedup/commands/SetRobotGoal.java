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
        error("Setting to position " + _superStructurePosition.name() + "on node " + _node.name());
        boolean isRedAlliance = _driveTrain.isRedAlliance();
        Pose2d pose = isRedAlliance ?
                Constants.Auto.FieldPoses.RED_NODE_ONE_GOAL :
                Constants.Auto.FieldPoses.BLUE_NODE_ONE_GOAL;
        SuperStructureSetpoints.Setpoint setpoint = SuperStructureSetpoints.idleConeSetpoint;
        switch (_node) {
            case ONE:
                switch (_superStructurePosition) {
                    case LOW:
                        setpoint = SuperStructureSetpoints.idleConeSetpoint;
                        break;
                    case MIDDLE:
                        setpoint = SuperStructureSetpoints.middleConePlaceSetpoint;
                        break;
                    case HIGH:
                        setpoint = SuperStructureSetpoints.highConePlaceSetpoint;
                        break;
                }
                pose = isRedAlliance ?
                        Constants.Auto.FieldPoses.RED_NODE_ONE_GOAL :
                        Constants.Auto.FieldPoses.BLUE_NODE_ONE_GOAL;
                break;
            case TWO:
                switch (_superStructurePosition) {
                    case LOW:
                        setpoint = SuperStructureSetpoints.idleCubeSetpoint;
                        break;
                    case MIDDLE:
                        setpoint = SuperStructureSetpoints.middleCubePlaceSetpoint;
                        break;
                    case HIGH:
                        setpoint = SuperStructureSetpoints.highCubePlaceSetpoint;
                        break;
                }
                pose = isRedAlliance ?
                        Constants.Auto.FieldPoses.RED_NODE_TWO_GOAL :
                        Constants.Auto.FieldPoses.BLUE_NODE_TWO_GOAL;
                break;
            case THREE:
                switch (_superStructurePosition) {
                    case LOW:
                        setpoint = SuperStructureSetpoints.idleConeSetpoint;
                        break;
                    case MIDDLE:
                        setpoint = SuperStructureSetpoints.middleConePlaceSetpoint;
                        break;
                    case HIGH:
                        setpoint = SuperStructureSetpoints.highConePlaceSetpoint;
                        break;
                }
                pose = isRedAlliance ?
                        Constants.Auto.FieldPoses.RED_NODE_THREE_GOAL:
                        Constants.Auto.FieldPoses.BLUE_NODE_THREE_GOAL;
                break;
            case FOUR:
                switch (_superStructurePosition) {
                    case LOW:
                        setpoint = SuperStructureSetpoints.idleConeSetpoint;
                        break;
                    case MIDDLE:
                        setpoint = SuperStructureSetpoints.middleConePlaceSetpoint;
                        break;
                    case HIGH:
                        setpoint = SuperStructureSetpoints.highConePlaceSetpoint;
                        break;
                }
                pose = isRedAlliance ?
                        Constants.Auto.FieldPoses.RED_NODE_FOUR_GOAL:
                        Constants.Auto.FieldPoses.BLUE_NODE_FOUR_GOAL;
                break;
            case FIVE:
                switch (_superStructurePosition) {
                    case LOW:
                        setpoint = SuperStructureSetpoints.idleCubeSetpoint;
                        break;
                    case MIDDLE:
                        setpoint = SuperStructureSetpoints.middleCubePlaceSetpoint;
                        break;
                    case HIGH:
                        setpoint = SuperStructureSetpoints.highCubePlaceSetpoint;
                }
                pose = isRedAlliance ?
                        Constants.Auto.FieldPoses.RED_NODE_FIVE_GOAL:
                        Constants.Auto.FieldPoses.BLUE_NODE_FIVE_GOAL;
                break;
            case SIX:
                switch (_superStructurePosition) {
                    case LOW:
                        setpoint = SuperStructureSetpoints.idleConeSetpoint;
                        break;
                    case MIDDLE:
                        setpoint = SuperStructureSetpoints.middleConePlaceSetpoint;
                        break;
                    case HIGH:
                        setpoint = SuperStructureSetpoints.highConePlaceSetpoint;
                        break;
                }
                pose = isRedAlliance ?
                        Constants.Auto.FieldPoses.RED_NODE_SIX_GOAL:
                        Constants.Auto.FieldPoses.BLUE_NODE_SIX_GOAL;
                break;
            case SEVEN:
                switch (_superStructurePosition) {
                    case LOW:
                        setpoint = SuperStructureSetpoints.idleConeSetpoint;
                        break;
                    case MIDDLE:
                        setpoint = SuperStructureSetpoints.middleConePlaceSetpoint;
                        break;
                    case HIGH:
                        setpoint = SuperStructureSetpoints.highConePlaceSetpoint;
                        break;
                }
                pose = isRedAlliance ?
                        Constants.Auto.FieldPoses.RED_NODE_SEVEN_GOAL :
                        Constants.Auto.FieldPoses.BLUE_NODE_SEVEN_GOAL;
                break;
            case EIGHT:
                switch (_superStructurePosition) {
                    case LOW:
                        setpoint = SuperStructureSetpoints.idleCubeSetpoint;
                        break;
                    case MIDDLE:
                        setpoint = SuperStructureSetpoints.middleCubePlaceSetpoint;
                        break;
                    case HIGH:
                        setpoint = SuperStructureSetpoints.highCubePlaceSetpoint;
                        break;
                }
                pose = isRedAlliance ?
                        Constants.Auto.FieldPoses.RED_NODE_EIGHT_GOAL :
                        Constants.Auto.FieldPoses.BLUE_NODE_EIGHT_GOAL;
                break;
            case NINE:
                switch (_superStructurePosition) {
                    case LOW:
                        setpoint = SuperStructureSetpoints.idleConeSetpoint;
                        break;
                    case MIDDLE:
                        setpoint = SuperStructureSetpoints.middleConePlaceSetpoint;
                        break;
                    case HIGH:
                        setpoint = SuperStructureSetpoints.highConePlaceSetpoint;
                        break;
                }
                pose = isRedAlliance ?
                        Constants.Auto.FieldPoses.RED_NODE_NINE_GOAL :
                        Constants.Auto.FieldPoses.BLUE_NODE_NINE_GOAL;
                break;
        }
        _endEffector.setSuperStructureSetpoint(setpoint);
        _driveTrain.setHoverGoal(pose);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
