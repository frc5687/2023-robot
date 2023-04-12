package org.frc5687.chargedup.util;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.Pair;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

/** This class contains all the trajectories we generate from path-planner; */
public class Trajectories {
    private HashMap<String, PathPlannerTrajectory> trajectories = new HashMap<>();
    private static final List<Pair<String, PathConstraints>> paths = new ArrayList<>(Arrays.asList(
            new Pair<>("RED_NODE_ONE_GOAL_ONE", new PathConstraints(2.5, 2.0)),  // to and from top and bottom two nodes and top and bottom two goals
            new Pair<>("RED_NODE_ONE_GOAL_TWO", new PathConstraints(3.0, 2.0)),
            new Pair<>("RED_NODE_TWO_GOAL_ONE", new PathConstraints(3.0, 2.0)),
            new Pair<>("RED_NODE_TWO_GOAL_TWO", new PathConstraints(3.0, 2.0)),
            new Pair<>("RED_NODE_EIGHT_GOAL_THREE", new PathConstraints(2.5, 2.0)),
            new Pair<>("RED_NODE_EIGHT_GOAL_FOUR", new PathConstraints(2.5, 2.0)),
            new Pair<>("RED_NODE_NINE_GOAL_THREE", new PathConstraints(2.5, 2.0)),
            new Pair<>("RED_NODE_NINE_GOAL_FOUR", new PathConstraints(2.5, 2.0)),

            new Pair<>("RED_GOAL_ONE_NODE_ONE", new PathConstraints(2.5, 2.0)),
            new Pair<>("RED_GOAL_ONE_NODE_TWO", new PathConstraints(2.5, 2.0)),
            new Pair<>("RED_GOAL_TWO_NODE_ONE", new PathConstraints(3.0, 2.0)),
            new Pair<>("RED_GOAL_TWO_NODE_TWO", new PathConstraints(3.0, 2.0)),
            new Pair<>("RED_GOAL_THREE_NODE_EIGHT", new PathConstraints(3.0, 2.0)),
            new Pair<>("RED_GOAL_THREE_NODE_NINE", new PathConstraints(3.0, 2.0)),
            new Pair<>("RED_GOAL_FOUR_NODE_EIGHT", new PathConstraints(3.0, 2.0)),
            
            new Pair<>("RED_BUMP_GOAL_ONE", new PathConstraints(2.5, 2.0)),
            new Pair<>("RED_GOAL_ONE_CHARGE_TWO", new PathConstraints(2.5, 2.0)),
            new Pair<>("RED_CHARGE_TWO_GOAL_TWO", new PathConstraints(3.0, 3.0)),
            new Pair<>("RED_GOAL_TWO_CHARGE_THREE", new PathConstraints(3.0, 3.0)),
            new Pair<>("RED_CHARGE_THREE_GOAL_THREE", new PathConstraints(3.0, 3.0)),
            new Pair<>("RED_GOAL_THREE_CHARGE_FOUR", new PathConstraints(3.0, 3.0)),

            
            new Pair<>("BLUE_NODE_ONE_GOAL_ONE", new PathConstraints(2.5, 2.0)),  // to and from top and bottom two nodes and top and bottom two goals
            new Pair<>("BLUE_NODE_ONE_GOAL_TWO", new PathConstraints(3.0, 2.0)),
            new Pair<>("BLUE_NODE_TWO_GOAL_ONE", new PathConstraints(3.0, 2.0)),
            new Pair<>("BLUE_NODE_TWO_GOAL_TWO", new PathConstraints(3.0, 2.0)),
            new Pair<>("BLUE_NODE_EIGHT_GOAL_THREE", new PathConstraints(2.5, 2.0)),
            new Pair<>("BLUE_NODE_EIGHT_GOAL_FOUR", new PathConstraints(2.5, 2.0)),
            new Pair<>("BLUE_NODE_NINE_GOAL_THREE", new PathConstraints(2.5, 2.0)),
            new Pair<>("BLUE_NODE_NINE_GOAL_FOUR", new PathConstraints(2.5, 2.0)),

            new Pair<>("BLUE_GOAL_ONE_NODE_ONE", new PathConstraints(3.0, 2.0)),
            new Pair<>("BLUE_GOAL_ONE_NODE_TWO", new PathConstraints(3.0, 2.0)),
            new Pair<>("BLUE_GOAL_TWO_NODE_ONE", new PathConstraints(3.0, 2.0)),
            new Pair<>("BLUE_GOAL_TWO_NODE_TWO", new PathConstraints(3.0, 2.0)),
            new Pair<>("BLUE_GOAL_THREE_NODE_EIGHT", new PathConstraints(3.0, 2.0)),
            new Pair<>("BLUE_GOAL_THREE_NODE_NINE", new PathConstraints(3.0, 2.0)),
            new Pair<>("BLUE_GOAL_FOUR_NODE_EIGHT", new PathConstraints(3.0, 2.0)),
            
            new Pair<>("BLUE_BUMP_GOAL_ONE", new PathConstraints(3.0, 2.0)),
            new Pair<>("BLUE_GOAL_ONE_CHARGE_TWO", new PathConstraints(3.0, 2.0)),
            new Pair<>("BLUE_CHARGE_TWO_GOAL_TWO", new PathConstraints(3.0, 2.0)),
            new Pair<>("BLUE_GOAL_TWO_CHARGE_THREE", new PathConstraints(3.0, 2.0)),
            new Pair<>("BLUE_CHARGE_THREE_GOAL_THREE", new PathConstraints(3.0, 2.0)),
            new Pair<>("BLUE_GOAL_THREE_CHARGE_FOUR", new PathConstraints(3.0, 2.0)),

            new Pair<>("BLUE_NODE_TWO_SHOOT_GOAL_TWO", new PathConstraints(3.0, 3.0)),
            new Pair<>("RED_NODE_TWO_SHOOT_GOAL_TWO", new PathConstraints(3.0, 3.0)),   
            new Pair<>("BLUE_NODE_EIGHT_SHOOT_GOAL_THREE", new PathConstraints(3.0, 3.0)),
            new Pair<>("RED_NODE_EIGHT_SHOOT_GOAL_THREE", new PathConstraints(3.0, 3.0)),
            new Pair<>("BLUE_NOBUMP_GOAL_THREE_CHARGE_FOUR", new PathConstraints(3.0, 3.0)),
            new Pair<>("RED_NOBUMP_GOAL_THREE_CHARGE_FOUR", new PathConstraints(3.0, 3.0)),
            
            new Pair<>("RED_NODE_ONE_GOAL_ONE_SHOOT", new PathConstraints(3.5, 3.0)),
            new Pair<>("RED_NODE_TWO_GOAL_TWO_LINE", new PathConstraints(3.5, 3.0))
    ));

    public Trajectories(PathConstraints constraints) {
        for (var pair : paths) {
            trajectories.put(pair.getFirst(), PathPlanner.loadPath(pair.getFirst(), pair.getSecond()));
        }
    }

    public PathPlannerTrajectory getTrajectory(String name) {
        return trajectories.get(name);
    }
}
