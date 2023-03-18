package org.frc5687.chargedup.util;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import java.util.HashMap;

/** This class contains all the trajectories we generate from path-planner; */
public class Trajectories {
    static HashMap<String, PathPlannerTrajectory> trajectories = new HashMap<>();
    String path_names[] =
            new String[] {
                "NODE_ONE_GOAL_ONE", // to and from top and bottom two nodes and top and bottom two goals
                "NODE_ONE_GOAL_TWO",
                "NODE_TWO_GOAL_ONE",
                "NODE_TWO_GOAL_TWO",
                "NODE_EIGHT_GOAL_THREE",
                "NODE_EIGHT_GOAL_FOUR",
                "NODE_NINE_GOAL_THREE",
                "NODE_NINE_GOAL_FOUR",
                "GOAL_ONE_NODE_ONE",
                "GOAL_ONE_NODE_TWO",
                "GOAL_TWO_NODE_ONE",
                "GOAL_TWO_NODE_TWO",
                "GOAL_THREE_NODE_EIGHT",
                "GOAL_THREE_NODE_NINE",
                "GOAL_FOUR_NODE_EIGHT",
                "GOAL_FOUR_NODE_NINE",
                "BUMP_GOAL_ONE",
                "GOAL_ONE_CHARGE_TWO", // specifically for steal cubes auto
                "CHARGE_TWO_GOAL_TWO",
                "GOAL_TWO_CHARGE_THREE",
                "CHARGE_THREE_GOAL_THREE",
                "GOAL_THREE_CHARGE_FOUR"
            };

    public Trajectories(PathConstraints constraints) {
        for (var name : path_names) {
            trajectories.put(name, PathPlanner.loadPath(name, constraints));
        }
    }

    public PathPlannerTrajectory getTrajectory(String name) {
        return trajectories.get(name);
    }
}
