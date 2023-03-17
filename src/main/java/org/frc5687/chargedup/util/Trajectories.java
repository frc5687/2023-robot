package org.frc5687.chargedup.util;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import java.util.HashMap;


/**
 * This class contains all the trajectories we generate from path-planner;
 */
public class Trajectories {
    HashMap<String, PathPlannerTrajectory> trajectories = new HashMap<>();
    String path_names[] = new String[] {
            "NODE_ONE_GOAL_ONE",
            "NODE_ONE_GOAL_TWO",
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
