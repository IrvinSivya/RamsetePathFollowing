// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;


import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class RamsetePathFollower {

    RamseteController ramseteController;
    Trajectory currentTrajectory;

    private double timeSinceStart;
    private double previousTime;
    private double currentTime;


    /**
     *initializes the trajectory, the b gain, and the zeta gain 
     * @param trajectory
     * @param b
     * @param zeta
     */

    public RamsetePathFollower(Trajectory trajectory, double b, double zeta) {
        ramseteController = new RamseteController(b, zeta);
        setTrajectory(trajectory);
        resetPath();
    }
  
    /**
     * resets the time variables
     * must be called before the getGoalSpeeds method
     */
    public void resetPath() {
        timeSinceStart = 0;
        previousTime = Timer.getFPGATimestamp();
        currentTime = Timer.getFPGATimestamp();
    }

    /**
     * resetPath method should be called before getGoalSpeeds in order for timeSinceStart to be accumulated 
     * calculates velocity needed to reach setpoint
     * @param currentPose
     * @return adjusted speeds
     */
    public ChassisSpeeds getGoalSpeeds(Pose2d currentPose) {
        currentTime = Timer.getFPGATimestamp();
        double deltaTime = currentTime - previousTime;
        previousTime = currentTime;
        timeSinceStart += deltaTime;

        Trajectory.State goal = currentTrajectory.sample(timeSinceStart);
        ChassisSpeeds adjustedSpeeds = ramseteController.calculate(currentPose, goal);
        return adjustedSpeeds;
    }
    
  /**
   * Gets JSON File from Pathweaver and assigns it to trajectory
   * @param filename Name of JSON File acquired from Pathweaver
   * @return currentTrajectory
   */
    public static Trajectory getTrajectory(String filename) { 
        Trajectory trajectory = null;
        try{
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch(IOException exception) {
            DriverStation.reportError("Unable to open trajectory" + filename, exception.getStackTrace());
        }
        return trajectory;
    }

    /**
     * returns currentTrajectory
     * @return currentTrajectory
     */
    public Trajectory getCurrentTrajectory() {
        return currentTrajectory;
    }

    /**
     * sets currentTrajectory to newTrajectory
     * @param newTrajectory
     */
    public void setTrajectory(Trajectory newTrajectory) {
        currentTrajectory = newTrajectory;
    }

    /**
     * sets b and zeta gains
     * @param b
     * @param zeta
     */
    public void setBZeta(double b, double zeta) {
        ramseteController = new RamseteController(b, zeta);
    }
}
