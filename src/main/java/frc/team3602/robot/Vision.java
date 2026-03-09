package frc.team3602.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3602.robot.LimelightHelpers;
import frc.team3602.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.team3602.robot.LimelightHelpers.RawFiducial;
import frc.team3602.robot.subsystems.TurretSubsystem;
import frc.team3602.robot.subsystems.CommandSwerveDrivetrain;

public class Vision {
    public TurretSubsystem turret;
    public CommandSwerveDrivetrain drivetrain;

    public Vision() {
        int[] validTagID = { 21, 26, 18, 5, 10, 2 };
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight-turret", validTagID);
    }

    public double getDistToCamera(int id) {
        var tags = LimelightHelpers.getRawFiducials("limelight-primary");
        return 0;
    }

    public double getTX() {
        return LimelightHelpers.getTX("limelight-primary");
    }

    public double getTurretTX() {
        return LimelightHelpers.getTX("limelight-turret");
    }

    public boolean getHasTarget() {
        return LimelightHelpers.getTV("limelight-primary");
    }

    public boolean getTurretHasTarget() {
        return LimelightHelpers.getTV("limelight-turret");
    }

    public double getTY() {
        return LimelightHelpers.getTY("limelight-turret");
    }

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-turret");
    NetworkTableEntry ty = table.getEntry("ty");

    double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 25.0;

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 20.0;

    // distance from the target to the floor
    double goalHeightInches = 44.25;

    double angleToGoalDegrees = limelightMountAngleDegrees + getTY();
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    // calculated Distance
    double angle = Math.toRadians(this.getTY() + this.getTurretIMUPitch());
    double distance;

    public double getDist() {
        return distance = (44.21875 - 15.625) / Math.tan(angle);
    }

    public double getDistanceToTarget() {

    // Robot pose from odometry
    Pose2d robotPose = drivetrain.getState().Pose;

    // Target field location (meters)
    Translation2d targetPosition = turret.getTargetPose(); // TODO set correct field coordinates

    // Robot position
    Translation2d robotPosition = robotPose.getTranslation();

    // Distance between the two
    double distance = robotPosition.getDistance(targetPosition);

    double distanceFeet = Units.metersToFeet(distance);

    return distanceFeet;
}

    public double getPoseY() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-primary").pose.getY();
    }

    public Pose2d getPose() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-primary").pose;
    }

    // Retrieve IMU Pitch. - for Up instead of Down
    public double getTurretIMUPitch() {
        return -LimelightHelpers.getIMUData("limelight-turret").Pitch;
    }

}
