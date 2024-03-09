package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;

public class DriverAllianceUtil {
    static Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    public static DriverStation.Alliance getCurrentAlliance() {
        return alliance.get();
    }
}
