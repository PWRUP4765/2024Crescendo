package frc.robot.util.identity;

import frc.robot.util.FileUtil;

import java.lang.reflect.Method;

public class Identity {
    String fileName = "hardwareID";
    final String identity;

    public Identity() {
        this.identity = FileUtil.getfileString(fileName);
    }

    public String getIdentity() {
        return this.identity;
    }
}
