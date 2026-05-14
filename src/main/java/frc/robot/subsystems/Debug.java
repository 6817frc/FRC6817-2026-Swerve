package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DebugConstants;

public class Debug extends SubsystemBase {
    private boolean useOutreachBindings = false;

    public Debug() {
        SmartDashboard.putBoolean("Outreach Bindings", useOutreachBindings);
    }

    public DebugConstants.ControllerBindingsUpdate checkControllerBindingsUpdate() {
        if (SmartDashboard.getBoolean("Outreach Bindings", useOutreachBindings) != useOutreachBindings) {
            useOutreachBindings = !useOutreachBindings;
            if (useOutreachBindings) {
                return DebugConstants.ControllerBindingsUpdate.OUTREACH;
            } else {
                return DebugConstants.ControllerBindingsUpdate.MAIN;
            }
        }
        return DebugConstants.ControllerBindingsUpdate.NONE;
    }

    @Override
    public void periodic() {
    }
}
