package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Landmark;

public class PlotLandmarks extends SubsystemBase {

    private final Field2d m_field = new Field2d();

    public PlotLandmarks() {
        SmartDashboard.putData("Field", m_field);

        // Plot all landmarks as named objects on the Field2d widget
        m_field.getObject("Hub").setPose(Landmark.HUB.get());
        m_field.getObject("Tower").setPose(Landmark.TOWER.get());
        m_field.getObject("Outpost").setPose(Landmark.OUTPOST.get());
        m_field.getObject("Depot").setPose(Landmark.DEPOT.get());
        m_field.getObject("RightBump").setPose(Landmark.RIGHT_BUMP.get());
        m_field.getObject("LeftBump").setPose(Landmark.LEFT_BUMP.get());
        m_field.getObject("RightTrench").setPose(Landmark.RIGHT_TRENCH.get());
        m_field.getObject("LeftTrench").setPose(Landmark.LEFT_TRENCH.get());
        m_field.getObject("RightStart").setPose(Landmark.RIGHT_START.get());
        m_field.getObject("LeftStart").setPose(Landmark.LEFT_START.get());
        m_field.getObject("MiddleStart").setPose(Landmark.MIDDLE_START.get());
    }
}

