package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
    // CAN Buses
    public static final CANBus kRoboRioCANBus = new CANBus("rio");
    public static final CANBus kCANivoreCANBus = new CANBus("main");

    // Talon FX IDs
    public static final int kIntakePivot = 17;
    public static final int kIntakeRollers = 11;
    public static final int kFloor = 10;
    public static final int kFeeder = 15;
    public static final int kShooterLeft = 12;
    public static final int kShooterMiddle = 13;
    public static final int kShooterRight = 16;
    public static final int kHanger = 19;

    // PWM Ports
    public static final int kHoodLeftServo = 0;
    public static final int kHoodRightServo = 1;
    public static final int kNeoPixel = 6;
}
