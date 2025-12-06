package frc.robot;

public class Constants {
  public static final double maxSpeed = 0.31;
  public static final double slowSpeed = 0.15;
  public static final double idleSpeed = -0.08;

  public static final class ModuleProperties {
    /**
     * <> direct quote from rev robotis:
     *
     * The MAXSwerve module can be configured with one of three pinion gears:
     * 12T, 13T, or 14T. This changes the drive speed of the module
     * (a pinion gear with more teeth will result in a robot that drives faster).
     */
    public static final int kDrivingMotorPinionTeeth = 13;

    // <> required for various calculations
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    // <> quote from revrobotics:
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22)
        / (ModuleProperties.kDrivingMotorPinionTeeth * 15);
    public static final double kDrivingEncoderPositionFactor = 1 / kDrivingMotorReduction;
    public static final double kDrivingEncoderVelocityFactor = 1 / kDrivingMotorReduction;

    public static final double kTurningEncoderPositionFactor = 1;
    public static final double kTurningEncoderVelocityFactor = 1;
  }
}
