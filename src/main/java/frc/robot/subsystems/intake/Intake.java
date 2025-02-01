package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private static Intake instance;
  private final TalonFX motor;
  private MotionMagicVelocityTorqueCurrentFOC controlMode;
  private double velocity;
  private SysIdRoutine sysID;

  public double getVelocity() {
    return velocity;
  }

  public void setVelocity(double velocity) {
    this.velocity = velocity;
  }

  private Intake() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(1000)
            .withMotionMagicCruiseVelocity(100);
    config.CurrentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimit(80);
    config.Slot0 =
        new Slot0Configs()
            .withKP(Constants.Intake.kP)
            .withKI(Constants.Intake.kI)
            .withKD(Constants.Intake.kD)
            .withKS(Constants.Intake.kS)
            .withKV(Constants.Intake.kV);
    motor = new TalonFX(Constants.Intake.motorId);
    motor.getConfigurator().apply(config);
    controlMode = new MotionMagicVelocityTorqueCurrentFOC(0).withEnableFOC(true);
    motor.setControl(controlMode);

    this.sysID =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Intake/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }
    return instance;
  }

  public void intake() {
    velocity = Constants.Intake.intakeVelocity;
  }

  public void runCharacterization(double voltage) {
    motor.setVoltage(voltage);
  }

  public void outtake() {
    velocity = Constants.Intake.outtakeVelocity;
  }

  public void stop() {
    velocity = 0.0;
  }

  @Override
  public void periodic() {
    controlMode.Velocity = velocity;
    motor.setControl(controlMode);
    Logger.recordOutput("RealOutput/Intake/IntakeVelocity", motor.getVelocity().getValueAsDouble());
    Logger.recordOutput("RealOutput/Intake/IntakePosition", motor.getPosition().getValueAsDouble());
    Logger.recordOutput(
        "RealOutput/Intake/IntakeVoltage", motor.getMotorVoltage().getValueAsDouble());
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysID.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysID.dynamic(direction));
  }
}
