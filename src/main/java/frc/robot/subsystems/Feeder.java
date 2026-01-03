
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Feeder Roller subsystem 1:1 is the  feed to shooter
//
// 1 Kraken x60 feeder roller the actual is 2:1
// 1 kraken x60 one to one

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.generated.TunerConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.SmartMechanism;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Feeder extends SubsystemBase {
  public class feederConstants {
    public static final int MOTOR_ID = 11;
    // public static final Voltage IDLE = Volts.of(0);
    // public static final Voltage SHOOTER_FEED = Volts.of(4);
    // public static final Voltage AMP_FEED = Volts.of(-4);
    // public static final Voltage SLOW_FEED = Volts.of(3);
    // public static final Voltage DE_AMP = Volts.of(2);

    public static final Current STATOR_CURRENT_LIMIT = Amps.of(60);

    public static final AngularVelocity SHOOTER_VELOCITY = RPM.of(6000);
    public static final AngularVelocity AMP_VELOCITY = RPM.of(-5400);
    public static final AngularVelocity SLOW_VELOCITY = RPM.of(4000);
    public static final AngularVelocity DE_AMP_VELOCITY = RPM.of(2400);
  }

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(30, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
      .withSimClosedLoopController(30, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
      // Feedforward Constants
      .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
      // Telemetry name and verbosity level
      .withTelemetry("FeederMotor", TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft.
      // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which
      // corresponds to the gearbox attached to your motor.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(2,1)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(feederConstants.STATOR_CURRENT_LIMIT);

  // Vendor motor controller object
  private TalonFX rollerMotor = new TalonFX(feederConstants.MOTOR_ID);

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController rollerSMC = new TalonFXWrapper(rollerMotor, DCMotor.getFalcon500(1), smcConfig);

  private FlyWheelConfig feederConfig = new FlyWheelConfig(rollerSMC)
      .withDiameter(Inches.of(2))
      .withMass(Pounds.of(1))
      .withTelemetry("Feeder", TelemetryVerbosity.HIGH);


  // Arm Mechanism
  private FlyWheel feeder = new FlyWheel(feederConfig);


  public Command sysId() {
    return feeder.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }
  public AngularVelocity getVelocity() {
    return feeder.getSpeed();
  }
  public Command setVelocity(AngularVelocity speed) {return feeder.setSpeed(speed);}

  public Command shooterFeed() {
    return setVelocity(feederConstants.SHOOTER_VELOCITY);
  }

  public Command slowFeed() {
    return setVelocity(feederConstants.SLOW_VELOCITY);
  }

  public Command ampFeed() {
    return setVelocity(feederConstants.AMP_VELOCITY);
  }

  public Command deAmp() {
    return setVelocity(feederConstants.DE_AMP_VELOCITY);
  }

  public Command idle() {
    return setVelocity(RPM.of(0));
  }
  public Command setDutyCycle(double dutyCycle) {
    return feeder.set(dutyCycle);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    feeder.updateTelemetry();

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    feeder.simIterate();
  }

}