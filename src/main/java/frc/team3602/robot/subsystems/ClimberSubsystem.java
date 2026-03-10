package frc.team3602.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3602.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    /* Motors */
    private final TalonFX climberMotor = new TalonFX(ClimberConstants.kClimberMotorID);

    /* Controls */
    private final ArmFeedforward climberFeedforward = new ArmFeedforward(0, 0.69, 15.79, 0.07);
    private final PIDController climberController = new PIDController(43.19, 0, 0.19);
    
    private double heightInches = 0.0;

    public ClimberSubsystem() {
        configClimberSubsys();
    }

    public double getHeight() {
       return (climberMotor.getRotorPosition().getValueAsDouble() * (Math.PI * 2.15) /  20.25);
    }

    public Command setHeight(DoubleSupplier heightInches) {
        return runOnce(() ->  this.heightInches = heightInches.getAsDouble());
    }

    public double getEffort() {
        var ffEffort climberFeedforward.calculate(Units.degreestoRadians(climberHeight), 0.0);
        var pidEffort climberController.calculate(getHeight(), climberHeight);

        return ffEffort + pidEffort;
    }

    public Command holdHeight() {
        return run(() -> {
            climberMotor.setVoltage(getEffort());
        });
    }

    public Command stopClimber() {
        return runOnce(() -> climberMotor.setVoltage(0.0));
    }
    
    @Override
    public void periodic() {
        
    }

    private void configClimberSubsys() {
        
    }
}
