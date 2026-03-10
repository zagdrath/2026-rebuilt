package frc.team3602.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3602.robot.Constants.IntakeConstants;
import frc.team3602.robot.Constants.ShooterConstants;

public class IntakeSubsystem extends SubsystemBase{
    /* Motors */
    private static TalonFX intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorID, "rio");
    
    public IntakeSubsystem(){
        configIntakeSubsys();
    }

    public Command runIntake() {
        return runOnce(() -> intakeMotor.set(IntakeConstants.kIntakeMotorSpeed));
    }

    public Command reverseIntake() {
        return runOnce(() -> intakeMotor.set(-IntakeConstants.kIntakeMotorSpeed));
    }
    
    public Command stopIntake() {
        return runOnce(() -> intakeMotor.set(0));
    }

    @Override
    public void periodic() {

    }

    private void configIntakeSubsys() {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        // Set motor current limits
        var currentLimitConfigs = talonFXConfigs.CurrentLimits;
        currentLimitConfigs.StatorCurrentLimitEnable = true;
        currentLimitConfigs.SupplyCurrentLimitEnable = true;
        // currentLimitConfigs.StatorCurrentLimit = 60;
        // currentLimitConfigs.SupplyCurrentLimit = 60;

        intakeMotor.getConfigurator().apply(talonFXConfigs);
    }
}
