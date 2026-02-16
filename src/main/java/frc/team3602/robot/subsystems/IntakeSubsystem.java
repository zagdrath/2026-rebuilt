package frc.team3602.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3602.robot.Constants.IntakeConstants;
import frc.team3602.robot.Constants.ShooterConstants;

public class IntakeSubsystem extends SubsystemBase{
    //variables
    private static TalonFX intakeMotor;
    private static TalonFX intakePivot;
    
    //constructor
    public IntakeSubsystem(){
        intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorID, "rio");
        intakePivot = new TalonFX(IntakeConstants.kIntakePivotID, "rio");
    }

    public Double getPosition() {
        return (intakePivot.getRotorPosition().getValueAsDouble() / 9); // every revolution is 12 degrees because it is                                                           // a 30:1 gear ratio 10:1 from gear, 3:1 from gear box
    }
    public Boolean isDown() {
        return (getPosition() < .25);
    }

    //EAT
    public Command setIntakeSpeed() {
        return runOnce(() -> 
            intakeMotor.set(IntakeConstants.kIntakeMotorSpeed));
    
    }

    //STOP
    public Command stopIntake() {
        return runOnce(() ->
        intakeMotor.set(0));
    }

    //VOMIT
    public Command reverseIntake() {
        return runOnce(() ->
        intakeMotor.set(-IntakeConstants.kIntakeMotorSpeed));
    }

    public Command runPivot(double power) {
        return runOnce(() ->
        intakePivot.setVoltage(power));
    }

    public Command stopPivot() {
        return runOnce(() -> 
        intakePivot.setVoltage(0));
    }

     public Command dropIntake() {
        return Commands.sequence(
         this.runPivot(.4).until(() -> this.isDown()),
         this.stopPivot()
        );
     }

     public Command raiseIntake(){
        return Commands.sequence(
            this.runPivot(-0.4).until(() -> !this.isDown()),
            this.stopPivot()
        );
     }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Position", intakePivot.getRotorPosition().getValueAsDouble() / 9);
        // SmartDashboard.putBoolean("Intake Boolean", isDown());
    }
}

