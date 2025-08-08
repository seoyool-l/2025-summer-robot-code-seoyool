package org.blackknights.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.blackknights.constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    SparkFlex topMotor = new SparkFlex(ShooterConstants.TOP_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    SparkFlex bottomMotor = new SparkFlex(ShooterConstants.BOTTOM_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    private SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(ShooterConstants.KS, ShooterConstants.KV, ShooterConstants.KA);
    public ShooterSubsystem() {
        topMotor.setInverted(true);
        bottomMotor.setInverted(false);
    }
    public void setBothMotorVoltage(double volts) {
        topMotor.setVoltage(volts);
        bottomMotor.setVoltage(volts);
    }
    public void setBothMotorPercent(double percent) {
        topMotor.set(percent);
        bottomMotor.set(percent);
    }
    public void setTopMotorVoltage(double volts) {
        topMotor.setVoltage(volts);
    }
    public void setBottomMotorVoltage(double volts) {
        bottomMotor.setVoltage(volts);
    }
    public void setTopMotorPercent(double percent) {
        topMotor.set(percent);
    }
    public void setBottomMotorPercent(double percent) {
        bottomMotor.set(percent);
    }
    public void setShooterSpeed(double speed) {
        setBothMotorVoltage(
            shooterFF.calculate(speed)
        );
    }
}
