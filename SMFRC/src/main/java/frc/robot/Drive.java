package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

public class Drive {
  
  public static CANSparkMax left1;
  public static WPI_TalonSRX left2;
  public static WPI_TalonSRX left3;

  public static CANSparkMax right1;
  public static WPI_TalonSRX right2;
  public static WPI_TalonSRX right3;

  public static Joystick JS;

  public CANEncoder leftE;
  public CANEncoder rightE;

  public double leftZero;
  public double rightZero;
  public double leftCPM; //CPM for click per meter;
  public double rightCPM;
  public double leftTar;
  public double rightTar;

  public Double leftInt;
  public Double rightInt;
  public Double leftPrevError;
  public Double rightPrevError;
  public Double prevTime;

  public Timer timer;

  public Drive(){
    left1 = new CANSparkMax(12, MotorType.kBrushless);
    setSpark(left1);
    left1.setInverted(false);
    left2 = new WPI_TalonSRX(3);
    left2.setInverted(false);
    left3 = new WPI_TalonSRX(4);
    left3.setInverted(false);

    right1 = new CANSparkMax(11, MotorType.kBrushless);
    setSpark(right1);
    right1.setInverted(true);
    right2 = new WPI_TalonSRX(2);
    right2.setInverted(true);
    right3 = new WPI_TalonSRX(1);
    right3.setInverted(true);
    JS = new Joystick(0);

    rightE = right1.getEncoder();
    leftE = left1.getEncoder();

    leftCPM = 18.38; //prev: 17.28
    rightCPM = 17.85; //prev: 19.6

    prevTime=0.0;
    leftPrevError = 0.0;
    rightPrevError = 0.0;
  }

  public void RC(){
    double X = -0.5*JS.getRawAxis(1);
    double Y = 0.5*JS.getRawAxis(0);

    double leftSpeed = 0;
    double rightSpeed = 0;
    
    leftSpeed = X + Y;
    rightSpeed = X - Y;

    move(leftSpeed, rightSpeed);
  }

  public void move(Double leftSpeed, Double rightSpeed ){

    //System.out.print(JS.getRawAxis(1));
    //System.out.print(" ");
    //System.out.println(JS.getRawAxis(0));
    //System.out.print(leftSpeed);
    // System.out.print(" ");
    // System.out.println(rightSpeed);
    left1.set(leftSpeed);
    left2.set(leftSpeed);
    left3.set(leftSpeed);

    right1.set(rightSpeed);
    right2.set(rightSpeed);
    right3.set(rightSpeed);
  }

  public void setZero(){
    leftZero = leftE.getPosition();
    rightZero = rightE.getPosition();
  }

  public void setPID(double leftMeter, double rightMeter){
    setZero();
    leftTar = leftZero + leftMeter*leftCPM;
    rightTar = rightZero + rightMeter*rightCPM;
    leftInt=0.0;
    rightInt=0.0;
    timer = new Timer();
    timer.start();
  }

  public void runPID(){
    double leftError = leftTar - leftE.getPosition(); // Error = Target - Actual
    double rightError = rightTar - rightE.getPosition();
    // System.out.print(leftError);
    // System.out.print(" ");
    // System.out.println(rightError);
    double t = timer.get()-prevTime;
    prevTime = timer.get();

    double leftSpeed = computePID(leftError, leftInt, leftPrevError, t, 1.0, 0.0, 0.024); //prev: 1, 0, 0.28
    double rightSpeed = computePID(rightError, rightInt, rightPrevError, t, 1.0, 0.0, 0.014); // prev: 1, 0, 0.016

    leftPrevError = leftError;
    rightPrevError = rightError;

    move(leftSpeed, rightSpeed);
  }

  public double computePID(double error, Double integral, Double prevError, Double t, Double P, Double I, Double D){
    //integral += (error*.02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
    // I doubt intergral is going to work since we are only working on one side.
    double derivative = (error - prevError) / t;
    System.out.println(derivative);
    System.out.println(t);
    // double derivative =0;
    double outPut = P*error + I*integral + D*derivative;
    return outPut;
  }

  private void setSpark(final CANSparkMax spark) {
    spark.restoreFactoryDefaults();
    spark.setOpenLoopRampRate(0.4);
    spark.setClosedLoopRampRate(0.4);
    // spark.enableVoltageCompensation(12.0);
    spark.setSmartCurrentLimit(40);
  }

}