#include <frc/Joystick.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>
#include <ctre/phoenix/sensors/CanCoder.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <ctre/Phoenix.h>
#include <frc/XboxController.h>
#include <frc/PWMVictorSPX.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/ADXRS450_Gyro.h>
#include <rev/ColorSensorV3.h>
#include <frc/Encoder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/ColorMatch.h>
#include <cameraserver/CameraServer.h>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Encoder.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <cmath>

////////////////////////////// 2020 ROBO BEAVERS OFFICAL CODE FOR GARY THE ROBOT ////////////////////////


class Robot : public frc::TimedRobot {
  public:
Robot() {
    DriveFront.SetExpiration(0.1);
    DriveBack.SetExpiration(0.1);
    timer.Start();
  }

void RobotInit() override {
static constexpr auto kSamplePeriod = 0.0005_s;
static constexpr double kCalibrationSampleTime = 1.0;
static constexpr double kDegreePerSecondPerLSB = 0.0125;
  timesYellow = 0;
  timesRed = 0;
  timesGreen = 0;
  timesBlue = 0;
  Gyro.Calibrate();
  frc::CameraServer::GetInstance()->StartAutomaticCapture();
  frc::CameraServer::GetInstance()->StartAutomaticCapture();
  shootleft.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0, 50);
  shootright.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0, 50);
  chooser.SetDefaultOption("Center Start", &CENTER);
  chooser.AddOption("Left Start", &LEFT);
  chooser.AddOption("Right Start", &RIGHT);
  frc::SmartDashboard::PutData("Autonomous Selector", &chooser);
  frc::SmartDashboard::PutNumber("Selector Value", *chooser.GetSelected());
  frc::SmartDashboard::PutNumber("Gyro Value", Gyro.GetAngle());

}

void DisabledInit() override {
  timesYellow = 0;
}
 
void AutonomousInit() override {
    timer.Reset();
    timer.Start();
    //Gyro.Calibrate();
    
    

  }


void AutonomousPeriodic() override {

////////////////////////// ~~AUTO CODE START~~ /////////////////////
double BreakOne = 1.0;
double BreakTwo = 3.0;
double BreakThree =7.0;
double BreakFour = 10.0;
double BreakFive = 12.0;
double BreakSix = 14.5;
double End = 15.0;
frc::SmartDashboard::PutNumber("Timer Value", timer.Get());
frc::SmartDashboard::PutNumber("Gyro Value", Gyro.GetAngle());



if(chooser.GetSelected()== &CENTER){
  if(timer.Get()<BreakTwo&&timer.Get()>BreakOne){
    DownPickup.Set(frc::DoubleSolenoid::kForward);
    DownShooter.Set(frc::DoubleSolenoid::kReverse);
    shootleft.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, 750);
    shootright.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, 750);
  };

  if(timer.Get()<BreakFour&&timer.Get()>BreakTwo){
    shootleft.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, 750);
    shootright.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, 750);
    Feeder.TankDrive(0.5,0.5);
  };

  if(timer.Get()<BreakFive&&timer.Get()>BreakFour){
    DriveFront.TankDrive(-0.5,-0.5);
    DriveBack.TankDrive(-0.5,-0.5);
  };
  
};

};
   
  ////////////////////////// ~~AUTO CODE END~~ ///////////////////
  

 public:
  void TeleopPeriodic() {  

/////////////////////////// ~~DRIVE TELEOP CODE~~ ////////////////////////////

frc::GenericHID::JoystickHand right = frc::GenericHID::JoystickHand::kRightHand;
frc::GenericHID::JoystickHand left = frc::GenericHID::JoystickHand::kLeftHand;

DriveFront.TankDrive(0.98*std::pow(stick.GetY(left),3.0),std::pow(stick.GetY(right),3.0));
DriveBack.TankDrive(0.98*std::pow(stick.GetY(left),3.0),std::pow(stick.GetY(right),3.0));

//Gear box shift between slow and fast mode
if (stick.GetBumperPressed(left)) {
      Gearbox.Set(frc::DoubleSolenoid::kForward);
    }
else if(stick.GetBumperPressed(right)) {
      Gearbox.Set(frc::DoubleSolenoid::kReverse);
    };


/////////////////////// ~~SECONDARY MECHANISMS CODE~~ ///////////////////////////

if (second.GetRawButton(2)) {
      DownPickup.Set(frc::DoubleSolenoid::kForward);
      DownShooter.Set(frc::DoubleSolenoid::kReverse);
}
else if(second.GetRawButton(1)){
   DownPickup.Set(frc::DoubleSolenoid::kReverse);
    DownShooter.Set(frc::DoubleSolenoid::kForward);
}
else {
      DownPickup.Set(frc::DoubleSolenoid::kOff);
      DownShooter.Set(frc::DoubleSolenoid::kOff);
    };



 if(second.GetRawButton(9)){
  winch.Set(0.5);
}

else{
  winch.Set(0);
};



if(second.GetRawButton(5)){
  climb.Set(-0.35);
}
else if(second.GetRawButton(6)){
  climb.Set(0.3);
}
else{
  climb.Set(0.0)
;}


if (second.GetRawButton(11)){
  
  shootleft.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, 750);
  shootright.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, 750);
}

else{
  shootleft.Set(0);
  shootright.Set(0);
};

Feeder.TankDrive((second.GetRawButton(7)),(second.GetRawButton(7)));


if(second.GetRawButton(3)){
pickup.Set(0.5); 
}
else if(second.GetRawButton(4)){
  pickup.Set(-0.5);
}
else {
  pickup.Set(0);
};

//////////////////////// ~~GYRO TELEOP CODE~~ //////////////////////////////

if(stick.GetYButton()){
  Gyro.Calibrate();
};

/* if(stick.GetAButtonPressed()){
   float target = Gyro.GetAngle()+173.0;
   
   while(Gyro.GetAngle()< target){
     DriveFront.TankDrive(0.5,-0.5);
     DriveBack.TankDrive(0.5,-0.5);
   }
}; */

/////////////////////// ~~COLOR SENSOR CODE~~ ////////////////////////////////////
frc::Color detectedColor = colorsensor.GetColor();
rev::ColorMatch colormatch;


static constexpr frc::Color kBlueTarget =frc::Color(0.143,0.427,0.429);
static constexpr frc::Color kGreenTarget =frc::Color(0.197,0.561,0.240);
static constexpr frc::Color kRedTarget =frc::Color(0.561,0.232,0.114);
static constexpr frc::Color kYellowTarget =frc::Color(0.361,0.524,0.113);

colormatch.AddColorMatch(kBlueTarget);
colormatch.AddColorMatch(kGreenTarget);
colormatch.AddColorMatch(kRedTarget);
colormatch.AddColorMatch(kYellowTarget);


std::string colorstring;
double confidence=0.0;

frc::Color matchedColor = colormatch.MatchClosestColor(detectedColor,confidence);

if(matchedColor == kBlueTarget){
  isAddedYellow = false;
  isAddedGreen = false;
  isAddedRed = false;

 if(isAddedBlue == false){
    isAddedBlue = true;
    timesBlue++;
  }
  colorstring = "Blue";
}
else if(matchedColor == kGreenTarget){
  isAddedYellow = false;
  isAddedRed = false;
  isAddedBlue = false;

   if(isAddedGreen == false){
    isAddedGreen = true;
    timesGreen++;
  }
  colorstring = "Green";
}
else if(matchedColor == kRedTarget){
  isAddedYellow = false;
  isAddedGreen = false;
  isAddedBlue = false;

   if(isAddedRed == false){
    isAddedRed = true;
    timesRed++;
  }
  colorstring = "Red";
}
else if(matchedColor == kYellowTarget){
    isAddedGreen = false;
    isAddedRed = false;
    isAddedBlue = false;
  if(isAddedYellow == false){
    isAddedYellow = true;
    timesYellow++;
  }
  colorstring = "Yellow";
}
else{
  colorstring = "Unknown";
  isAddedYellow = false;
  isAddedGreen = false;
  isAddedRed = false;
  isAddedBlue = false;
}




frc::SmartDashboard::PutNumber("Confidence", confidence);
frc::SmartDashboard::PutString("Detected Color",colorstring);
frc::SmartDashboard::PutNumber("Times for Yellow",timesYellow);
frc::SmartDashboard::PutBoolean("isAddedYellow",isAddedYellow);
frc::SmartDashboard::PutNumber("Times for Blue",timesBlue);
frc::SmartDashboard::PutBoolean("isAddedBlue",isAddedBlue);
frc::SmartDashboard::PutNumber("Times for Red",timesRed);
frc::SmartDashboard::PutBoolean("isAddedRed",isAddedRed);
frc::SmartDashboard::PutNumber("Times for Green",timesGreen);
frc::SmartDashboard::PutBoolean("isAddedGreen",isAddedGreen);


  }
/////////////////////// ~~END TELEOP~~ /////////////////////////////////////////  


///////////////////// ~~VARIABLE DEFINITIONS~~ ////////////////////////////////
  
  /* Auton Variables */
  frc::LiveWindow& m_lw = *frc::LiveWindow::GetInstance();
  frc::Timer timer;  
  char auton;
  frc::SendableChooser<int*> chooser;
  int CENTER = 0;
  int LEFT = 1;
  int RIGHT = 2;
  

  
  /* Drive Variables */
  frc::DifferentialDrive DriveFront{leftfront, rightfront};
  frc::DifferentialDrive DriveBack{leftback,rightback};
  can::WPI_VictorSPX leftfront{0};
  can::WPI_VictorSPX rightfront{2};
  can::WPI_VictorSPX leftback{1};
  can::WPI_VictorSPX rightback{3};
  frc::XboxController stick{0};
  

  /* Secondary things variables */
  sensors::CANCoder encoder{0};
  sensors::CANCoder encodertwo{1};
  can::WPI_TalonSRX shootleft{0};
  can::WPI_TalonSRX shootright{1};
  can::WPI_VictorSPX winch{6};
  can::WPI_TalonSRX climb{2};
  can::WPI_VictorSPX pickup{4};
  frc::DifferentialDrive Shoot{shootleft, shootright};
  can::WPI_VictorSPX feedleft{5};
  can::WPI_VictorSPX feedright{7};
  frc::DifferentialDrive Feeder{feedleft,feedright};
  frc::Joystick second{1};
  frc::ADXRS450_Gyro Gyro{};
  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
  rev::ColorSensorV3 colorsensor{i2cPort};
  frc::DoubleSolenoid Gearbox{6,7};
  frc::DoubleSolenoid DownPickup{2,3};
  frc::DoubleSolenoid DownShooter{0,1};
 

  int timesYellow;
  int timesRed;
  int timesBlue;
  int timesGreen;

  bool isAddedYellow;
  bool isAddedRed;
  bool isAddedBlue;
  bool isAddedGreen;

 
};


#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
