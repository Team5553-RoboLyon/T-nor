// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::NearShoot()
{
  m_robotContainer.m_planetary.SetSetpoint(NEAR_ANGLE);
  m_goal = NEAR_SPEED_SHOOT * SHOOTER_GOALS_CONVERSION;
  m_count++;
  switch (m_stateNearShoot)
  {
  case StateNearShoot::PreShoot:
    m_robotContainer.m_shooter.SetShooter(NEAR_SPEED_SHOOT); // 0.5
    if (NABS(m_robotContainer.m_shooter.GetShooterVelocity()) > m_goal && m_robotContainer.m_planetary.m_planetaryPid.AtSetpoint())
    {
      m_stateNearShoot = StateNearShoot::Shoot;
    }
    break;
  case StateNearShoot::Shoot:
    m_robotContainer.m_feeder.SetFeeder(CATCH_FEEDER_SPEED);
    m_robotContainer.m_shooter.SetShooter(NEAR_SPEED_SHOOT); // 0.5
    if (!m_robotContainer.m_feeder.GetFeederInfraSensorValue())
    {
      m_count = 0;
      m_stateNearShoot = StateNearShoot::Shooting;
    }
    break;
  case StateNearShoot::Shooting:
    m_robotContainer.m_feeder.SetFeeder(CATCH_FEEDER_SPEED);
    m_robotContainer.m_shooter.SetShooter(NEAR_SPEED_SHOOT);
    std::cout << m_count << std::endl;
    std::cout << m_robotContainer.m_feeder.GetFeederInfraSensorValue() << std::endl;

    if (m_robotContainer.m_feeder.GetFeederInfraSensorValue() and (m_count > SHOOTER_COUNT_READY))
    {
      m_robotContainer.m_feeder.IsNoteLoaded = false;
      m_robotContainer.m_planetary.SetSetpoint(REST_ANGLE);
      m_robotContainer.m_feeder.SetFeeder(STOP_FEEDER_SPEED);
      m_robotContainer.m_shooter.SetShooter(STOP_SHOOTER_SPEED);
      m_stateNearShoot = StateNearShoot::End;
    }
    break;
  case StateNearShoot::End:
    m_robotContainer.m_feeder.IsNoteLoaded = false;
    m_robotContainer.m_planetary.SetSetpoint(REST_ANGLE);
    m_robotContainer.m_feeder.SetFeeder(STOP_FEEDER_SPEED);
    m_robotContainer.m_shooter.SetShooter(STOP_SHOOTER_SPEED);
    break;

  default:
    break;
  }
}

void Robot::TakeNoteSwitch()
{
  m_robotContainer.m_planetary.SetSetpoint(TAKE_ANGLE);
  switch (m_stateTakeNote)
  {
  case StateTakeNote::Catch:
    m_robotContainer.m_feeder.SetFeeder(CATCH_FEEDER_SPEED);
    m_robotContainer.m_intake.SetIntake(INTAKE_SPEED);
    if (!m_robotContainer.m_feeder.GetFeederInfraSensorValue())
    {
      m_robotContainer.m_intake.SetIntake(STOP_INTAKE_SPEED);
      m_robotContainer.m_feeder.SetFeeder(SPIT_FEEDER_SPEED);
      m_stateTakeNote = StateTakeNote::Recul;
    }
    break;
  case StateTakeNote::Recul:
    m_robotContainer.m_feeder.SetFeeder(SPIT_FEEDER_SPEED);
    if (m_robotContainer.m_feeder.GetFeederInfraSensorValue())
    {
      m_robotContainer.m_feeder.SetFeeder(STOP_FEEDER_SPEED);
      m_stateTakeNote = StateTakeNote::Loaded;
    }
    break;
  case StateTakeNote::Loaded:
    m_robotContainer.m_feeder.SetFeeder(STOP_FEEDER_SPEED);
    m_robotContainer.m_feeder.IsNoteLoaded = true;
    m_robotContainer.m_planetary.SetSetpoint(REST_ANGLE);
    m_takeNote = false;
    break;
  case StateTakeNote::End:
    m_robotContainer.m_feeder.SetFeeder(STOP_FEEDER_SPEED);
    m_robotContainer.m_intake.SetIntake(STOP_INTAKE_SPEED);
    m_robotContainer.m_feeder.IsNoteLoaded = false;
    break;
  case StateTakeNote::nule:
    break;
  default:
    break;
  }
}

void Robot::ShootSwitch()
{
  shooter_speed = m_robotContainer.m_shooter.shooterDataTable[m_robotContainer.m_shooter.getNearestElementId(m_robotContainer.m_camera.GetPitch(ID_APRILTAG_MIDDLE, ID_APRILTAG_LEFT))][2];
  planteray_angle = m_robotContainer.m_shooter.shooterDataTable[m_robotContainer.m_shooter.getNearestElementId(m_robotContainer.m_camera.GetPitch(ID_APRILTAG_MIDDLE, ID_APRILTAG_LEFT))][1];

  m_robotContainer.m_planetary.SetSetpoint(planteray_angle);
  m_goal = shooter_speed * SHOOTER_GOALS_CONVERSION;
  m_count++;
  switch (m_stateShootSwitch)
  {
  case StateShootSwitch::Loaded:
    m_robotContainer.m_feeder.SetFeeder(STOP_FEEDER_SPEED);
    if (m_robotContainer.m_feeder.IsNoteLoaded)
    {
      m_stateShootSwitch = StateShootSwitch::PreShoot;
    }
    break;
  case StateShootSwitch::PreShoot:
    m_robotContainer.m_shooter.SetShooter(shooter_speed);
    if (NABS(m_robotContainer.m_shooter.GetShooterVelocity()) > m_goal && m_robotContainer.m_planetary.m_planetaryPid.AtSetpoint())
    {
      m_stateShootSwitch = StateShootSwitch::Shoot;
    }
    break;
  case StateShootSwitch::Shoot:
    m_robotContainer.m_feeder.SetFeeder(CATCH_FEEDER_SPEED);
    m_robotContainer.m_shooter.SetShooter(shooter_speed);
    if (!m_robotContainer.m_feeder.GetFeederInfraSensorValue())
    {
      m_stateShootSwitch = StateShootSwitch::Shooting;
      m_count = 0;
    }
    break;
  case StateShootSwitch::Shooting:
    m_robotContainer.m_feeder.SetFeeder(CATCH_FEEDER_SPEED);
    m_robotContainer.m_shooter.SetShooter(shooter_speed);
    if (m_robotContainer.m_feeder.GetFeederInfraSensorValue() && m_count > SHOOTER_COUNT_READY)
    {
      m_stateShootSwitch = StateShootSwitch::End;
    }
    break;
  case StateShootSwitch::End:
    m_robotContainer.m_feeder.IsNoteLoaded = false;
    m_robotContainer.m_planetary.SetSetpoint(REST_ANGLE);
    m_robotContainer.m_feeder.SetFeeder(STOP_FEEDER_SPEED);
    m_robotContainer.m_shooter.SetShooter(STOP_SHOOTER_SPEED);
    m_shoot = false;
    break;

  default:
    break;
  }
}

void Robot::PreShoot()
{
  if (m_robotContainer.m_camera.getAprilId() == ID_APRILTAG_MIDDLE or m_robotContainer.m_camera.getAprilId() == ID_APRILTAG_LEFT)
  {
    shooter_speed = m_robotContainer.m_shooter.shooterDataTable[m_robotContainer.m_shooter.getNearestElementId(m_robotContainer.m_camera.GetPitch(ID_APRILTAG_MIDDLE, ID_APRILTAG_LEFT))][2];
    planteray_angle = m_robotContainer.m_shooter.shooterDataTable[m_robotContainer.m_shooter.getNearestElementId(m_robotContainer.m_camera.GetPitch(ID_APRILTAG_MIDDLE, ID_APRILTAG_LEFT))][1];
    m_robotContainer.m_shooter.SetShooter(shooter_speed);
    m_robotContainer.m_planetary.SetSetpoint(planteray_angle);
  }
  else
  {
    m_robotContainer.m_shooter.SetShooter(m_robotContainer.m_shooter.shooterDataTable[SHOOTER_TABLE_SIZE - 3][2]);
    m_robotContainer.m_planetary.SetSetpoint(m_robotContainer.m_shooter.shooterDataTable[SHOOTER_TABLE_SIZE - 3][1]);
  }
}

void Robot::Shoot(double speed, double angle)
{
  m_robotContainer.m_planetary.SetSetpoint(angle);
  m_goal = speed * SHOOTER_GOALS_CONVERSION;
  m_count++;
  switch (m_stateShoot)
  {
  case StateShoot::Loaded:
    m_robotContainer.m_feeder.SetFeeder(STOP_FEEDER_SPEED);
    if (m_robotContainer.m_feeder.IsNoteLoaded)
    {
      m_stateShoot = StateShoot::PreShoot;
    }
    break;
  case StateShoot::PreShoot:
    m_robotContainer.m_shooter.SetShooter(speed); // 0.5
    if (NABS(m_robotContainer.m_shooter.GetShooterVelocity()) > m_goal && m_robotContainer.m_planetary.m_planetaryPid.AtSetpoint())
    {
      m_stateShoot = StateShoot::Shoot;
    }
    break;
  case StateShoot::Shoot:
    m_robotContainer.m_feeder.SetFeeder(CATCH_FEEDER_SPEED);
    m_robotContainer.m_shooter.SetShooter(speed); // 0.5
    if (!m_robotContainer.m_feeder.GetFeederInfraSensorValue())
    {
      m_stateShoot = StateShoot::Shooting;
      m_count = 0;
    }
    break;
  case StateShoot::Shooting:
    m_robotContainer.m_feeder.SetFeeder(CATCH_FEEDER_SPEED);
    m_robotContainer.m_shooter.SetShooter(speed);
    if (m_robotContainer.m_feeder.GetFeederInfraSensorValue() && m_count > SHOOTER_COUNT_READY)
    {
      m_stateShoot = StateShoot::End;
    }
    break;
  case StateShoot::End:
    m_robotContainer.m_feeder.IsNoteLoaded = false;
    m_robotContainer.m_planetary.SetSetpoint(REST_ANGLE);
    m_robotContainer.m_feeder.SetFeeder(STOP_FEEDER_SPEED);
    m_robotContainer.m_shooter.SetShooter(STOP_SHOOTER_SPEED);
    break;

  default:
    break;
  }
}

void Robot::Center2Auto()
{
  switch (m_stateCenter2Auto)
  {
  case StateCenter2Auto::Nearshoot:
    NearShoot();
    if (m_stateNearShoot == StateNearShoot::End)
    {
      m_stateTakeNote = StateTakeNote::Catch;
      m_stateCenter2Auto = StateCenter2Auto::Backward;
    }
    break;
  case StateCenter2Auto::Backward:
    m_robotContainer.m_drivetrain.DriveAuto(0.2, 0.0);
    if (m_stateTakeNote == StateTakeNote::Recul)
    {
      m_robotContainer.m_drivetrain.DriveAuto(0.0, 0.0);
      m_stateCenter2Auto = StateCenter2Auto::wait;
    }
    break;
  case StateCenter2Auto::wait:
    if (m_stateTakeNote == StateTakeNote::Loaded)
    {
      m_stateCenter2Auto = StateCenter2Auto::Shooting;
    }
  case StateCenter2Auto::Shooting:
    Shoot(0.7, 23.0); // 0.7 23
    if (m_stateShootSwitch == StateShootSwitch::End)
    {
      m_stateCenter2Auto = StateCenter2Auto::End;
    }
    break;
  }
}

void Robot::ShootOnly()
{
  m_countWait++;
  std::cout << m_countWait << std::endl;
  switch (m_stateShootOnly)
  {
  case StateShootOnly::Wait:
    if (m_countWait > 200)
    {
      m_stateNearShoot = StateNearShoot::PreShoot;
      m_stateShootOnly = StateShootOnly::Nearshoot;
    }
  case StateShootOnly::Nearshoot:
    NearShoot();
    if (m_stateNearShoot == StateNearShoot::End)
    {
      m_stateShootOnly = StateShootOnly::End;
    }
    break;
  case StateShootOnly::Backward:
    m_robotContainer.m_drivetrain.DriveAuto(0.2, 0.0);
    if (NABS(m_robotContainer.m_drivetrain.m_EncoderRight.GetDistance()) > 3000.0 / (6.25 * 25.4 * 3.14))
    {
      m_stateShootOnly = StateShootOnly::End;
    }
    break;
  case StateShootOnly::End:
    m_robotContainer.m_drivetrain.DriveAuto(0.0, 0.0);
    break;
  default:
    break;
  }
}
void Robot::RobotInit()
{
}
void Robot::RobotPeriodic()
{
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::AutonomousInit()
{
  m_countWait = 0;
  m_robotContainer.m_drivetrain.IsAuto = true;
  // m_stateTakeNote = StateTakeNote::nule;

  m_TrajectoryPack.load("/home/lvuser/auto/1metre.trk");
  m_TrajectoryPack.exportTxt("/home/lvuser/auto/test1m.txt");

  // m_gyro.Reset();

  m_CrtzL.m_forwardKv = 2.961074589352691f;
  m_CrtzL.m_backwardKv = 2.955671698032205f;         // = m_kv[1]
  m_CrtzL.m_forwardKa = 0.42279125944509977f;        // = m_ka[0]
  m_CrtzL.m_backwardKa = 0.4037295051102273f;        // = m_ka[1]
  m_CrtzL.m_forwardIntercept = 0.29337288743049417f; // = m_intercept[0]
  m_CrtzL.m_backwardIntercept = -0.2980326729368912f;

  m_CrtzR.m_forwardKv = 2.6368631645896765f;
  m_CrtzR.m_backwardKv = 2.6235298370570757f;        // = m_kv[1]
  m_CrtzR.m_forwardKa = 0.451022875842432f;          // = m_ka[0]
  m_CrtzR.m_backwardKa = 0.45627272443623107f;       // = m_ka[1]
  m_CrtzR.m_forwardIntercept = 0.30401126757243535f; // = m_intercept[0]
  m_CrtzR.m_backwardIntercept = -0.3249879861355316f;

  m_follower.initialize(&m_TrajectoryPack);
  m_state = 0;

  // m_stateShootOnly = StateShootOnly::Backward;
  // m_stateCenter2Auto = StateCenter2Auto::Nearshoot;
  // m_stateNearShoot = StateNearShoot::PreShoot;
}
void Robot::AutonomousPeriodic()
{
  // TakeNoteSwitch();

  // ShootOnly();
  // Center2Auto();

  NLRAMSETEOUTPUT output;
  NLFOLLOWER_TANK_OUTPUT *pout = nullptr;

  NLTRJ_POSTED_MESSAGE message; // Posted Message
  // switch (m_state)
  // {
  // case Robot::STATE::PATH_ERROR:
  //   break;

  // case Robot::STATE::PATH_FOLLOWING:

  //   // *****************************************************    'THE' METHOD(e)
  //   // A) Feed back:
  //   // avec les encodeurs on estime la position du robot:
  //   //			l = distance parcourue par la roue gauche depuis le dernier reset encodeur.
  //   //			r = distance parcourue par la roue droite depuis le dernier reset encodeur.
  //   //
  //   //			dl et dr = distances parcourues par les roues gauche et droite depuis le dernier call.
  //   //			(note dl/dt = vitesse roue gauche et dr/dt = vitesse roue droite  )
  //   //

  //   m_follower.estimate(m_robotContainer.m_drivetrain.m_EncoderLeft.GetDistance(), m_robotContainer.m_drivetrain.m_EncoderRight.GetDistance(), NDEGtoRAD(m_gyro.GetAngle()));
  //   m_follower.updateTarget(&m_TrajectoryPack, 0.02f);
  //   pout = m_follower.compute();
  //   m_robotContainer.m_drivetrain.DriveAuto(m_CrtzR.getVoltage(pout->m_rightVelocity, pout->m_rightAcceleration), m_CrtzL.getVoltage(pout->m_leftVelocity, pout->m_leftAcceleration), 0.0);
  //   std::cout << "pathFollowing" << std::endl;
  //   while (m_follower.getMessage(&message))
  //   {
  //     switch (message.m_id)
  //     {
  //     case TAKE_NOTE:
  //       m_stateTakeNote = StateTakeNote::Catch;
  //       m_takeNote = true;
  //       break;
  //     case ODO_SHOOT:
  //       m_preShoot = false;

  //       break;
  //     case CAMERA_SHOOT:
  //       m_stateShoot = StateShoot::Loaded;
  //       m_preShoot = false;
  //       m_shoot = true;
  //       break;
  //     case PRE_SHOOT:
  //       m_preShoot = true;
  //     default:
  //       break;
  //     }
  //   }

  //   break;
  // case Robot::STATE::PATH_END:
  //   std::cout << "pathEND" << std::endl;
  //   break;
  // default:
  //   NErrorIf(1, NERROR_UNAUTHORIZED_CASE);
  //   break;
  // }

  m_encoderLeftValue = m_robotContainer.m_drivetrain.m_EncoderLeft.GetDistance();
  m_encoderRightValue = m_robotContainer.m_drivetrain.m_EncoderRight.GetDistance();
  m_gyroAngle = 0.0;
  m_follower.estimate(m_encoderLeftValue, m_encoderRightValue, m_gyroAngle);
  m_follower.updateTarget(&m_TrajectoryPack, 0.02f);
  pout = m_follower.compute();
  m_robotContainer.m_drivetrain.SetVoltage(m_CrtzR.getVoltage(pout->m_rightVelocity, 0.0), m_CrtzL.getVoltage(pout->m_leftVelocity, 0.0));

  m_VoltageLeft = m_CrtzL.getVoltage(pout->m_leftVelocity, pout->m_leftAcceleration);
  m_VoltageRight = m_CrtzR.getVoltage(pout->m_rightVelocity, pout->m_rightAcceleration);

  m_LeftV = pout->m_leftVelocity;
  m_RightV = pout->m_rightVelocity;

  std::cout << m_state << std::endl;
  while (m_follower.getMessage(&message))
  {
    switch (message.m_id)
    {
    case NL_CATEGORIZED_MESSAGE_TRJ_START:
      break;

    case NL_CATEGORIZED_MESSAGE_TRJ_TIMEOUT:
      // m_robotContainer.m_drivetrain.SetVoltage(0.0, 0.0);

      break;

    case NL_CATEGORIZED_MESSAGE_TRJ_ENDOFMESSAGE:
      break;

    case TAKE_NOTE:
      m_stateTakeNote = StateTakeNote::Catch;
      m_takeNote = true;
      break;
    case ODO_SHOOT:
      m_preShoot = false;

      break;
    case CAMERA_SHOOT:
      m_stateShoot = StateShoot::Loaded;
      m_preShoot = false;
      m_shoot = true;
      break;
    case PRE_SHOOT:
      m_preShoot = true;
    default:
      break;
    }
  }

  if (m_takeNote)
  {
    TakeNoteSwitch();
  }
  if (m_shoot)
  {
    ShootSwitch();
  }
  if (m_preShoot)
  {
    PreShoot();
  }

  std::cout << "droite" << m_robotContainer.m_drivetrain.m_EncoderRight.GetDistance() << std::endl;
  std::cout << "gauche" << m_robotContainer.m_drivetrain.m_EncoderLeft.GetDistance() << std::endl;
}

void Robot::TeleopInit()
{
  m_robotContainer.m_drivetrain.IsAuto = false;
}
void Robot::TeleopPeriodic()
{
  // std::cout << "droite" << m_robotContainer.m_drivetrain.m_EncoderRight.GetRaw() << std::endl;
  // std::cout << "gauche" << m_robotContainer.m_drivetrain.m_EncoderLeft.GetRaw() << std::endl;
  frc::SmartDashboard::PutBoolean("Loaded", m_robotContainer.m_feeder.IsNoteLoaded);
  frc::SmartDashboard::PutNumber("auto", m_robotContainer.m_drivetrain.m_EncoderLeft.GetDistance());

  if (m_countable < 50 and m_robotContainer.m_feeder.IsRumbled)
  {
    m_countable++;
    std::cout << "cc" << std::endl;
    m_robotContainer.m_xboxControllerCopilote.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.5);
  }
  else
  {
    m_countable = 0;
    m_robotContainer.m_feeder.IsRumbled = false;
    m_robotContainer.m_xboxControllerCopilote.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
  }
}

void Robot::DisabledInit()
{
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
