#include "Aria.h"
#include <iostream>

int matriz[1001][1001];

class ActionGo: public ArAction {
  public:
    ActionGo(double maxSpeed, ArActionGoto *go_to);
    virtual ~ActionGo(void) {};
    virtual ArActionDesired *fire(ArActionDesired currentDesired);
    virtual void setRobot(ArRobot *robot);
  protected:
    ArRangeDevice *mySonar;
    ArActionDesired myDesired;
    double myMaxSpeed;
    ArActionGoto *myGoto;
};

ActionGo::ActionGo(double maxSpeed, ArActionGoto *go_to) :
  ArAction("Go")
{
  mySonar = NULL;
  myMaxSpeed = maxSpeed;
  myGoto = go_to;
}

void ActionGo::setRobot(ArRobot *robot)
{
  ArAction::setRobot(robot);
  mySonar = robot->findRangeDevice("sonar");
  if (robot == NULL)
    {
      ArLog::log(ArLog::Terse, "actionExample: ActionGo: Warning: I found no sonar, deactivating.");
      deactivate();
    }
}
ArActionDesired *ActionGo::fire(ArActionDesired currentDesired)
{
  double range;
  double speed;

  myDesired.reset();

  if (mySonar == NULL)
  {
    deactivate();
    return NULL;
  }

  printf("%lf %lf\n", myRobot->getX(), myRobot->getY());

  myRobot->lock();
  myGoto->setGoal(ArPose(3000, 2000));
  myRobot->unlock();

  ArUtil::sleep(100);

  //range = mySonar->currentReadingPolar(-70, 70) - myRobot->getRobotRadius();
  //myDesired.setVel(range);
  
  return &myDesired;
}

int main(int argc, char**argv){

  int infinity = std::numeric_limits<int>::max();
	//-1 escolhido para representar a posição do robo
	//tamanho do robo é 500mm, ent de cada posicao da matriz pra outra
	//é 510mm
	//fazer modulo (%510)

  Aria::init();
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();
  ArRobot robot;
  ArSonarDevice sonar;

  // Connect to the robot, get some initial data from it such as type and name,
  // and then load parameter files for this robot.
  ArRobotConnector robotConnector(&parser, &robot);
  if(!robotConnector.connectRobot()) {
    ArLog::log(ArLog::Terse, "actionExample: Could not connect to the robot.");
    if(parser.checkHelpAndWarnUnparsed()) {
        // -help not given
        Aria::logOptions();
        Aria::exit(1);
    }
  }
  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed()) {
    Aria::logOptions();
    Aria::exit(1);
  }
  ArLog::log(ArLog::Normal, "actionExample: Connected to robot.");

  //adicionando sonar
  robot.addRangeDevice(&sonar);
  //robot.runAsync(true);

  ArActionStallRecover recover;
  ArActionAvoidFront AvoidFront("AvoidFront");
  ArActionGoto gotoPoseAction("goto");
  ActionGo go(400, &gotoPoseAction);

  //adicionando acoes para o robo
  robot.addAction(&recover, 500);
  robot.addAction(&AvoidFront, 450);
  robot.addAction(&go, 400);
  robot.addAction(&gotoPoseAction, 350);

  robot.enableMotors();

/*  while(Aria::getRunning()) {

    for(int i = 0; i < robot.getNumSonar(); i++) {
      
      ArSensorReading *sonar = robot.getSonarReading(i);
      int alcance = robot.getSonarRange(i);
      
      //coordenadas do range
      double cordX = sonar->getX();
      double cordY = sonar->getY();

      //posicao na matriz
      int posMatrizX = cordX / 510;
      int posMatrizY = cordY / 510;

      //coordenadas do robo no mapa
      double posRoboX = robot.getX();
      double posRoboY = robot.getY();

      //posicao na matriz do robo
      int posMatrizRoboX = posRoboX / 510;
      int posMatrizRoboY = posRoboY / 510;

      matriz[posMatrizRoboX][posMatrizRoboY] = -1;

      //preechendo na matriz os lugares alcançados pelo robo
      if(alcance < 4990) {
        matriz[posMatrizX][posMatrizY] = infinity;
      } else if(matriz[posMatrizX][posMatrizY] != infinity) {
        matriz[posMatrizX][posMatrizY] = 0;
      }

    }
  }*/

  robot.run(true);

  Aria::exit(0);

  return 0;
}