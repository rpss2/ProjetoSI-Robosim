#include "Aria.h"
#include <iostream>
#include <vector>
#include <queue>
#include <math.h>
#include <utility>

using namespace std;

const int TAM_MATRIZ = 61;
const int MATRIZ_METADE = TAM_MATRIZ / 2;
const int INFINITO = 2147483646;

int matriz[TAM_MATRIZ][TAM_MATRIZ];
int i_x, i_y, head;
int f_x, f_y;

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
ArActionDesired *ActionGo::fire(ArActionDesired currentDesired) {

    //coordenadas do robo no mapa
    double posRoboX = myRobot->getX() + i_x;
    double posRoboY = myRobot->getY() + i_y;

    //posicao na matriz do robo
    int posMatrizRoboX = posRoboX / 510;
    int posMatrizRoboY = posRoboY / 510;

    matriz[posMatrizRoboX + MATRIZ_METADE][posMatrizRoboY + MATRIZ_METADE] = -1;

    if(posMatrizRoboX == ((f_x - i_x) / 510) && posMatrizRoboY == ((f_y - i_y) / 510)) {
      return &myDesired;
    }
    myDesired.reset();

    for(int i = 0; i < myRobot->getNumSonar(); i++) {
      
      ArSensorReading *sonar = myRobot->getSonarReading(i);
      int alcance = myRobot->getSonarRange(i);
      
      //coordenadas do range
      double cordX = sonar->getX();
      double cordY = sonar->getY();

      //posicao na matriz
      int posMatrizX = cordX / 510;
      int posMatrizY = cordY / 510;

      //preechendo na matriz os lugares alcançados pelo robo
      if(alcance < 4990) {
        matriz[posMatrizX + MATRIZ_METADE][posMatrizY + MATRIZ_METADE] = INFINITO;
      } else if(matriz[posMatrizX][posMatrizY] != INFINITO) {
        matriz[posMatrizX + MATRIZ_METADE][posMatrizY + MATRIZ_METADE] = 0;
      }

    }

    myRobot->lock();
    myGoto->setGoal(ArPose(f_x, f_y));
    myRobot->unlock();

    ArUtil::sleep(100);

  //range = mySonar->currentReadingPolar(-70, 70) - myRobot->getRobotRadius();
  //myDesired.setVel(range);
  
  return &myDesired;
}

int main(int argc, char**argv){

  cin >> i_x >> i_y >> head;
  cin >> f_x >> f_y;

  Aria::init();
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();
  ArRobot robot;
  ArSonarDevice sonar;

  for(int i = 0; i < TAM_MATRIZ; i++) {
    for(int j = 0; j < TAM_MATRIZ; j++) {
      matriz[i][j] = 1;
    }
  }

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

  robot.run(true);

  Aria::exit(0);

  return 0;
}