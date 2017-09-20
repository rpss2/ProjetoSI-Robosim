#include "Aria.h"
#include <iostream>
#include <vector>
#include <queue>
#include <math.h>
#include <utility>

using namespace std;

const int TAM_MATRIZ = 15500 / 400;
const int MATRIZ_METADE = TAM_MATRIZ / 2;
const int INFINITO = 2147483646;

int matriz[TAM_MATRIZ][TAM_MATRIZ];

int inicial_x, inicial_y, head;
int final_x, final_y;

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
  myDesired.setHeading(head);
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

double calc_dist(double x1, double y1, double x2, double y2){
  return sqrt(((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)));
}

ArActionDesired *ActionGo::fire(ArActionDesired currentDesired) {

  //coordenadas do robo no mapa
  double posRoboX = myRobot->getX();
  double posRoboY = myRobot->getY();

  //posicao na matriz do robo
  int posMatrizRoboX = posRoboX / 510;
  int posMatrizRoboY = posRoboY / 510;

  matriz[posMatrizRoboX + MATRIZ_METADE][posMatrizRoboY + MATRIZ_METADE] = -1;

  if(posMatrizRoboX == ((final_x - inicial_x) / 510) && posMatrizRoboY == ((final_y - inicial_y) / 510)) {
    return &myDesired;
  }
  myDesired.reset();

  //priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> heap;
  priority_queue<pair<double,int>,  vector<pair<double,int>>, greater<pair<double,int>>> heap;

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
    if(alcance < 3600) {
      matriz[posMatrizX + MATRIZ_METADE][posMatrizY + MATRIZ_METADE] = INFINITO;
    } else if(matriz[posMatrizX + MATRIZ_METADE][posMatrizY + MATRIZ_METADE] != INFINITO) {
      /*if((matriz[posMatrizX + MATRIZ_METADE][posMatrizY + MATRIZ_METADE] == -1 && i == 15) || 
        (matriz[posMatrizX + MATRIZ_METADE][posMatrizY + MATRIZ_METADE] == 1)) {
        matriz[posMatrizX + MATRIZ_METADE][posMatrizY + MATRIZ_METADE] = 0;*/

        int factor = 1, num_it = 3;
        vector<pair<double,double>> points;
        for(int j = 0 ; j < num_it ; ++j){
          double a = (cordX/factor);
          double b = (cordY/factor);
          factor *= 2;
          points.push_back(make_pair(a,b));                
        }
        for(int j = 0 ; j < num_it ; ++j){
          double distance = calc_dist(points[j].first, points[j].second,(final_x - inicial_x), 
            (final_y - inicial_y));
          heap.push(make_pair(distance/myRobot->getSonarReading(i)->getRange(), i));
        }
      //}
    }
  }

  /*myRobot->lock();
  myGoto->setGoal(ArPose((400 * myMap.find(heap.top())->second.first), (400 * myMap.find(heap.top())->second.second)));
  myRobot->unlock();
*/
  myDesired.setHeading(myRobot->getSonarReading(heap.top().second)->getSensorTh());
  myDesired.setVel(300);
  ArUtil::sleep(100);
  
  return &myDesired;
}

int main(int argc, char**argv) {

  cin >> inicial_x >> inicial_y >> head;
  cin >> final_x >> final_y;


  for(int i = 0; i < TAM_MATRIZ; i++) {
    for(int j = 0; j  < TAM_MATRIZ; j++) {
      matriz[i][j] = 1;
    }
  }

  //-1 escolhido para representar a posição do robo
  //tamanho do robo é 500mm, ent de cada posicao da matriz pra outra
  //é 400mm
  //fazer modulo (%400)

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

  robot.run(true);

  Aria::exit(0);

  return 0;
}