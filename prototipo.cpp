#include "Aria.h"




int main(int argc, char**argv){

	int matriz[1001][1001];
	matriz[500][500] = -1;
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
  	if(!robotConnector.connectRobot())
  	{
    	ArLog::log(ArLog::Terse, "actionExample: Could not connect to the robot.");
    	if(parser.checkHelpAndWarnUnparsed())
    	{
        	// -help not given
        	Aria::logOptions();
        	Aria::exit(1);
    	}
  	}
  	if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
  	{
    	Aria::logOptions();
    	Aria::exit(1);
  	}
  	ArLog::log(ArLog::Normal, "actionExample: Connected to robot.");
  	cout<<(robot.getPos())
  	//adicionandno sonar
  	robot.addRangeDevice(&sonar);

  	double distSonar;
  	int distSonarInt = static_cast<int>(distSonar); //valor inteiro da distancia que ele ve
  	int valorDesloc = distSonarInt%510;


  	robot.enableMotors();
  	robot.run(true);
  	Aria.exit(0);]

	

}