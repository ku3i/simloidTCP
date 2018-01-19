#include "./configuration.h"
#include "./constants.h"

/* Neue Variablen einfuegen:
   - Variable im Header unter public anlegen
   - in init_parameter_vector() Eintrag erzeugen
   - im Konstruktor initialisieren */

Configuration::Configuration()
: tcp_port         (8000)
, robot            (31)
, scene            (0)
, initial_gravity  (true)
, step_length      (constants::default_step_length)
, real_time        (true)
, initial_pause    (false)
, contact_soft_ERP (constants::contact_soft_ERP)
, contact_soft_CFM (constants::contact_soft_CFM)
, disable_graphics (false)
, draw_scene       (true)
, show_aabb        (false)
, show_contacts    (false)
, show_joints      (false)
, show_accels      (false)
, show_cam_position(true)
, show_time_stat   (true)
, fps              (25.0)
, use_fps_control  (true)
, window_width     (640)
, window_height    (480)
, record_frames    (false)
, init_max_torque  (0.5)
, pidP             (3.5)
, pidI             (0.0)
, pidD             (0.1)
{
    // create list for external access to configuration parameter
    init_parameter_vector();
}

Configuration::~Configuration()
{
}

void Configuration::init_parameter_vector(void)
{
    theParameterVector.clear();
    /* General */
    theParameterVector.push_back(parameter("General"      , "tcp_port"          , &tcp_port          , INT   , "TCP port where to connect client"          ));
    /* Environment   */
    theParameterVector.push_back(parameter("Environment"  , "robot"             , &robot             , INT   , "index number of robot's bodyplan"          ));
    theParameterVector.push_back(parameter("Environment"  , "scene"             , &scene             , INT   , "index number of experimental setup"        ));
    theParameterVector.push_back(parameter("Simulation"   , "initial_gravity"   , &initial_gravity   , BOOL  , "start simulation with gravity on"          ));
    /* Simulation    */
    theParameterVector.push_back(parameter("Simulation"   , "step_length"       , &step_length       , DOUBLE, "step length for one simulation step in s"  ));
    theParameterVector.push_back(parameter("Simulation"   , "real_time"         , &real_time         , BOOL  , "slow down simulation velocity to 1.0x"     ));
    theParameterVector.push_back(parameter("Simulation"   , "initial_pause"     , &initial_pause     , BOOL  , "start simulation in pause-mode"            ));
    theParameterVector.push_back(parameter("Simulation"   , "contact_soft_ERP"  , &contact_soft_ERP  , DOUBLE, "error reduction parameter during contacts" ));
    theParameterVector.push_back(parameter("Simulation"   , "contact_soft_CFM"  , &contact_soft_CFM  , DOUBLE, "constraint force mixing during contacts"   ));
    /* Visualization */
    theParameterVector.push_back(parameter("Visualization", "show_aabb"         , &show_aabb         , BOOL  , "show geom AABBs"                           ));
    theParameterVector.push_back(parameter("Visualization", "show_contacts"     , &show_contacts     , BOOL  , "show contact points"                       ));
    theParameterVector.push_back(parameter("Visualization", "show_joints"       , &show_joints       , BOOL  , "show the joints' anchor and axis"          ));
    theParameterVector.push_back(parameter("Visualization", "show_accels"       , &show_accels       , BOOL  , "show the acceleration sensors"             ));
    theParameterVector.push_back(parameter("Visualization", "show_cam_position" , &show_cam_position , BOOL  , "display camera position"                   ));
    theParameterVector.push_back(parameter("Visualization", "show_time_stat"    , &show_time_stat    , BOOL  , "display time statistics and fps"           ));
    theParameterVector.push_back(parameter("Visualization", "disable_graphics"  , &disable_graphics  , BOOL  , "graphical window or terminal only"         ));
    theParameterVector.push_back(parameter("Visualization", "fps"               , &fps               , DOUBLE, "frames per second"                         ));
    theParameterVector.push_back(parameter("Visualization", "use_fps_control"   , &use_fps_control   , BOOL  , "use fps-controller or draw every step"     ));
    theParameterVector.push_back(parameter("Visualization", "window_width"      , &window_width      , INT   , "window width"                              ));
    theParameterVector.push_back(parameter("Visualization", "window_height"     , &window_height     , INT   , "window height"                             ));
    theParameterVector.push_back(parameter("Visualization", "record_frames"     , &record_frames     , BOOL  , "record frames"                             ));
    /* Controller */
    theParameterVector.push_back(parameter("Controller"   , "init_max_torque"   , &init_max_torque   , DOUBLE, "initial max. torque value for joint motors"));
    theParameterVector.push_back(parameter("Controller"   , "pidP"              , &pidP              , DOUBLE, "P-Value for PID-Controller"                ));
    theParameterVector.push_back(parameter("Controller"   , "pidI"              , &pidI              , DOUBLE, "I-Value for PID-Controller"                ));
    theParameterVector.push_back(parameter("Controller"   , "pidD"              , &pidD              , DOUBLE, "D-Value for PID-Controller"                ));
    return;
}

bool Configuration::readConfigurationFile(const char* filename)
{
  FileHandler fh;
  if (fh.setFile(filename, FileHandler::READONLY) == false) {
	  //Probleme beim Fileopener => neues Configfile schreiben
	  if(createConfigFile(filename) == false) {
		  printf("ERROR: Creating config file!\n");
		  return false;
	  }
	  fh.setFile(filename, FileHandler::READONLY);
  }

  char parameterName[256];
  char parameterValue[256];
  std::string parameterStringValue;
  std::vector<parameter>::const_iterator pos;

  // Einlesen der Parameter
  const char* line;
  while( ( line = fh.readNextLine() ) != NULL) {
	//Zeile einlesen
    int n = sscanf(line,"%s %s", parameterName, parameterValue) ;
    if (n < 2) {
      printf("ERROR: Illegal line: (%s) in config file (line #%d)\n", line, fh.getLineNumber());
      continue ;
    }
	//parameterName in parameterVector suchen
	pos = lookup(theParameterVector, parameterName);
	if(pos == theParameterVector.end()) {
		printf("WARNING: unknown parameter '%s' in config file!\n", parameterName);
		continue;
	}
	switch(pos->type) {
		case STRING:
			{
				char lineCopy[strlen(line)];
				strncpy(lineCopy, line, strlen(line));
				strtok(lineCopy,"\"");
				char* tmp = strtok(NULL,"\"");
				if(tmp != NULL) {
					parameterStringValue = std::string(tmp);
					std::string* target = static_cast<std::string*>(pos->variable);
					*target = parameterStringValue;
				}
				else {
					printf("WARNING: Illegal line: (%s) in config file (line #%d)\n", lineCopy, fh.getLineNumber());
					continue ;
				}
			}
			break;

		case BOOL:
			{
				bool value = false;
				if ( (strncmp( parameterValue, "yes",  3 ) == 0) ||
				   	 (strncmp( parameterValue, "on",   2 ) == 0) ||
				     (strncmp( parameterValue, "true", 4 ) == 0) ||
				     (strncmp( parameterValue, "1",    1 ) == 0) )
				{
					value = true;
				}
				bool* tmp = static_cast<bool*>(pos->variable);
				if (tmp != NULL) {
					*tmp = value;
				}
				else {
				  	printf("WARNING: unknown parameter '%s' in config file.\n", parameterName);
				}
  			}
			break;

		case INT:
			{
				int* tmp = static_cast<int*>(pos->variable);
			  	if (tmp != NULL) {
					*tmp = atoi(parameterValue);
			  	}
			  	else {
				  	printf("WARNING: unknown parameter '%s'.\n", parameterName);
			  	}
			}
			break;

		case DOUBLE:
			{
				double* tmp = static_cast<double*>(pos->variable);
			  	if (tmp != NULL) {
					*tmp = atof(parameterValue);
			  	}
			  	else {
				  	printf("WARNING: unknown parameter '%s'.\n", parameterName);
			  	}
			}
			break;

		default:
			printf("ERROR: unknown parametertype in parameterVector '%s'!\n", parameterName);
	}
  }

  return true;
}

bool Configuration::createConfigFile(const char* filename)
{
	const int firstRowWidth  = 35;
	const int secondRowWidth = 35;

	FileHandler fh;
	fh.setFile(filename, FileHandler::WRITE);
	//Header schreiben
	fh.append("#Configuration file of ");
	fh.append(version::name.c_str());
	fh.append(" simulation environment");
	fh.append('\n');

	std::string category = "";
	//Vector durchgehen
	std::vector<parameter>::const_iterator pos;
	for (pos = theParameterVector.begin(); pos != theParameterVector.end(); ++pos) {
		//Kategorie
		if(category.compare(pos->category) != 0) {
			category = pos->category;
			fh.append('\n');
			fh.append('#');
			fh.append("---");
			fh.append(category);
			fh.append("---");
			fh.append('\n');
		}

		//ParameterName
		fh.append(pos->name);
		int i = firstRowWidth-(pos->name).size();
		while(i-->0) if(fh.append(' ') == false) return false;

		//Value
		i = secondRowWidth;
		switch(pos->type) {
			case STRING:
				i-=fh.append('"');
				i-=fh.append(*(std::string*)pos->variable);
				i-=fh.append('"');
				break;

			case BOOL:
				i-=fh.append((*(bool*)(pos->variable)?"true":"false"));
				break;

			case INT:
				i-=fh.append(*(int*)pos->variable);
				break;

			case DOUBLE:
				i-=fh.append(*(double*)(pos->variable));
				break;

			default:
				printf("ERROR: Unknown parametertype in parameter vector! (%s)\n", (pos->name).c_str());
		}

		while(i-->0) if(fh.append(' ') == false) return false;
		fh.append('#');

		//Description
		if(pos->description.compare("") != 0) {
			fh.append(pos->description);
			fh.append(' ');
		}
		//Type
		switch(pos->type) {
			case STRING:
				fh.append("(string)");
				break;

			case BOOL:
				fh.append("(bool)");
				break;

			case INT:
				fh.append("(int)");
				break;

			case DOUBLE:
				fh.append("(double)");
				break;

			default:
				printf("ERROR: Unknown parametertype in parameter vector! (%s)\n", (pos->name).c_str());
		}

		fh.append('\n');
    }
	return true;
}

std::vector<Configuration::parameter>::const_iterator Configuration::lookup(std::vector<parameter>& pv, const char* name)
{
	std::vector<parameter>::const_iterator pos;
	for(pos=pv.begin(); pos!=pv.end(); ++pos) {
		if(pos->name.compare(name) == 0) return pos;
	}
	return pos;
}

/* fin */

