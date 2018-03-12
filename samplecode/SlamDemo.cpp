#include "../ZR300Camera.h"
//#include <okvis/VioParametersReader.hpp>
//#include <okvis/ThreadedKFVio.hpp>
#include "../OkvisSLAMSystem.h"

using namespace ark;

int main(int argc, char **argv)
{

  if (argc != 2 && argc != 3) {
    LOG(ERROR)<<
    "Usage: ./" << argv[0] << " configuration-yaml-file [skip-first-seconds]";
    return -1;
  }

  okvis::Duration deltaT(0.0);
  if (argc == 4) {
    deltaT = okvis::Duration(atof(argv[2]));
  }

  // read configuration file
  std::string configFilename(argv[1]);

  OkvisSLAMSystem slam("ORBvoc.yml", configFilename);
  //okvis::VioParameters parameters;
  //vio_parameters_reader.getParameters(parameters);

  //okvis::ThreadedKFVio okvis_estimator(parameters, "ORBvoc.yml");

  //okvis_estimator.setBlocking(false);

  //setup display
  //if (!MyGUI::Manager::init())
  //{
  //  fprintf(stdout, "Failed to initialize GLFW\n");
  //  return -1;
  //}

  std::cout << "camera initialization started" << std::endl << std::flush; 
  ZR300Camera camera;
  
  std::cout << "camera initialization complete" << std::endl << std::flush; 
  //Just a temp window for now
  //MyGUI::CameraWindow path_win("Path Viewer", 1024, 620);


  //okvis::Time start(0.0);
  //okvis::Time t_imu(0.0); 

  //run until display is closed
  while(true){ //MyGUI::Manager::running()){
    //Update the display
    //MyGUI::Manager::update();
    //okvis_estimator.display();
    slam.display();

    //std::cout << "camera update started" << std::endl << std::flush; 
    camera.update();
    //std::cout << "camera update complete" << std::endl << std::flush; 

    //convert to okvis time. Note: rs time is in ms, okvis is in s
    //okvis::Time t_image(camera.getTimeStamp()/1000.0);


    //if (start == okvis::Time(0.0)) {
    //  start = t_image;
    //}

    cv::Mat fisheyeMap = camera.getFishEyeMap();

    slam.PushIMU(camera.getImuData());
    slam.PushFrame(camera.getFishEyeMap(), camera.getTimeStamp()); 
    /*for(size_t i=0; i<imuMap.size(); i++){   
        t_imu = okvis::Time(imuMap[i].timestamp/1000.0);
        if (t_imu - start + okvis::Duration(1.0) > deltaT) {
          okvis_estimator.addImuMeasurement(t_imu, imuMap.at(i).accel, imuMap.at(i).gyro);
        }
    }

      // add the image to the frontend for (blocking) processing
      if (t_image - start > deltaT) {
        okvis_estimator.addImage(t_image, 0, fisheyeMap);
      }

*/
    }
  
  //Clean up
 // MyGUI::Manager::terminate();
  printf("\nDisplay Terminated\n");

  return 0;

}