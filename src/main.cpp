#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

#define TWIDDLE_ON 0
#define TARGET_SPEED 50
#define MAX_CTE 4.0

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}



int main()
{
  uWS::Hub h;

  //PID for steering 
  PID pid_steer;
  //start values
  //double Kp_steer = 0.1;
  //double Ki_steer = 0.0;
  //double Kd_steer = 1.0;
  //double dKp_steer = 0.05;
  //double dKi_steer = 0.0025;
  //double dKd_steer = 0.5;  
  
  // Twiddle with 60 mph
  //best twiddle collection steps: 691
  // Best Kp: 0.095 Kd: 1.86086 Ki: 0.00227475 Error: 0.631308
  //double Kp_steer = 0.095;
  //double Ki_steer = 0.00227475;
  //double Kd_steer = 1.86086;
  
  //Next Twiddle with 60
  //Best Kp: 0.116 Kd: 1.86086 Ki: 0.00252475 Error: 0.536394
  double Kp_steer = 0.116;
  double Ki_steer = 0.00252475;
  double Kd_steer = 1.86086;
  
  pid_steer.Init(Kp_steer, Ki_steer, Kd_steer);

  double dKp_steer = 0.01;
  double dKi_steer = 0.00025;
  double dKd_steer = 0.05;  

  if (TWIDDLE_ON) pid_steer.initTwiddle(dKp_steer, dKi_steer, dKd_steer, MAX_CTE);
  
  //PID for speed control
  PID pid_CC;
  double Kp_CC = 0.1;
  double Ki_CC = 0.005;
  double Kd_CC = 1.0;
  pid_CC.Init(Kp_CC, Ki_CC, Kd_CC);
  
  h.onMessage([&pid_steer, &pid_CC](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        //DEBUG
        //std::cout << "s: " << s << std::endl;
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          //std::cout << "cte: " << cte << "; speed: " << speed << "; angle: " << angle << std::endl;
          pid_steer.UpdateError(-cte);
          double steer_value = pid_steer.TotalError();
          
          bool reset = false;
          if(TWIDDLE_ON) reset = pid_steer.Twiddle(cte);

          //cruise control
          double target_speed = TARGET_SPEED;
          pid_CC.UpdateError(target_speed-speed);
          double throttle = pid_CC.TotalError();

          if (reset) 
           {
              std::string reset_msg = "42[\"reset\",{}]";
              ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
           }
          else
           {
              json msgJson;
              msgJson["steering_angle"] = steer_value;
              msgJson["throttle"] = throttle;
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              //std::cout << msg << std::endl;
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
           }
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
