#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "MPC.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }



int main() {
  uWS::Hub h;

  // MPC is initialized here!

  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {

    /*
    "42" at the start of the message means there's a websocket message event.
    The 4 signifies a websocket message
    The 2 signifies a websocket event
    */

    string sdata = string(data).substr(0, length);
    std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          /**
           * Calculate steering angle and throttle using MPC.
           * Both are in between [-1, 1].
           */


          // start

          // Init the t_Lf and t_dt, same as MPC class
          const double t_Lf = 2.67;
          double t_dt = 0.1;

          // Init Throttle and Steering Angle
          double t_throttle = j[1]["throttle"];
          double t_steer = j[1]["steering_angle"];

          // Transform WayPoint from Vector to Eigen::VectorXd
          Eigen::VectorXd t_ptsx_car = Eigen::VectorXd::Map(ptsx.data(), ptsx.size());
          Eigen::VectorXd t_ptsy_car = Eigen::VectorXd::Map(ptsy.data(), ptsy.size());

          // Transform the cooridination system
          for (int i = 0; i < t_ptsx_car.size(); i++){
            double t_diff_dx = t_ptsx_car[i] - px;
            double t_diff_dy = t_ptsy_car[i] - py;

            t_ptsx_car[i] = t_diff_dx * cos(-psi) - t_diff_dy  * sin(-psi);
            t_ptsy_car[i] = t_diff_dx * sin(-psi) + t_diff_dy * cos(-psi);

          }

          // Fit Polynomial function
          auto t_paras = polyfit(t_ptsx_car, t_ptsy_car, 3);

          // Calculate Cross Track Error
          double t_cte = polyeval(t_paras, 0) - 0.0;

          double t_epsi = 0.0 - atan(t_paras[1]);

          // Update next state
          double t_pred_x   = 0.0 + v * cos(0) * t_dt ;
          double t_pred_y   = 0.0 + v * sin(0) * t_dt ;
          double t_pred_psi = 0.0 + (-t_steer / t_Lf) * v * t_dt;
          double t_pred_v   = v + t_throttle * t_dt;

          double t_pred_cte  = t_cte + v * sin(t_epsi) * t_dt;
          double t_pred_epsi = t_epsi  + v * ( -t_steer / t_Lf) * t_dt;

          // Update current state 
          Eigen::VectorXd t_current_state(6);
          t_current_state << t_pred_x, t_pred_y, t_pred_psi, t_pred_v, t_pred_cte, t_pred_epsi;

          // Call MPC class to get the predict result
          auto mpc_solution = mpc.Solve(t_current_state, t_paras);

          // set the steer and throttle value
          double t_steer_value;
          double t_throttle_value;

          t_steer_value = mpc_solution[0] / (deg2rad(25.0) * t_Lf);
          t_throttle_value = mpc_solution[1];

          // end




          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the 
          //   steering value back. Otherwise the values will be in between 
          //   [-deg2rad(25), deg2rad(25] instead of [-1, 1].

          msgJson["steering_angle"] = t_steer_value;
          msgJson["throttle"] = t_throttle_value;

          // Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;



          // start

          mpc_x_vals.push_back(t_pred_x);
          mpc_y_vals.push_back(t_pred_y);

          for(int i = 2; i < mpc_solution.size(); i += 2 ){

            mpc_x_vals.push_back(mpc_solution[i]);
            mpc_y_vals.push_back(mpc_solution[i + 1]);

          }

          // end



          /**
           * Add (x,y) points to list here, points are in reference to the vehicle's coordinate system the 
           * points in the simulator are connected by a Green line
           */


          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;


          // start 

          double t_latency = 2.5;
          double t_number_latency = 20;

          for (int i = 1; i < t_number_latency; i++){

            next_x_vals.push_back(t_latency * i);
            next_y_vals.push_back(polyeval(t_paras, t_latency * i));

          }

          // end

          /**
           * Add (x,y) points to list here, points are in reference to the vehicle's coordinate system the 
           * points in the simulator are connected by a Yellow line
           */

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;

          /*
          Latency
          The purpose is to mimic real driving conditions where
            the car does actuate the commands instantly.
          
          Feel free to play around with this value but should be to drive
            around the track with 100ms latency.
          
          NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          */

          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
