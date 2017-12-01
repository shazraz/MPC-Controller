#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          int n_waypoints = ptsx.size();

          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          
          double v = j[1]["speed"];
          //Convert v to m/s
          v *= 0.44704;

          //Capture current steer and throttle values
          //Steering angle is in radians
          double delta = j[1]["steering_angle"];
          delta *= -1; //Flip the sign of the steering angle after receiving from the simulator to take into account turn direction	

          //Throttle value is in [-1, 1]
          double a = j[1]["throttle"];
         
          //Transform the waypoints returned by the simulator to car coordinates
          //Note that the vehicle direction is the x-axis and the z-axis points out the right of the vehicle
          double sin_psi = sin(psi);
          double cos_psi = cos(psi);

          for (int i = 0; i < n_waypoints; i++){

          	//offset the axis origin to the car location
          	double shift_x = ptsx[i] - px;
          	double shift_y = ptsy[i] - py;

          	//Rotate the waypoint coordinates clockwise by psi
          	//Waypoint coordinates are x, z
          	//Vehicle coordinates are x', z'
          	//Psi is measured positive from x to z
          	//https://en.wikipedia.org/wiki/Transformation_matrix
          	//Use trig identities: 
          	
          	ptsx[i] = (shift_x * cos_psi + shift_y * sin_psi);
          	ptsy[i] = (shift_x * -sin_psi + shift_y * cos_psi);
          }

          //Convert the waypoints arrays to an Eigen vector typedef to be used with MPC::Solve
          //https://stackoverflow.com/a/39157864
          double* ptr_ptsx = &ptsx[0];
          double* ptr_ptsy = &ptsy[0];
          Eigen::Map<Eigen::VectorXd> ptsx_car(ptr_ptsx, n_waypoints);
          Eigen::Map<Eigen::VectorXd> ptsy_car(ptr_ptsy, n_waypoints);

          //Fit the waypoints to a 3rd order polynomial
          auto coeffs = polyfit(ptsx_car, ptsy_car, 3);

          //Calculate the CTE at x = 0 since the car is always at the origin
          double cte = polyeval(coeffs, 0);
          //Calculate error in heading using psi = 0 due to rotation of waypoints
          double epsi = -atan(coeffs[1]);

          //Use kinematic equations to transition state to consider latency
          /*double latency = 0.1; //set latency to 100 ms
          const double Lf = 2.67;

          double x_latency = v * cos(psi) * latency; //cos(0) = 1
          double y_latency = v * sin(psi) * latency; //sin(0) = 0
          double psi_latency = v * delta / Lf * latency; //psi = 0 without latency
          double v_latency = v + a * latency;
          double cte_latency = cte + v * sin(epsi) * latency;
          double epsi_latency = epsi + psi_latency;*/

          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;
          //state << x_latency, y_latency, psi_latency, v_latency, cte_latency, epsi_latency;

          //Call the MPC solver
          auto solution = mpc.Solve(state, coeffs);

          //Construct the message to send back to the simulator
          json msgJson;
          double steer_value;
          double throttle_value;
          steer_value = -solution[0]/deg2rad(25); //flip the steering value and normalize prior to sending to the simulator
          throttle_value = solution[1];

          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //Start at i = 2 to extract mpc x and y vals
          for (unsigned int i = 2; i < solution.size(); i++){
          	if (i%2 == 0){
          		mpc_x_vals.push_back(solution[i]);
          	}
          	else {
          		mpc_y_vals.push_back(solution[i]);
          	}
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //Populate the polynomial coordinates fitted to the waypoints
          int n_points = 20;
          double increment = 2.;

          for (int i = 1; i < n_points; i++){
          	next_x_vals.push_back(i * increment);
          	next_y_vals.push_back(polyeval(coeffs, i * increment));
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          std::cout << "Steer Angle: "<< rad2deg(-solution[0]) << " Throttle Value: " << throttle_value << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          //this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
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
