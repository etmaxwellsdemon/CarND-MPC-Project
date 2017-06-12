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


// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

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
        cout << sdata << endl;
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
                    double steering_angle = j[1]["steering_angle"];
                    double throttle = j[1]["throttle"];
       
                    
                    /*
                     * TODO: Calculate steeering angle and throttle using MPC.
                     *
                     * Both are in between [-1, 1].
                     *
                     */
                    
                    //kinematic model of the car
                    //As there's 100ms latency, first we estimate next state of the car
                    double dt = 0.1;
                    double next_px = px + v*cos(psi)*dt;
                    double next_py = py + v*sin(psi)*dt;
                    double next_psi = psi - v/Lf*steering_angle*dt;
                    double next_v = v + throttle*dt;
                    
                    
                    //then convert map coordinates to car coordinates
                    vector<double> points_x;
                    vector<double> points_y;
                    for (int i = 0; i < ptsx.size(); i++) {
                        
                        double x_shifted = ptsx[i] - next_px;
                        double y_shifted = ptsy[i] - next_py;
                        
                        points_x.push_back(x_shifted * cos(-next_psi) - y_shifted * sin(-next_psi));
                        points_y.push_back(x_shifted * sin(-next_psi) + y_shifted * cos(-next_psi));
                        
                    }
                    
                    //After taht, we fit the x and y values to a 3rd degree poly
                    Eigen::VectorXd points_ex = Eigen::VectorXd::Map(points_x.data(), points_x.size());
                    Eigen::VectorXd points_ey = Eigen::VectorXd::Map(points_y.data(), points_y.size());
                    
                    Eigen::VectorXd poly_coeff = polyfit(points_ex, points_ey, 3);
                    

                    
                    
                    double cte = polyeval(poly_coeff, 0);
                    // Orientation Error
                    double epsi = -atan(poly_coeff(1));
                    
                    Eigen::VectorXd state(6);
                    state << v*dt, 0, -v * steering_angle / Lf * dt, next_v, cte, epsi;
                    
                    //solve the optimizer problem and get the steer and throttle value
                    auto vars = mpc.Solve(state, poly_coeff);
                    
                    double steer_value;
                    double throttle_value;
                    
                    steer_value = vars[0]/ (deg2rad(25)*Lf);
                    throttle_value = vars[1];
                    
                    json msgJson;
                    
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throttle_value;
                    
                    
                    
                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Green line
                    
                    msgJson["mpc_x"] = mpc.x_vals;
                    msgJson["mpc_y"] = mpc.y_vals;
                    
                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Yellow line
                    
                    msgJson["next_x"] = points_x;
                    msgJson["next_y"] = points_y;
                    
                    
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    std::cout << msg << std::endl;
                    // Latency
                    // The purpose is to mimic real driving conditions where
                    // the car does actuate the commands instantly.
                    //
                    // Feel free to play around with this value but should be to drive
                    // around the track with 100ms latency.
                    //
                    // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
                    // SUBMITTING.
                    this_thread::sleep_for(chrono::milliseconds(100));
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
