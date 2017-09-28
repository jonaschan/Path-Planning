#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include <list>

using namespace std;

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
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  // Tutorial : Start in lane 1
  int lane = 1;
  int desired_lane = 0;

  // Tutorial : Have a reference velocity to target
  // Here, if we start say at a high velocity, the car will jerk on start, so if we start with 0, the car will incrementally build its speed...
  double ref_vel = 0.001;

  h.onMessage([&ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane, &desired_lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road. 
            // Use it to check where cars are and how fast they are going.

          	//auto sensor_fusion = j[1]["sensor_fusion"];
            vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];


          	// Tutorial : Store the count of the previous path
          	int prev_size = previous_path_x.size();

            /* This is the part where it checks for collision */
            if (prev_size > 0)
            {
              car_s = end_path_s;
            }

			double distance_between_car = 0.001;
            bool too_close = false;
			bool change_lanes = false;
			double max_speed = 49.5;
            // Here we find the reference velocity (ref_v) to use.
            // Loop through all the vehicles in the sensor fusion list
            for (int i = 0; i< sensor_fusion.size(); i++)
            {
              // If the car in the sensor fusion list is in my lane...
              // d coordinates of the ith car on the road in the sensor fusion database
              float d = sensor_fusion[i][6];

              if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2))
              {
                // Get the velocity in the x direction
                double vx = sensor_fusion[i][3];
                // Get the velocity in the y direction
                double vy = sensor_fusion[i][4];
                // Get the magnitudinal velocity
                double check_speed = sqrt(vx * vx + vy * vy);
                double check_car_s = sensor_fusion[i][5];

                // Project the s values outwards since we are using the previous points to see what the car
                // will be in the future
				check_car_s += ((double)prev_size * 0.02 * check_speed);

				// check_car_s (the other car that we are checking) > car_s (our car), this checks that if the car is in front of us...
				// check_car_s - car_s is the distance between the car in front of us 
				distance_between_car = check_car_s - car_s;

				if ((check_car_s > car_s) && ((check_car_s - car_s) < 30))
				{
					max_speed = check_speed;
					// TODO: Do a decremental change instead of telling it to suddenly change to this velocity.
					// in this case, we're just telling the car to slow down...
					//ref_vel = 29.5;
					too_close = true;

					change_lanes = true;

					cout << "Currently in lane: " << lane << ". Detected car in front!" << endl;
				}
			  }
			}

		std::list<bool> should_change_right;
		std::list<bool> should_change_left;

			//try to change lanes if too close to car in front
			if (change_lanes)
			{
				cout << "Currently in lane: " << lane << ". Deciding to turn..." << endl;

				bool changed_lanes = false;
				//first try to change to left lane
				if (lane != 0 && !changed_lanes)
				{
					bool lane_safe = true;
					for (int i = 0; i < sensor_fusion.size(); i++)
					{
						//car is in left lane
						float d = sensor_fusion[i][6];
						if (d < (2 + 4 * (lane - 1) + 2) && d >(2 + 4 * (lane - 1) - 2))
						{
							double vx = sensor_fusion[i][3];
							double vy = sensor_fusion[i][4];
							double check_speed = sqrt(vx*vx + vy*vy);

							double check_car_s = sensor_fusion[i][5];
							
							check_car_s += ((double)prev_size * 0.02 * check_speed);
							
							double dist_s = check_car_s - car_s;
							distance_between_car = check_car_s - car_s;

							if (dist_s < 30 && dist_s > -30)
							{
								cout << "[LEFT] Not safe to turn at the moment because I'm too close to the car!" << endl;
								should_change_right.push_back(false);
							}
							else 
							{
								if (check_speed >= car_speed)
								{
									if (dist_s < -30)
									{
										cout << "[LEFT] Might be able to turn!" << endl;
										should_change_right.push_back(true);
									}
									else
									{
										cout << "[LEFT] Enough space to turn, but I'm too slow" << endl;
										should_change_right.push_back(false);
									}
								}
								else if (check_speed < car_speed)
								{
									cout << "[LEFT] Possible to turn left" << endl;
									should_change_right.push_back(true);
								}
							}
						}
					}

					bool found = std::find(std::begin(should_change_right), std::end(should_change_right), false) != std::end(should_change_right);

					if (!found && lane == desired_lane)
					{
						desired_lane = lane - 1;
						lane -= 1;

						change_lanes = false;
						changed_lanes = true;
						
						cout << "[LEFT] Turning to lane " << lane << "!" << endl;
						should_change_right.clear();
					}
					else
					{
						cout << "[LEFT] Changing my mind because its not safe!" << endl;
						desired_lane = lane;
						lane = lane;
					}
				}

				if (lane != 2 && !changed_lanes)
				{
					int count = 0;
					bool lane_safe = true;
					for (int i = 0; i < sensor_fusion.size(); i++)
					{
						//car is in left lane
						float d = sensor_fusion[i][6];
						if (d < (2 + 4 * (lane + 1) + 2) && d > (2 + 4 * (lane + 1) - 2))
						{
							double vx = sensor_fusion[i][3];
							double vy = sensor_fusion[i][4];
							double check_speed = sqrt(vx*vx + vy*vy);

							double check_car_s = sensor_fusion[i][5];

							check_car_s += ((double)prev_size * 0.02 * check_speed);

							if (check_car_s - car_s < 30 && check_car_s - car_s > -30)
							{
								cout << "[RIGHT] Not safe to turn at the moment because I'm too close to the car!" << endl;
								should_change_left.push_back(false);

							}
							else 
							{
								if (check_speed >= car_speed)
								{
									if (check_car_s - car_s < -0)
									{
										cout << "[RIGHT] Might be able to turn!" << endl;
										should_change_left.push_back(true);
									}
									else
									{
										cout << "[RIGHT] Enough space to turn, but I'm too slow" << endl;
										should_change_left.push_back(false);
									}
								}
								else
								{
									cout << "[RIGHT] Possible to turn right" << endl;
									should_change_left.push_back(true);
								}
							}
						}
					}					
					bool found = std::find(std::begin(should_change_left), std::end(should_change_left), false) != std::end(should_change_left);

					if (!found && lane == desired_lane)
					{
						desired_lane = lane + 1;
						lane += 1;

						changed_lanes = true;
						change_lanes = false;

						cout << "[RIGHT] Turning to lane " << lane << "!" << endl;
						should_change_left.clear();
					}
					else
					{
						cout << "[RIGHT] Changing my mind because its not safe!" << endl;
						desired_lane = lane;
						lane = lane;
					}
				}
			}

			if (too_close)
			{
				if (ref_vel == max_speed)
				{
					ref_vel == max_speed;
				}
				else if (ref_vel > max_speed)
				{
					ref_vel -= 0.224;
				}

			}
			else if (ref_vel < 49.5)
			{
				if (ref_vel < max_speed)
				{
					ref_vel += 0.224;
				}
				else
				{
					ref_vel == max_speed;
				}
			}

          	// Tutorial : Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m.
          	// These points will later be interpolated with a spline and filled with more points that will
          	// control the speed
          	vector<double> ptsx;
          	vector<double> ptsy;

          	// Tutorial : Reference x, y and yaw states
          	double ref_x = car_x;
          	double ref_y = car_y;
          	double ref_yaw = deg2rad(car_yaw);

          	// Tutorial : If previous size is almost empty, use the car as starting reference
          	if (prev_size < 2)
          	{
          		// Use two points that make the path tangent to the car
          		double prev_car_x = car_x - cos(car_yaw);
          		double prev_car_y = car_y - sin(car_yaw);

          		ptsx.push_back(prev_car_x);
          		ptsx.push_back(car_x);

          		ptsy.push_back(prev_car_y);
          		ptsy.push_back(car_y);
          	}

          	else
          	{
          		ref_x = previous_path_x[prev_size - 1];
          		ref_y = previous_path_y[prev_size - 1];

          		double ref_x_prev = previous_path_x[prev_size - 2];
          		double ref_y_prev = previous_path_y[prev_size - 2];
          		ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

          		ptsx.push_back(ref_x_prev);
          		ptsx.push_back(ref_x);

          		ptsy.push_back(ref_y_prev);
          		ptsy.push_back(ref_y);
          	}

        		// In Frenet, add evenly 30m of evenly spaced points ahead of the starting reference
        		vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
        		vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
        		vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          	ptsx.push_back(next_wp0[0]);
          	ptsx.push_back(next_wp1[0]);
          	ptsx.push_back(next_wp2[0]);

        		ptsy.push_back(next_wp0[1]);
        		ptsy.push_back(next_wp1[1]);
        		ptsy.push_back(next_wp2[1]);

        		for (int i = 0; i < ptsx.size(); i++)
        		{
        			double shift_x = ptsx[i] - ref_x;
        			double shift_y = ptsy[i] - ref_y;

    				//Transformation to local car coordinates. Similar to MPC.
        			ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        			ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
        		}

      		  tk::spline s;

        		// *Here we add the anchor points to the splines.* 
        		s.set_points(ptsx, ptsy);

        		// Define the actual x and y points to be used for the planner
        		vector<double> next_x_vals;
        		vector<double> next_y_vals;

      		  // *Here we add the path planning points*
      		  // Start by adding the previous points from the last time
        		for(int i = 0; i < previous_path_x.size(); i++)
        		{
        			next_x_vals.push_back(previous_path_x[i]);
        			next_y_vals.push_back(previous_path_y[i]);      			
        		}

      		  // *Here we are looking back at the anchor points again...*
      		  // Calculate how to break up the spline points so that we travel at our desired velocity
      		  double target_x = 30.0;
      		  double target_y = s(target_x);
      		  double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

      		  double x_add_on = 0;

      		// Fill up the rest of the path planner after filling it with previous points, here we will always output 50 points.
      		// previous_path_x is the points that has been used earlier. 50 - previos_path_x is the remaining that never got used
      		// so instead of regenerating these points again, we reuse them by adding them back to the planner list. Here we are adding back the points
      		// along the spline. Watch tutorial @ time: 33.11.

      		// 0.02 is equal to 1/50 which is the time for each frame, 2.24 is the conversion from miles per hour to meters per second.
      		for(int i = 0; i <= 50 - previous_path_x.size(); i++)
      		{
      			double N = (target_dist / (0.02 * ref_vel / 2.24));
      			double x_point = x_add_on + (target_x) / N;
      			double y_point = s(x_point);

      			x_add_on = x_point;

      			double x_ref = x_point;
      			double y_ref = y_point;

      			// Rotate back after rotating it earlier
      			x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
				    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

    				x_point += ref_x;
    				y_point += ref_y;

    				next_x_vals.push_back(x_point);
    				next_y_vals.push_back(y_point);
      		}

          	json msgJson;
      		msgJson["next_x"] = next_x_vals;
      		msgJson["next_y"] = next_y_vals;

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	auto msg = "42[\"control\","+msgJson.dump()+"]";
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
