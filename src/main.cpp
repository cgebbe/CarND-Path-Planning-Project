#include <uWS/uWS.h>
#include <string>
#include <vector>
#include <assert.h>

//#include "../thirdparty/Eigen-3.3/Eigen/Core"
//#include "../thirdparty/Eigen-3.3/Eigen/QR"

#include "helpers.hpp"
#include "json.hpp"
#include "map.h"
#include "own/generate_path.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;


int main() {

    // get map waypoints from file
    struct::Map map = get_map();

    // get messages
    uWS::Hub h;
    h.onMessage([&map]
                (uWS::WebSocket<uWS::SERVER> ws,
                char *data,
                size_t length,
                uWS::OpCode opCode)
    {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (s != "") {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object

                    // Get car position
                    struct::CarPos car_pos;
                    car_pos.x = j[1]["x"];
                    car_pos.y = j[1]["y"];
                    car_pos.s = j[1]["s"];
                    car_pos.d= j[1]["d"];
                    car_pos.yaw = j[1]["yaw"];
                    car_pos.speed = j[1]["speed"];

                    // Remaining path data given to the Planner in previous step
                    struct::Path path_remaining = {
                        j[1]["previous_path_x"],
                                j[1]["previous_path_y"]
                    };
                    //prev_path.x = j[1]["previous_path_x"];
                    //prev_path.y = j[1]["previous_path_y"];
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    // check consistency between prev_path and end_path -> yes :)
                    vector<double> xy = convert_sd_to_xy(end_path_s, end_path_d, map);

                    // Sensor Fusion Data, a list of all other cars on the same side
                    //   of the road.
                    vector<vector<double>> other_cars = j[1]["sensor_fusion"];
                    for (int i=0; i<other_cars.size(); i++) {
                        auto other_car = other_cars[i];
                        double id= other_car[0];
                        double x= other_car[1];
                        double y= other_car[2];
                        double vx = other_car[3];
                        double vy = other_car[4];
                        double s = other_car[5];
                        double d = other_car[6];
                    }


                    // ==================================================
                    // NEW
                    struct::Path next_path = get_next_path(car_pos, path_remaining, map);

                    // ==================================================

                    json msgJson;
                    msgJson["next_x"] = next_path.x;
                    msgJson["next_y"] = next_path.y;

                    auto msg = "42[\"control\","+ msgJson.dump()+"]";

                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }  // end "telemetry" if
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }  // end websocket if
    }); // end h.onMessage

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
