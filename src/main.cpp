#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

void GetInputs(float &start_x, float &start_y, float &end_x, float &end_y)
{
    int i = 0;
    while(i < 4)
    {
        switch(i)
        {
            case 0: std::cout << "enter a value between 0 and 100 for 'start_x': \n";
                    std::cin >> start_x;
                    if(start_x >= 0 && start_x <= 100) i++;
                    break;
            case 1: std::cout << "enter a value btween 0 and 100 for 'start_y': \n";
                    std::cin >> start_y;
                    if(start_y >= 0 && start_y <= 100) i++;
                    break;
            case 2: std::cout << "enter a value between start_x and 100 for 'end_x': \n"; 
                    std::cin >> end_x;
                    if(end_x >= start_x && end_x <= 100) i++;
                    break;
            case 3: std::cout << "enter a value between start_y and 100 for 'end_y': \n"; 
                    std::cin >> end_y;
                    if(end_y >= start_y && end_y <= 100) i++;
                    break;
        }
    }

}

int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;

        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    
    // 1: Declare the user inputs and get these user inputs
    float start_x, start_y, end_x, end_y;
    GetInputs(start_x, start_y, end_x, end_y);

    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
