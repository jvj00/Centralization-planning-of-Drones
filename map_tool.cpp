#include <iostream>
#include <fstream>
#include <algorithm>
#include <string.h>
#include <math.h>
#include <curl/curl.h>

using namespace std;

#define MAX_HEIGHT 100
#define MAX_WIDTH 100
#define MAX_VISIT_POINTS 100
#define MAX_LEVELS 5
#define MAX_INCLINATION 0.2 //corresponds to ~12°

#define X 0
#define Y 1
#define Z 2
#define DIR 3
#define DBL_MAX 1.7976931348623157E+308
#define PI 3.14159265358979323846
#define API_ELEVATION "https://api.open-elevation.com/api/v1/lookup"


char* getCmdOption(char ** begin, char ** end, const std::string & option)
{
    char ** itr = std::find(begin, end, option);
    if (itr != end && ++itr != end)
    {
        return *itr;
    }
    return 0;
}
bool cmdOptionExists(char** begin, char** end, const std::string& option)
{
    return std::find(begin, end, option) != end;
}
bool is_number_int(const string& s)
{
    string::const_iterator it = s.begin();
    while (it != s.end() && isdigit(*it)) ++it;
    return !s.empty() && it == s.end();
}
bool is_number(const string& s)
{
    string::const_iterator it = s.begin();
    while (it != s.end() && (isdigit(*it) || *it=='.')) ++it;
    return !s.empty() && it == s.end();
}
void common_error()
{
    cerr << "Wrong usage of this tool. Type --help or -h to see the guide" << endl; 
    exit(1);
}


void compute_drone_pos(int l, int h, int w, string points[MAX_LEVELS][MAX_HEIGHT][MAX_WIDTH], double points_coord[MAX_LEVELS][MAX_HEIGHT][MAX_WIDTH][3], double* drone_coord, string* drone_pos)
{
    double drone_min_value=DBL_MAX;
    for(int i=0;i<l;i++)
        for(int j=0;j<h;j++)
            for(int k=0;k<w;k++)
            {
                double drone_dist=sqrt(pow(drone_coord[X]-points_coord[i][j][k][X],2)+pow(drone_coord[Y]-points_coord[i][j][k][Y],2)+pow(drone_coord[Z]-points_coord[i][j][k][Z],2));
                if(drone_dist<drone_min_value)
                {
                    drone_min_value=drone_dist;
                    *drone_pos=points[i][j][k];
                }
            }
}
void compute_agv_pos(int l, int h, int w, string points_agv[MAX_HEIGHT][MAX_WIDTH], double points_coord[MAX_LEVELS][MAX_HEIGHT][MAX_WIDTH][3], double* agv_coord, string* agv_pos)
{
    double agv_min_value=DBL_MAX;
    for(int j=0;j<h;j++)
        for(int k=0;k<w;k++)
        {
            double agv_dist=sqrt(pow(agv_coord[X]-points_coord[0][j][k][X],2)+pow(agv_coord[Y]-points_coord[0][j][k][Y],2));
            if(agv_dist<agv_min_value)
            {
                agv_min_value=agv_dist;
                *agv_pos=points_agv[j][k];
            }
        }
}
void compute_targets_pos(int l, int h, int w, int ntarget, string points[MAX_LEVELS][MAX_HEIGHT][MAX_WIDTH], double points_coord[MAX_LEVELS][MAX_HEIGHT][MAX_WIDTH][3], double targets_coord[MAX_VISIT_POINTS][3], string targets[MAX_VISIT_POINTS])
{
    double targets_min_value[MAX_VISIT_POINTS];
    for(int i=0;i<MAX_VISIT_POINTS;i++)
        targets_min_value[i]=DBL_MAX;

    for(int n=0; n<ntarget; n++)
    {
        for(int i=0;i<l;i++)
            for(int j=0;j<h;j++)
                for(int k=0;k<w;k++)
                {
                    double dist=sqrt(pow(targets_coord[n][X]-points_coord[i][j][k][X],2)+pow(targets_coord[n][Y]-points_coord[i][j][k][Y],2)+pow(targets_coord[n][Z]-points_coord[i][j][k][Z],2));
                    if(dist<targets_min_value[n])
                    {
                        targets_min_value[n]=dist;
                        targets[n]=points[i][j][k];
                    }
                }
    }
    
}

//GEOSPATIAL FUNCTIONS
void pointAtDistance(long double latC, long double lonC, long double dx, long double dy, long double* lat, long double* lon)
{
    #define circEq 40075.017
    #define circPol 40007.863
    *lat = latC + (dy * 360 / circPol / 1000);
    *lon = lonC + (dx * 360 / circEq / 1000 / cos((latC+*lat)*PI/360));
}

int minElevation(int elevation[MAX_HEIGHT][MAX_WIDTH], int h, int w)
{
    int minValue=100000;
    for(int i=0; i<h; i++)
        for(int j=0; j<w; j++)
            minValue= elevation[i][j]<minValue ? elevation[i][j] : minValue;
    return minValue;
}

//CURL FUNCTIONS
string createJSON(long double lat[], long double lon[], int arr_length)
{
    string ret="{\"locations\":[";
    for(int i=0; i<arr_length; i++)
    {
        ret+="{\"latitude\": "+to_string(lat[i])+",\"longitude\": "+to_string(lon[i])+"}";
        if(i!=arr_length-1) ret+=",";
    }
    ret+="]}";
    return ret;
}
int readJSON(string buffer, int elevation_out[MAX_HEIGHT][MAX_WIDTH], int arr_length, int w)
{
    int index = 0, stop=0;
    index = buffer.find("results"); 
    if(index==string::npos) return 1;
    for(int i=0; i<arr_length && index<buffer.length(); i++)
    {
        index = buffer.find("elevation", index) + 12;
        stop = buffer.find("}", index);
        elevation_out[int(i/w)][i%w] = stoi(buffer.substr(index, stop-index));
    }
    return 0;
}
static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}
int getElevation(long double lat[], long double lon[], int elevation_out[MAX_HEIGHT][MAX_WIDTH], int arr_length, int w)
{
    CURL *curl;
    CURLcode res;
    string readBuffer;

    curl = curl_easy_init();
    if(curl) {
        //URL
        curl_easy_setopt(curl, CURLOPT_URL, API_ELEVATION);

        //HEADER DATA
        struct curl_slist* headers = NULL;
        headers = curl_slist_append(headers, "Content-Type: application/json");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

        //BODY DATA
        string data = createJSON(lat, lon, arr_length);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, (long) strlen(data.c_str()));
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data.c_str());

        //RESPONSE
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);

        //FORMAT RESPONSE
        readJSON(readBuffer, elevation_out, arr_length, w);
    }
    return 0;
}

int main(int argc, char** argv)
{
    int h, w, l, ntarget=0, index_arguments=1;
    bool pddl=false, links=false, json=false, return_drones=false;
    char *pddl_file, *links_file, *json_file;
    double dst_pts, dst_lvl, low_lvl;
    string points[MAX_LEVELS][MAX_HEIGHT][MAX_WIDTH], points_agv[MAX_HEIGHT][MAX_WIDTH];
    double points_coord[MAX_LEVELS+1][MAX_HEIGHT][MAX_WIDTH][3];
    string out_link_pddl="",out_distance_pddl="", out_list_pddl="", out_coordinates="", out_empty_pddl="", out_link_agv_pddl="", out_distance_agv_pddl="", out_link_json="";
    string agv_pos="", d1_pos="", d2_pos="", targets[MAX_VISIT_POINTS];
    double agv_coord[2], targets_coord[MAX_VISIT_POINTS][3], d1_coord[3], d2_coord[3];
    long double latC, lonC;

    //PARSING ARGUMENTS
    //OUTPUTS
    if(cmdOptionExists(argv, argv+argc, "--pddl"))
    {
        pddl=true;
        pddl_file=getCmdOption(argv, argv + argc, "--pddl");
        index_arguments+=2;
        if(!pddl_file) common_error();
    }
    if(cmdOptionExists(argv, argv+argc, "--links"))
    {
        links=true;
        links_file=getCmdOption(argv, argv + argc, "--links");
        index_arguments+=2;
        if(!links_file) common_error();
    }
    if(cmdOptionExists(argv, argv+argc, "--json"))
    {
        json=true;
        json_file=getCmdOption(argv, argv + argc, "--json");
        index_arguments+=2;
        if(!json_file) common_error();
    }
    if(cmdOptionExists(argv, argv+argc, "--return"))
    {
        return_drones=true;
        index_arguments++;
    }
    //PARAMETERS
    if(!return_drones && argc>=13+index_arguments && (argc-(13+index_arguments))%3==0 && is_number_int(argv[index_arguments]) && is_number_int(argv[1+index_arguments]) && is_number_int(argv[2+index_arguments]) && is_number(argv[3+index_arguments]) && is_number(argv[4+index_arguments]) && is_number(argv[5+index_arguments]) && is_number(argv[6+index_arguments]) && is_number(argv[7+index_arguments]) && is_number(argv[8+index_arguments]) && is_number(argv[9+index_arguments]))
    {
        h = atoi(argv[index_arguments++]);
        w = atoi(argv[index_arguments++]);
        l = atoi(argv[index_arguments++]);
        dst_pts = stod(argv[index_arguments++]);
        dst_lvl = stod(argv[index_arguments++]);
        low_lvl = stod(argv[index_arguments++]);
        agv_coord[X] = stod(argv[index_arguments++]);
        agv_coord[Y] = stod(argv[index_arguments++]);
        latC = stold(argv[index_arguments++]);
        lonC = stold(argv[index_arguments++]);
        for(int i=index_arguments;i<argc && i<MAX_VISIT_POINTS*3+index_arguments;i+=3)
        {
            if(!is_number(argv[i]) || !is_number(argv[i+1]) || !is_number(argv[i+2])) common_error();
            targets_coord[ntarget][X]=stod(argv[i]);
            targets_coord[ntarget][Y]=stod(argv[i+1]);
            targets_coord[ntarget][Z]=stod(argv[i+2]);
            ntarget++;
        }
    }
    else if(return_drones && argc==16+index_arguments && is_number_int(argv[index_arguments]) && is_number_int(argv[1+index_arguments]) && is_number_int(argv[2+index_arguments]) && is_number(argv[3+index_arguments]) && is_number(argv[4+index_arguments]) && is_number(argv[5+index_arguments]) && is_number(argv[6+index_arguments]) && is_number(argv[7+index_arguments]) && is_number(argv[8+index_arguments]) && is_number(argv[9+index_arguments]) && is_number(argv[10+index_arguments]) && is_number(argv[11+index_arguments]) && is_number(argv[12+index_arguments]) && is_number(argv[13+index_arguments]) && is_number(argv[14+index_arguments]) && is_number(argv[15+index_arguments]))
    {
        h = atoi(argv[index_arguments++]);
        w = atoi(argv[index_arguments++]);
        l = atoi(argv[index_arguments++]);
        dst_pts = stod(argv[index_arguments++]);
        dst_lvl = stod(argv[index_arguments++]);
        low_lvl = stod(argv[index_arguments++]);
        agv_coord[X] = stod(argv[index_arguments++]);
        agv_coord[Y] = stod(argv[index_arguments++]);
        latC = stold(argv[index_arguments++]);
        lonC = stold(argv[index_arguments++]);
        d1_coord[X]=stod(argv[index_arguments++]);
        d1_coord[Y]=stod(argv[index_arguments++]);
        d1_coord[Z]=stod(argv[index_arguments++]);
        d2_coord[X]=stod(argv[index_arguments++]);
        d2_coord[Y]=stod(argv[index_arguments++]);
        d2_coord[Z]=stod(argv[index_arguments++]);
    }
    //HELP & ERRORS
    else if(argc==2 && (!((string)argv[1]).compare("--help") || !((string)argv[1]).compare("-h")))
    {
        cout << "This tool generate a 3D map of points, giving height, width and levels, distance between points in meters, lowest level in meters, agv position and targets position. 2 drones are on AGV" << endl << endl;
        cout << "Usage: ./map_tool [options] <height> <width> <levels> <distance_pts> <distance_lvl> <lowest_lvl> <agv_x> <agv_y> <latC> <lonC> (<t1_x t1_y t1_z, t2_x t2_y t2_z ...>)or(<d1_x d1_y d1_z, d2_x d2_y d2_z>)" << endl;
        cout << endl;
        cout << "Parameters:" << endl;
        cout << "   <height(pts)> height 2D (y) in points of the map" << endl;
        cout << "   <width(pts)> width 2D (x) in points of the map" << endl;
        cout << "   <levels> # of levels (z) of the map" << endl;
        cout << "   <distance_pts> distance in meters between points of the same level" << endl;
        cout << "   <distance_lvl> distance in meters between levels" << endl;
        cout << "   <lowest_lvl> heigth in meters of the lowest level" << endl;
        cout << "   <agv_x> <agv_y> position (x,y) of the agv. E.g.: 22.1 23.4" << endl;
        cout << "   <latC> <lonC> latitude and longitude of initial point (double dot value). E.g.: 40.93257 30.27429" << endl;
        cout << "   <t1_x t1_y t1_z, t2_x t2_y t2_z ...> list of target position (x,y,z) of the planning. E.g.: 30.1 40.2 25.1" << endl;
        cout << "   <d1_x d1_y d1_z, d2_x d2_y d2_z> drones position (x,y,z) for the return to the agv. E.g.: 30.1 40.2 25.1 23.6 45.0 67.3" << endl;
        cout << endl;
        cout << "Options:" << endl;
        cout << "   --return                           problem file to return drones to agv" << endl;
        cout << "   --pddl <path_to_out_file>          generates a pddl problem as output" << endl;
        cout << "   --links <path_to_out_file>         generates a json file with links as secondary output" << endl;
        cout << "   --json <path_to_out_file>          generates a json file with coordinates as secondary output" << endl;
        exit(0);
    }
    else
    {
        common_error();
    }

    //MAIN PROGRAM

    //COMPUTE ELEVATIONS
    int geocoord_length=h*w;
    long double lat[geocoord_length], lon[geocoord_length];
    int el[MAX_HEIGHT][MAX_WIDTH];
    for(int j=0;j<h;j++)
        for(int k=0;k<w;k++)
            pointAtDistance(latC, lonC, k*dst_pts, j*dst_pts, &lat[j*w+k], &lon[j*w+k]);
    getElevation(lat, lon, el, geocoord_length, w);

    //COMPUTES POINTS & COORDINATES 2 POINTS
    for(int i=0;i<l;i++)
        for(int j=0;j<h;j++)
            for(int k=0;k<w;k++)
            {
                points[i][j][k] = "p" + to_string((i*w*h)+(j*w)+k);

                points_coord[i][j][k][X]=k*dst_pts;
                points_coord[i][j][k][Y]=j*dst_pts;
                points_coord[i][j][k][Z]=i*dst_lvl+low_lvl+el[j][k]; //elevations type: SLM
                out_coordinates += "{\"name\":\"" + points[i][j][k] + "\",\"x\":" + to_string(points_coord[i][j][k][X]) + ",\"y\":" + to_string(points_coord[i][j][k][Y]) + ",\"z\":" + to_string(points_coord[i][j][k][Z]) + "}";
                if(i==0)
                {
                    points_agv[j][k] = "pagv" + to_string((j*w)+k);
                    out_coordinates += ",{\"name\":\"" + points_agv[j][k] + "\",\"x\":" + to_string(points_coord[i][j][k][X]) + ",\"y\":" + to_string(points_coord[i][j][k][Y]) + ",\"z\":" + to_string(el[j][k]) + "}";
                }
                if(i!=l-1 || j!=h-1 || k!=w-1)
                    out_coordinates += ",";
                
            }
    if(return_drones)
    {
        compute_drone_pos(l, h, w, points, points_coord, d1_coord, &d1_pos);
        compute_drone_pos(l, h, w, points, points_coord, d2_coord, &d2_pos);
    }
    compute_agv_pos(l,h,w, points_agv, points_coord, agv_coord, &agv_pos);
    compute_targets_pos(l,h,w, ntarget, points, points_coord, targets_coord, targets);    

    //COMPUTES LINKS & DISTANCES DRONES
    for(int i=0;i<l;i++)
        for(int j=0;j<h;j++)
            for(int k=0;k<w;k++)
            {
                //points
                out_list_pddl += points[i][j][k] + " ";
                out_empty_pddl += "    (empty " + points[i][j][k] + ")\n";

                //links && distances in the level
                if(j>0 && k>0) //nw
                {
                    out_link_pddl += "    (link " + points[i][j][k] + " " + points[i][j-1][k-1] + ")\n";
                    out_distance_pddl += "    (= (distance " + points[i][j][k] + " " + points[i][j-1][k-1] + ") " + to_string(sqrt(2*pow(dst_pts,2)+pow(points_coord[i][j][k][Z]-points_coord[i][j-1][k-1][Z],2))) + ")\n";
                    out_link_json += "{\"type\": \"drone\",\"from\":{\"x\":"+to_string(points_coord[i][j][k][X])+",\"y\":"+to_string(points_coord[i][j][k][Y])+",\"z\":"+to_string(points_coord[i][j][k][Z])+"},\"to\":{\"x\":"+to_string(points_coord[i][j-1][k-1][X])+",\"y\":"+to_string(points_coord[i][j-1][k-1][Y])+",\"z\":"+to_string(points_coord[i][j-1][k-1][Z])+"}},";
                } 
                if(j>0) //nn
                {
                    out_link_pddl += "    (link " + points[i][j][k] + " " + points[i][j-1][k] + ")\n";
                    out_distance_pddl += "    (= (distance " + points[i][j][k] + " " + points[i][j-1][k] + ") " + to_string(sqrt(pow(dst_pts,2)+pow(points_coord[i][j][k][Z]-points_coord[i][j-1][k][Z],2))) + ")\n";
                    out_link_json += "{\"type\": \"drone\",\"from\":{\"x\":"+to_string(points_coord[i][j][k][X])+",\"y\":"+to_string(points_coord[i][j][k][Y])+",\"z\":"+to_string(points_coord[i][j][k][Z])+"},\"to\":{\"x\":"+to_string(points_coord[i][j-1][k][X])+",\"y\":"+to_string(points_coord[i][j-1][k][Y])+",\"z\":"+to_string(points_coord[i][j-1][k][Z])+"}},";
                } 
                if(j>0 && k<w-1) //ne
                {
                    out_link_pddl += "    (link " + points[i][j][k] + " " + points[i][j-1][k+1] + ")\n";
                    out_distance_pddl += "    (= (distance " + points[i][j][k] + " " + points[i][j-1][k+1] + ") " + to_string(sqrt(2*pow(dst_pts,2)+pow(points_coord[i][j][k][Z]-points_coord[i][j-1][k+1][Z],2))) + ")\n";
                    out_link_json += "{\"type\": \"drone\",\"from\":{\"x\":"+to_string(points_coord[i][j][k][X])+",\"y\":"+to_string(points_coord[i][j][k][Y])+",\"z\":"+to_string(points_coord[i][j][k][Z])+"},\"to\":{\"x\":"+to_string(points_coord[i][j-1][k+1][X])+",\"y\":"+to_string(points_coord[i][j-1][k+1][Y])+",\"z\":"+to_string(points_coord[i][j-1][k+1][Z])+"}},";
                } 
                if(k<w-1) //ee
                {
                    out_link_pddl += "    (link " + points[i][j][k] + " " + points[i][j][k+1] + ")\n";
                    out_distance_pddl += "    (= (distance " + points[i][j][k] + " " + points[i][j][k+1] + ") " + to_string(sqrt(pow(dst_pts,2)+pow(points_coord[i][j][k][Z]-points_coord[i][j][k+1][Z],2))) + ")\n";
                    out_link_json += "{\"type\": \"drone\",\"from\":{\"x\":"+to_string(points_coord[i][j][k][X])+",\"y\":"+to_string(points_coord[i][j][k][Y])+",\"z\":"+to_string(points_coord[i][j][k][Z])+"},\"to\":{\"x\":"+to_string(points_coord[i][j][k+1][X])+",\"y\":"+to_string(points_coord[i][j][k+1][Y])+",\"z\":"+to_string(points_coord[i][j][k+1][Z])+"}},";
                } 
                if(j<h-1 && k<w-1) //se
                {
                    out_link_pddl += "    (link " + points[i][j][k] + " " + points[i][j+1][k+1] + ")\n";
                    out_distance_pddl += "    (= (distance " + points[i][j][k] + " " + points[i][j+1][k+1] + ") " + to_string(sqrt(2*pow(dst_pts,2)+pow(points_coord[i][j][k][Z]-points_coord[i][j+1][k+1][Z],2))) + ")\n";
                    out_link_json += "{\"type\": \"drone\",\"from\":{\"x\":"+to_string(points_coord[i][j][k][X])+",\"y\":"+to_string(points_coord[i][j][k][Y])+",\"z\":"+to_string(points_coord[i][j][k][Z])+"},\"to\":{\"x\":"+to_string(points_coord[i][j+1][k+1][X])+",\"y\":"+to_string(points_coord[i][j+1][k+1][Y])+",\"z\":"+to_string(points_coord[i][j+1][k+1][Z])+"}},";
                } 
                if(j<h-1) //ss
                {
                    out_link_pddl += "    (link " + points[i][j][k] + " " + points[i][j+1][k] + ")\n";
                    out_distance_pddl += "    (= (distance " + points[i][j][k] + " " + points[i][j+1][k] + ") " + to_string(sqrt(pow(dst_pts,2)+pow(points_coord[i][j][k][Z]-points_coord[i][j+1][k][Z],2))) + ")\n";
                    out_link_json += "{\"type\": \"drone\",\"from\":{\"x\":"+to_string(points_coord[i][j][k][X])+",\"y\":"+to_string(points_coord[i][j][k][Y])+",\"z\":"+to_string(points_coord[i][j][k][Z])+"},\"to\":{\"x\":"+to_string(points_coord[i][j+1][k][X])+",\"y\":"+to_string(points_coord[i][j+1][k][Y])+",\"z\":"+to_string(points_coord[i][j+1][k][Z])+"}},";
                } 
                if(j<h-1 && k>0) //sw
                {
                    out_link_pddl += "    (link " + points[i][j][k] + " " + points[i][j+1][k-1] + ")\n";
                    out_distance_pddl += "    (= (distance " + points[i][j][k] + " " + points[i][j+1][k-1] + ") " + to_string(sqrt(2*pow(dst_pts,2)+pow(points_coord[i][j][k][Z]-points_coord[i][j+1][k-1][Z],2))) + ")\n";
                    out_link_json += "{\"type\": \"drone\",\"from\":{\"x\":"+to_string(points_coord[i][j][k][X])+",\"y\":"+to_string(points_coord[i][j][k][Y])+",\"z\":"+to_string(points_coord[i][j][k][Z])+"},\"to\":{\"x\":"+to_string(points_coord[i][j+1][k-1][X])+",\"y\":"+to_string(points_coord[i][j+1][k-1][Y])+",\"z\":"+to_string(points_coord[i][j+1][k-1][Z])+"}},";
                } 
                if(k>0) //ww
                {
                    out_link_pddl += "    (link " + points[i][j][k] + " " + points[i][j][k-1] + ")\n";
                    out_distance_pddl += "    (= (distance " + points[i][j][k] + " " + points[i][j][k-1] + ") " + to_string(sqrt(pow(dst_pts,2)+pow(points_coord[i][j][k][Z]-points_coord[i][j][k-1][Z],2))) + ")\n";
                    out_link_json += "{\"type\": \"drone\",\"from\":{\"x\":"+to_string(points_coord[i][j][k][X])+",\"y\":"+to_string(points_coord[i][j][k][Y])+",\"z\":"+to_string(points_coord[i][j][k][Z])+"},\"to\":{\"x\":"+to_string(points_coord[i][j][k-1][X])+",\"y\":"+to_string(points_coord[i][j][k-1][Y])+",\"z\":"+to_string(points_coord[i][j][k-1][Z])+"}},";
                } 

                //links && distances under level
                if(i>0)
                {
                    if(j>0 && k>0) //nw
                    {
                        out_link_pddl += "    (link " + points[i][j][k] + " " + points[i-1][j-1][k-1] + ")\n";
                        out_distance_pddl += "    (= (distance " + points[i][j][k] + " " + points[i-1][j-1][k-1] + ") " + to_string(sqrt(2*pow(dst_pts,2)+pow(points_coord[i][j][k][Z]-points_coord[i-1][j-1][k-1][Z],2))) + ")\n";
                        out_link_json += "{\"type\": \"drone\",\"from\":{\"x\":"+to_string(points_coord[i][j][k][X])+",\"y\":"+to_string(points_coord[i][j][k][Y])+",\"z\":"+to_string(points_coord[i][j][k][Z])+"},\"to\":{\"x\":"+to_string(points_coord[i-1][j-1][k-1][X])+",\"y\":"+to_string(points_coord[i-1][j-1][k-1][Y])+",\"z\":"+to_string(points_coord[i-1][j-1][k-1][Z])+"}},";
                    } 
                    if(j>0) //nn
                    {
                        out_link_pddl += "    (link " + points[i][j][k] + " " + points[i-1][j-1][k] + ")\n";
                        out_distance_pddl += "    (= (distance " + points[i][j][k] + " " + points[i-1][j-1][k] + ") " + to_string(sqrt(pow(dst_pts,2)+pow(points_coord[i][j][k][Z]-points_coord[i-1][j-1][k][Z],2))) + ")\n";
                        out_link_json += "{\"type\": \"drone\",\"from\":{\"x\":"+to_string(points_coord[i][j][k][X])+",\"y\":"+to_string(points_coord[i][j][k][Y])+",\"z\":"+to_string(points_coord[i][j][k][Z])+"},\"to\":{\"x\":"+to_string(points_coord[i-1][j-1][k][X])+",\"y\":"+to_string(points_coord[i-1][j-1][k][Y])+",\"z\":"+to_string(points_coord[i-1][j-1][k][Z])+"}},";
                    } 
                    if(j>0 && k<w-1) //ne
                    {
                        out_link_pddl += "    (link " + points[i][j][k] + " " + points[i-1][j-1][k+1] + ")\n";
                        out_distance_pddl += "    (= (distance " + points[i][j][k] + " " + points[i-1][j-1][k+1] + ") " + to_string(sqrt(2*pow(dst_pts,2)+pow(points_coord[i][j][k][Z]-points_coord[i-1][j-1][k+1][Z],2))) + ")\n";
                        out_link_json += "{\"type\": \"drone\",\"from\":{\"x\":"+to_string(points_coord[i][j][k][X])+",\"y\":"+to_string(points_coord[i][j][k][Y])+",\"z\":"+to_string(points_coord[i][j][k][Z])+"},\"to\":{\"x\":"+to_string(points_coord[i-1][j-1][k+1][X])+",\"y\":"+to_string(points_coord[i-1][j-1][k+1][Y])+",\"z\":"+to_string(points_coord[i-1][j-1][k+1][Z])+"}},";
                    } 
                    if(k<w-1) //ee
                    {
                        out_link_pddl += "    (link " + points[i][j][k] + " " + points[i-1][j][k+1] + ")\n";
                        out_distance_pddl += "    (= (distance " + points[i][j][k] + " " + points[i-1][j][k+1] + ") " + to_string(sqrt(pow(dst_pts,2)+pow(points_coord[i][j][k][Z]-points_coord[i-1][j][k+1][Z],2))) + ")\n";
                        out_link_json += "{\"type\": \"drone\",\"from\":{\"x\":"+to_string(points_coord[i][j][k][X])+",\"y\":"+to_string(points_coord[i][j][k][Y])+",\"z\":"+to_string(points_coord[i][j][k][Z])+"},\"to\":{\"x\":"+to_string(points_coord[i-1][j][k+1][X])+",\"y\":"+to_string(points_coord[i-1][j][k+1][Y])+",\"z\":"+to_string(points_coord[i-1][j][k+1][Z])+"}},";
                    } 
                    if(j<h-1 && k<w-1) //se
                    {
                        out_link_pddl += "    (link " + points[i][j][k] + " " + points[i-1][j+1][k+1] + ")\n";
                        out_distance_pddl += "    (= (distance " + points[i][j][k] + " " + points[i-1][j+1][k+1] + ") " + to_string(sqrt(2*pow(dst_pts,2)+pow(points_coord[i][j][k][Z]-points_coord[i-1][j+1][k+1][Z],2))) + ")\n";
                        out_link_json += "{\"type\": \"drone\",\"from\":{\"x\":"+to_string(points_coord[i][j][k][X])+",\"y\":"+to_string(points_coord[i][j][k][Y])+",\"z\":"+to_string(points_coord[i][j][k][Z])+"},\"to\":{\"x\":"+to_string(points_coord[i-1][j+1][k+1][X])+",\"y\":"+to_string(points_coord[i-1][j+1][k+1][Y])+",\"z\":"+to_string(points_coord[i-1][j+1][k+1][Z])+"}},";
                    } 
                    if(j<h-1) //ss
                    {
                        out_link_pddl += "    (link " + points[i][j][k] + " " + points[i-1][j+1][k] + ")\n";
                        out_distance_pddl += "    (= (distance " + points[i][j][k] + " " + points[i-1][j+1][k] + ") " + to_string(sqrt(pow(dst_pts,2)+pow(points_coord[i][j][k][Z]-points_coord[i-1][j+1][k][Z],2))) + ")\n";
                        out_link_json += "{\"type\": \"drone\",\"from\":{\"x\":"+to_string(points_coord[i][j][k][X])+",\"y\":"+to_string(points_coord[i][j][k][Y])+",\"z\":"+to_string(points_coord[i][j][k][Z])+"},\"to\":{\"x\":"+to_string(points_coord[i-1][j+1][k][X])+",\"y\":"+to_string(points_coord[i-1][j+1][k][Y])+",\"z\":"+to_string(points_coord[i-1][j+1][k][Z])+"}},";
                    } 
                    if(j<h-1 && k>0) //sw
                    {
                        out_link_pddl += "    (link " + points[i][j][k] + " " + points[i-1][j+1][k-1] + ")\n";
                        out_distance_pddl += "    (= (distance " + points[i][j][k] + " " + points[i-1][j+1][k-1] + ") " + to_string(sqrt(2*pow(dst_pts,2)+pow(points_coord[i][j][k][Z]-points_coord[i-1][j+1][k-1][Z],2))) + ")\n";
                        out_link_json += "{\"type\": \"drone\",\"from\":{\"x\":"+to_string(points_coord[i][j][k][X])+",\"y\":"+to_string(points_coord[i][j][k][Y])+",\"z\":"+to_string(points_coord[i][j][k][Z])+"},\"to\":{\"x\":"+to_string(points_coord[i-1][j+1][k-1][X])+",\"y\":"+to_string(points_coord[i-1][j+1][k-1][Y])+",\"z\":"+to_string(points_coord[i-1][j+1][k-1][Z])+"}},";
                    } 
                    if(k>0) //ww
                    {
                        out_link_pddl += "    (link " + points[i][j][k] + " " + points[i-1][j][k-1] + ")\n";
                        out_distance_pddl += "    (= (distance " + points[i][j][k] + " " + points[i-1][j][k-1] + ") " + to_string(sqrt(pow(dst_pts,2)+pow(points_coord[i][j][k][Z]-points_coord[i-1][j][k-1][Z],2))) + ")\n";
                        out_link_json += "{\"type\": \"drone\",\"from\":{\"x\":"+to_string(points_coord[i][j][k][X])+",\"y\":"+to_string(points_coord[i][j][k][Y])+",\"z\":"+to_string(points_coord[i][j][k][Z])+"},\"to\":{\"x\":"+to_string(points_coord[i-1][j][k-1][X])+",\"y\":"+to_string(points_coord[i-1][j][k-1][Y])+",\"z\":"+to_string(points_coord[i-1][j][k-1][Z])+"}},";
                    }
                    //center
                    out_link_pddl += "    (link " + points[i][j][k] + " " + points[i-1][j][k] + ")\n";
                    out_distance_pddl += "    (= (distance " + points[i][j][k] + " " + points[i-1][j][k] + ") " + to_string(abs(points_coord[i][j][k][Z]-points_coord[i-1][j][k][Z])) + ")\n";
                    out_link_json += "{\"type\": \"drone\",\"from\":{\"x\":"+to_string(points_coord[i][j][k][X])+",\"y\":"+to_string(points_coord[i][j][k][Y])+",\"z\":"+to_string(points_coord[i][j][k][Z])+"},\"to\":{\"x\":"+to_string(points_coord[i-1][j][k][X])+",\"y\":"+to_string(points_coord[i-1][j][k][Y])+",\"z\":"+to_string(points_coord[i-1][j][k][Z])+"}},";
                }

                //links && distances above level
                if(i<l-1)
                {
                    if(j>0 && k>0) //nw
                    {
                        out_link_pddl += "    (link " + points[i][j][k] + " " + points[i+1][j-1][k-1] + ")\n";
                        out_distance_pddl += "    (= (distance " + points[i][j][k] + " " + points[i+1][j-1][k-1] + ") " + to_string(sqrt(2*pow(dst_pts,2)+pow(points_coord[i][j][k][Z]-points_coord[i+1][j-1][k-1][Z],2))) + ")\n";
                        out_link_json += "{\"type\": \"drone\",\"from\":{\"x\":"+to_string(points_coord[i][j][k][X])+",\"y\":"+to_string(points_coord[i][j][k][Y])+",\"z\":"+to_string(points_coord[i][j][k][Z])+"},\"to\":{\"x\":"+to_string(points_coord[i+1][j-1][k-1][X])+",\"y\":"+to_string(points_coord[i+1][j-1][k-1][Y])+",\"z\":"+to_string(points_coord[i+1][j-1][k-1][Z])+"}},";
                    } 
                    if(j>0) //nn
                    {
                        out_link_pddl += "    (link " + points[i][j][k] + " " + points[i+1][j-1][k] + ")\n";
                        out_distance_pddl += "    (= (distance " + points[i][j][k] + " " + points[i+1][j-1][k] + ") " + to_string(sqrt(pow(dst_pts,2)+pow(points_coord[i][j][k][Z]-points_coord[i+1][j-1][k][Z],2))) + ")\n";
                        out_link_json += "{\"type\": \"drone\",\"from\":{\"x\":"+to_string(points_coord[i][j][k][X])+",\"y\":"+to_string(points_coord[i][j][k][Y])+",\"z\":"+to_string(points_coord[i][j][k][Z])+"},\"to\":{\"x\":"+to_string(points_coord[i+1][j-1][k][X])+",\"y\":"+to_string(points_coord[i+1][j-1][k][Y])+",\"z\":"+to_string(points_coord[i+1][j-1][k][Z])+"}},";
                    } 
                    if(j>0 && k<w-1) //ne
                    {
                        out_link_pddl += "    (link " + points[i][j][k] + " " + points[i+1][j-1][k+1] + ")\n";
                        out_distance_pddl += "    (= (distance " + points[i][j][k] + " " + points[i+1][j-1][k+1] + ") " + to_string(sqrt(2*pow(dst_pts,2)+pow(points_coord[i][j][k][Z]-points_coord[i+1][j-1][k+1][Z],2))) + ")\n";
                        out_link_json += "{\"type\": \"drone\",\"from\":{\"x\":"+to_string(points_coord[i][j][k][X])+",\"y\":"+to_string(points_coord[i][j][k][Y])+",\"z\":"+to_string(points_coord[i][j][k][Z])+"},\"to\":{\"x\":"+to_string(points_coord[i+1][j-1][k+1][X])+",\"y\":"+to_string(points_coord[i+1][j-1][k+1][Y])+",\"z\":"+to_string(points_coord[i+1][j-1][k+1][Z])+"}},";
                    } 
                    if(k<w-1) //ee
                    {
                        out_link_pddl += "    (link " + points[i][j][k] + " " + points[i+1][j][k+1] + ")\n";
                        out_distance_pddl += "    (= (distance " + points[i][j][k] + " " + points[i+1][j][k+1] + ") " + to_string(sqrt(pow(dst_pts,2)+pow(points_coord[i][j][k][Z]-points_coord[i+1][j][k+1][Z],2))) + ")\n";
                        out_link_json += "{\"type\": \"drone\",\"from\":{\"x\":"+to_string(points_coord[i][j][k][X])+",\"y\":"+to_string(points_coord[i][j][k][Y])+",\"z\":"+to_string(points_coord[i][j][k][Z])+"},\"to\":{\"x\":"+to_string(points_coord[i+1][j][k+1][X])+",\"y\":"+to_string(points_coord[i+1][j][k+1][Y])+",\"z\":"+to_string(points_coord[i+1][j][k+1][Z])+"}},";
                    } 
                    if(j<h-1 && k<w-1) //se
                    {
                        out_link_pddl += "    (link " + points[i][j][k] + " " + points[i+1][j+1][k+1] + ")\n";
                        out_distance_pddl += "    (= (distance " + points[i][j][k] + " " + points[i+1][j+1][k+1] + ") " + to_string(sqrt(2*pow(dst_pts,2)+pow(points_coord[i][j][k][Z]-points_coord[i+1][j+1][k+1][Z],2))) + ")\n";
                        out_link_json += "{\"type\": \"drone\",\"from\":{\"x\":"+to_string(points_coord[i][j][k][X])+",\"y\":"+to_string(points_coord[i][j][k][Y])+",\"z\":"+to_string(points_coord[i][j][k][Z])+"},\"to\":{\"x\":"+to_string(points_coord[i+1][j+1][k+1][X])+",\"y\":"+to_string(points_coord[i+1][j+1][k+1][Y])+",\"z\":"+to_string(points_coord[i+1][j+1][k+1][Z])+"}},";
                    } 
                    if(j<h-1) //ss
                    {
                        out_link_pddl += "    (link " + points[i][j][k] + " " + points[i+1][j+1][k] + ")\n";
                        out_distance_pddl += "    (= (distance " + points[i][j][k] + " " + points[i+1][j+1][k] + ") " + to_string(sqrt(pow(dst_pts,2)+pow(points_coord[i][j][k][Z]-points_coord[i+1][j+1][k][Z],2))) + ")\n";
                        out_link_json += "{\"type\": \"drone\",\"from\":{\"x\":"+to_string(points_coord[i][j][k][X])+",\"y\":"+to_string(points_coord[i][j][k][Y])+",\"z\":"+to_string(points_coord[i][j][k][Z])+"},\"to\":{\"x\":"+to_string(points_coord[i+1][j+1][k][X])+",\"y\":"+to_string(points_coord[i+1][j+1][k][Y])+",\"z\":"+to_string(points_coord[i+1][j+1][k][Z])+"}},";
                    } 
                    if(j<h-1 && k>0) //sw
                    {
                        out_link_pddl += "    (link " + points[i][j][k] + " " + points[i+1][j+1][k-1] + ")\n";
                        out_distance_pddl += "    (= (distance " + points[i][j][k] + " " + points[i+1][j+1][k-1] + ") " + to_string(sqrt(2*pow(dst_pts,2)+pow(points_coord[i][j][k][Z]-points_coord[i+1][j+1][k-1][Z],2))) + ")\n";
                        out_link_json += "{\"type\": \"drone\",\"from\":{\"x\":"+to_string(points_coord[i][j][k][X])+",\"y\":"+to_string(points_coord[i][j][k][Y])+",\"z\":"+to_string(points_coord[i][j][k][Z])+"},\"to\":{\"x\":"+to_string(points_coord[i+1][j+1][k-1][X])+",\"y\":"+to_string(points_coord[i+1][j+1][k-1][Y])+",\"z\":"+to_string(points_coord[i+1][j+1][k-1][Z])+"}},";
                    } 
                    if(k>0) //ww
                    {
                        out_link_pddl += "    (link " + points[i][j][k] + " " + points[i+1][j][k-1] + ")\n";
                        out_distance_pddl += "    (= (distance " + points[i][j][k] + " " + points[i+1][j][k-1] + ") " + to_string(sqrt(pow(dst_pts,2)+pow(points_coord[i][j][k][Z]-points_coord[i+1][j][k-1][Z],2))) + ")\n";
                        out_link_json += "{\"type\": \"drone\",\"from\":{\"x\":"+to_string(points_coord[i][j][k][X])+",\"y\":"+to_string(points_coord[i][j][k][Y])+",\"z\":"+to_string(points_coord[i][j][k][Z])+"},\"to\":{\"x\":"+to_string(points_coord[i+1][j][k-1][X])+",\"y\":"+to_string(points_coord[i+1][j][k-1][Y])+",\"z\":"+to_string(points_coord[i+1][j][k-1][Z])+"}},";
                    }
                    //center
                    out_link_pddl += "    (link " + points[i][j][k] + " " + points[i+1][j][k] + ")\n";
                    out_distance_pddl += "    (= (distance " + points[i][j][k] + " " + points[i+1][j][k] + ") " + to_string(abs(points_coord[i][j][k][Z]-points_coord[i+1][j][k][Z])) + ")\n";
                    out_link_json += "{\"type\": \"drone\",\"from\":{\"x\":"+to_string(points_coord[i][j][k][X])+",\"y\":"+to_string(points_coord[i][j][k][Y])+",\"z\":"+to_string(points_coord[i][j][k][Z])+"},\"to\":{\"x\":"+to_string(points_coord[i+1][j][k][X])+",\"y\":"+to_string(points_coord[i+1][j][k][Y])+",\"z\":"+to_string(points_coord[i+1][j][k][Z])+"}},";
                }
            }

    //COMPUTES LINKS & DISTANCES AGV
    for(int j=0;j<h;j++)
        for(int k=0;k<w;k++)
        {
            out_list_pddl += points_agv[j][k] + " ";
            out_link_pddl += "    (link " + points[0][j][k] + " " + points_agv[j][k] + ")\n";
            out_distance_pddl += "    (= (distance " + points[0][j][k] + " " + points_agv[j][k] + ") " + to_string(low_lvl) + ")\n";
            out_link_json += "{\"type\": \"drone\",\"from\":{\"x\":"+to_string(points_coord[0][j][k][X])+",\"y\":"+to_string(points_coord[0][j][k][Y])+",\"z\":"+to_string(el[j][k])+"},\"to\":{\"x\":"+to_string(points_coord[0][j][k][X])+",\"y\":"+to_string(points_coord[0][j][k][Y])+",\"z\":"+to_string(points_coord[0][j][k][Z])+"}},";
            if(j>0 && k>0) //nw
                if(abs(el[j][k]-el[j-1][k-1])/(dst_pts*1.4142)<=MAX_INCLINATION)
                {
                    out_link_agv_pddl += "    (link_agv " + points_agv[j][k] + " " + points_agv[j-1][k-1] + ")\n";
                    out_distance_agv_pddl += "    (= (distance_agv " + points_agv[j][k] + " " + points_agv[j-1][k-1] + ") " + to_string(sqrt(2*pow(dst_pts,2)+pow(el[j][k]-el[j-1][k-1],2))) + ")\n";
                    out_link_json += "{\"type\": \"agv\",\"from\":{\"x\":"+to_string(points_coord[0][j][k][X])+",\"y\":"+to_string(points_coord[0][j][k][Y])+",\"z\":"+to_string(el[j][k])+"},\"to\":{\"x\":"+to_string(points_coord[0][j-1][k-1][X])+",\"y\":"+to_string(points_coord[0][j-1][k-1][Y])+",\"z\":"+to_string(el[j-1][k-1])+"}},";
                }
            if(j>0) //nn
                if(abs(el[j][k]-el[j-1][k])/(dst_pts*1.0)<=MAX_INCLINATION)
                {
                    out_link_agv_pddl += "    (link_agv " + points_agv[j][k] + " " + points_agv[j-1][k] + ")\n";
                    out_distance_agv_pddl += "    (= (distance_agv " + points_agv[j][k] + " " + points_agv[j-1][k] + ") " + to_string(sqrt(pow(dst_pts,2)+pow(el[j][k]-el[j-1][k],2))) + ")\n";
                    out_link_json += "{\"type\": \"agv\",\"from\":{\"x\":"+to_string(points_coord[0][j][k][X])+",\"y\":"+to_string(points_coord[0][j][k][Y])+",\"z\":"+to_string(el[j][k])+"},\"to\":{\"x\":"+to_string(points_coord[0][j-1][k][X])+",\"y\":"+to_string(points_coord[0][j-1][k][Y])+",\"z\":"+to_string(el[j-1][k])+"}},";
                }
            if(j>0 && k<w-1) //ne
                if(abs(el[j][k]-el[j-1][k+1])/(dst_pts*1.4142)<=MAX_INCLINATION)
                {
                    out_link_agv_pddl += "    (link_agv " + points_agv[j][k] + " " + points_agv[j-1][k+1] + ")\n";
                    out_distance_agv_pddl += "    (= (distance_agv " + points_agv[j][k] + " " + points_agv[j-1][k+1] + ") " + to_string(sqrt(2*pow(dst_pts,2)+pow(el[j][k]-el[j-1][k+1],2))) + ")\n";
                    out_link_json += "{\"type\": \"agv\",\"from\":{\"x\":"+to_string(points_coord[0][j][k][X])+",\"y\":"+to_string(points_coord[0][j][k][Y])+",\"z\":"+to_string(el[j][k])+"},\"to\":{\"x\":"+to_string(points_coord[0][j-1][k+1][X])+",\"y\":"+to_string(points_coord[0][j-1][k+1][Y])+",\"z\":"+to_string(el[j-1][k+1])+"}},";
                }
            if(k<w-1) //ee
                if(abs(el[j][k]-el[j][k+1])/(dst_pts*1.0)<=MAX_INCLINATION)
                {
                    out_link_agv_pddl += "    (link_agv " + points_agv[j][k] + " " + points_agv[j][k+1] + ")\n";
                    out_distance_agv_pddl += "    (= (distance_agv " + points_agv[j][k] + " " + points_agv[j][k+1] + ") " + to_string(sqrt(pow(dst_pts,2)+pow(el[j][k]-el[j][k+1],2))) + ")\n";
                    out_link_json += "{\"type\": \"agv\",\"from\":{\"x\":"+to_string(points_coord[0][j][k][X])+",\"y\":"+to_string(points_coord[0][j][k][Y])+",\"z\":"+to_string(el[j][k])+"},\"to\":{\"x\":"+to_string(points_coord[0][j][k+1][X])+",\"y\":"+to_string(points_coord[0][j][k+1][Y])+",\"z\":"+to_string(el[j][k+1])+"}},";
                }
            if(j<h-1 && k<w-1) //se
                if(abs(el[j][k]-el[j+1][k+1])/(dst_pts*1.4142)<=MAX_INCLINATION)
                {
                    out_link_agv_pddl += "    (link_agv " + points_agv[j][k] + " " + points_agv[j+1][k+1] + ")\n";
                    out_distance_agv_pddl += "    (= (distance_agv " + points_agv[j][k] + " " + points_agv[j+1][k+1] + ") " + to_string(sqrt(2*pow(dst_pts,2)+pow(el[j][k]-el[j+1][k+1],2))) + ")\n";
                    out_link_json += "{\"type\": \"agv\",\"from\":{\"x\":"+to_string(points_coord[0][j][k][X])+",\"y\":"+to_string(points_coord[0][j][k][Y])+",\"z\":"+to_string(el[j][k])+"},\"to\":{\"x\":"+to_string(points_coord[0][j+1][k+1][X])+",\"y\":"+to_string(points_coord[0][j+1][k+1][Y])+",\"z\":"+to_string(el[j+1][k+1])+"}},";
                }
            if(j<h-1) //ss
                if(abs(el[j][k]-el[j+1][k])/(dst_pts*1.0)<=MAX_INCLINATION)
                {
                    out_link_agv_pddl += "    (link_agv " + points_agv[j][k] + " " + points_agv[j+1][k] + ")\n";
                    out_distance_agv_pddl += "    (= (distance_agv " + points_agv[j][k] + " " + points_agv[j+1][k] + ") " + to_string(sqrt(pow(dst_pts,2)+pow(el[j][k]-el[j+1][k],2))) + ")\n";
                    out_link_json += "{\"type\": \"agv\",\"from\":{\"x\":"+to_string(points_coord[0][j][k][X])+",\"y\":"+to_string(points_coord[0][j][k][Y])+",\"z\":"+to_string(el[j][k])+"},\"to\":{\"x\":"+to_string(points_coord[0][j+1][k][X])+",\"y\":"+to_string(points_coord[0][j+1][k][Y])+",\"z\":"+to_string(el[j+1][k])+"}},";
                }
            if(j<h-1 && k>0) //sw
                if(abs(el[j][k]-el[j+1][k-1])/(dst_pts*1.4142)<=MAX_INCLINATION)
                {
                    out_link_agv_pddl += "    (link_agv " + points_agv[j][k] + " " + points_agv[j+1][k-1] + ")\n";
                    out_distance_agv_pddl += "    (= (distance_agv " + points_agv[j][k] + " " + points_agv[j+1][k-1] + ") " + to_string(sqrt(2*pow(dst_pts,2)+pow(el[j][k]-el[j+1][k-1],2))) + ")\n";
                    out_link_json += "{\"type\": \"agv\",\"from\":{\"x\":"+to_string(points_coord[0][j][k][X])+",\"y\":"+to_string(points_coord[0][j][k][Y])+",\"z\":"+to_string(el[j][k])+"},\"to\":{\"x\":"+to_string(points_coord[0][j+1][k-1][X])+",\"y\":"+to_string(points_coord[0][j+1][k-1][Y])+",\"z\":"+to_string(el[j+1][k-1])+"}},";
                }
            if(k>0) //ww
                if(abs(el[j][k]-el[j][k-1])/(dst_pts*1.0)<=MAX_INCLINATION)
                {
                    out_link_agv_pddl += "    (link_agv " + points_agv[j][k] + " " + points_agv[j][k-1] + ")\n";
                    out_distance_agv_pddl += "    (= (distance_agv " + points_agv[j][k] + " " + points_agv[j][k-1] + ") " + to_string(sqrt(pow(dst_pts,2)+pow(el[j][k]-el[j][k-1],2))) + ")\n";
                    out_link_json += "{\"type\": \"agv\",\"from\":{\"x\":"+to_string(points_coord[0][j][k][X])+",\"y\":"+to_string(points_coord[0][j][k][Y])+",\"z\":"+to_string(el[j][k])+"},\"to\":{\"x\":"+to_string(points_coord[0][j][k-1][X])+",\"y\":"+to_string(points_coord[0][j][k-1][Y])+",\"z\":"+to_string(el[j][k-1])+"}},";
                }
        }

    //WRITE ON PDDL FILE
    if(pddl)
    {
        ofstream pddl_out;
        pddl_out.open(pddl_file);

        pddl_out << "(define (problem planning_map)""\n"
                "  (:domain planner)""\n"
                "  (:objects""\n"
                "     " << out_list_pddl << " - point\n"
                "     d1 d2 - drone\n"
                "  )""\n"
                "  ""\n"
                "  (:init""\n"
                "    (= (cost) 0)""\n";
        if(!return_drones)
            pddl_out << "    (agv_has d1)\n"
                        "    (agv_has d2)\n";
        else
            pddl_out << "    (drone_pos d1 "<<d1_pos<<")\n"
                        "    (drone_pos d2 "<<d2_pos<<")\n";

        pddl_out << "    (agv_pos " << agv_pos << ")""\n\n" << out_empty_pddl << "\n";
        
        pddl_out << out_link_pddl << out_distance_pddl << out_link_agv_pddl << out_distance_agv_pddl;

        pddl_out << "  )""\n"
                "  (:goal""\n";

        if(!return_drones)
        {
            if(ntarget==1)
                pddl_out << "    (picture "<< targets[0] << ")\n";
            else
            {
                pddl_out << "    (and \n";
                for(int i=0; i<ntarget; i++)
                    pddl_out << "      (picture " << targets[i] << ")\n";
                pddl_out << "    )\n";
            }
        }
        else
        {
            pddl_out << "    (and\n"
                        "       (agv_has d1)\n"
                        "       (agv_has d2)\n"
                        "    )\n";
        }
        
        pddl_out << "  )""\n"
                "  (:metric minimize""\n"
                "    (cost)""\n"
                "  )""\n"
                ")""\n";
    }

    //WRITE ON PS2 FILE
    if(links)
    {
        ofstream links_out;
        links_out.open(links_file);
        out_link_json.pop_back();
        links_out << "{\"links\": [" << out_link_json << "]}";
    }

    //WRITE ON JSON FILE
    if(json)
    {
        ofstream json_out;
        json_out.open(json_file);
        json_out << "{\"points\":[" << out_coordinates << "]}";
    }


    return 0;
}