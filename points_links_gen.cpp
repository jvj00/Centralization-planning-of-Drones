#include <iostream>
#include <fstream>
#include <algorithm>
#include <string.h>
#include <math.h>

#define MAX_HEIGHT 100
#define MAX_WIDTH 100
#define MAX_VISIT_POINTS 100
#define MAX_LEVELS 5
#define LEVEL_DST 30

using namespace std;


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

int main(int argc, char** argv)
{
    int h, w, l, ntarget=0, index_arguments=1;
    bool pddl=false, ps2=false, json=false;
    char *pddl_file, *ps2_file, *json_file;
    double dst_pts, dst_lvl, low_lvl;
    string points[MAX_LEVELS][MAX_HEIGHT][MAX_WIDTH];
    string out_l="",out_d="", out_list_p="", out_coordinates="";
    string d1_pos="", d2_pos="", d1_dir="", d2_dir="", targets[MAX_VISIT_POINTS], directions[MAX_VISIT_POINTS];

    //PARSING ARGUMENTS
    //OUTPUTS
    if(cmdOptionExists(argv, argv+argc, "--pddl"))
    {
        pddl=true;
        pddl_file=getCmdOption(argv, argv + argc, "--pddl");
        index_arguments+=2;
        if(!pddl_file) common_error();
    }
    if(cmdOptionExists(argv, argv+argc, "--ps2"))
    {
        ps2=true;
        ps2_file=getCmdOption(argv, argv + argc, "--ps2");
        index_arguments+=2;
        if(!ps2_file) common_error();
    }
    if(cmdOptionExists(argv, argv+argc, "--json"))
    {
        json=true;
        json_file=getCmdOption(argv, argv + argc, "--json");
        index_arguments+=2;
        if(!json_file) common_error();
    }
    //PARAMETERS
    if(argc>=12+index_arguments && argc%2==1 && is_number_int(argv[index_arguments]) && is_number_int(argv[1+index_arguments]) && is_number_int(argv[2+index_arguments]) && is_number(argv[3+index_arguments]) && is_number(argv[4+index_arguments]) && is_number(argv[5+index_arguments]))
    {
        h = atoi(argv[index_arguments++]);
        w = atoi(argv[index_arguments++]);
        l = atoi(argv[index_arguments++]);
        dst_pts = stod(argv[index_arguments++]);
        dst_lvl = stod(argv[index_arguments++]);
        low_lvl = stod(argv[index_arguments++]);
        d1_pos = argv[index_arguments++];
        d1_dir = argv[index_arguments++];
        d2_pos = argv[index_arguments++];
        d2_dir = argv[index_arguments++];
        for(int i=index_arguments;i<argc && i<MAX_VISIT_POINTS*2+10;i+=2)
        {
            targets[ntarget]=argv[i];
            directions[ntarget]=argv[i+1];
            ntarget++;
        }
    }
    //HELP & ERRORS
    else if(argc==2 && (!((string)argv[1]).compare("--help") || !((string)argv[1]).compare("-h")))
    {
        cout << "This tool generate a 2D map of points, giving height and width in points as input." << endl << endl;
        cout << "Usage: ./map_tool [options] <height> <width> <levels> <distance_pts> <distance_lvl> <lowest_lvl> <drone1_position> <drone1_direction> <drone2_position> <drone2_direction> <target1 direction1 target2 direction2 ...>" << endl;
        cout << endl;
        cout << "Parameters:" << endl;
        cout << "   <height(pts)> height 2D (y) in points of the map" << endl;
        cout << "   <width(pts)> width 2D (x) in points of the map" << endl;
        cout << "   <levels> # of levels (z) of the map" << endl;
        cout << "   <distance_pts> distance in meters between points of the same level" << endl;
        cout << "   <distance_lvl> distance in meters between levels" << endl;
        cout << "   <lowest_lvl> heigth in meters of the lowest level" << endl;
        cout << "   <drone1_position> <drone1_direction> position (name of point) and direction (name of direction) of the first drone. E.g.: p23 dir45" << endl;
        cout << "   <drone2_position> <drone2_direction> same, but with second drone" << endl;
        cout << "   <target1 direction1 target2 direction2 ...> list of target position and direction of the planning. E.g.: p45 dir180 p89 dir225" << endl;
        cout << endl;
        cout << "Options:" << endl;
        cout << "   --pddl <path_to_out_file>          generates a pddl problem as output" << endl;
        cout << "   --ps2 <path_to_out_file>           generates a plansys2 problem as output" << endl;
        cout << "   --json <path_to_out_file>          generates a json file with coordinates as secondary output" << endl;
        exit(0);
    }
    else
    {
        common_error();
    }

    //MAIN PROGRAM
    for(int i=0;i<l;i++)
        for(int j=0;j<h;j++)
            for(int k=0;k<w;k++)
            {
                points[i][j][k] = "p" + to_string((i*w*h)+(j*w)+k);
                out_coordinates += "{\"name\":\"" + points[i][j][k] + "\",\"x\":" + to_string(k*dst_pts) + ",\"y\":" + to_string(j*dst_pts) + ",\"z\":" + to_string(i*dst_lvl+low_lvl) + "}";
                if(i!=l-1 || j!=h-1 || k!=w-1)
                    out_coordinates += ",";
            }

    for(int i=0;i<l;i++)
        for(int j=0;j<h;j++)
            for(int k=0;k<w;k++)
            {
                //points
                out_list_p += points[i][j][k] + " ";

                //links && distances in the level
                if(j>0 && k>0) //nw
                {
                    out_l += "    (link " + points[i][j][k] + " " + points[i][j-1][k-1] + ")\n";
                    out_d += "    (= (distance " + points[i][j][k] + " " + points[i][j-1][k-1] + ") " + to_string(dst_pts * 1.4142) + ")\n";
                } 
                if(j>0) //nn
                {
                    out_l += "    (link " + points[i][j][k] + " " + points[i][j-1][k] + ")\n";
                    out_d += "    (= (distance " + points[i][j][k] + " " + points[i][j-1][k] + ") " + to_string(dst_pts * 1.0) + ")\n";
                } 
                if(j>0 && k<w-1) //ne
                {
                    out_l += "    (link " + points[i][j][k] + " " + points[i][j-1][k+1] + ")\n";
                    out_d += "    (= (distance " + points[i][j][k] + " " + points[i][j-1][k+1] + ") " + to_string(dst_pts * 1.4142) + ")\n";
                } 
                if(k<w-1) //ee
                {
                    out_l += "    (link " + points[i][j][k] + " " + points[i][j][k+1] + ")\n";
                    out_d += "    (= (distance " + points[i][j][k] + " " + points[i][j][k+1] + ") " + to_string(dst_pts * 1.0) + ")\n";
                } 
                if(j<h-1 && k<w-1) //se
                {
                    out_l += "    (link " + points[i][j][k] + " " + points[i][j+1][k+1] + ")\n";
                    out_d += "    (= (distance " + points[i][j][k] + " " + points[i][j+1][k+1] + ") " + to_string(dst_pts * 1.4142) + ")\n";
                } 
                if(j<h-1) //ss
                {
                    out_l += "    (link " + points[i][j][k] + " " + points[i][j+1][k] + ")\n";
                    out_d += "    (= (distance " + points[i][j][k] + " " + points[i][j+1][k] + ") " + to_string(dst_pts * 1.0) + ")\n";
                } 
                if(j<h-1 && k>0) //sw
                {
                    out_l += "    (link " + points[i][j][k] + " " + points[i][j+1][k-1] + ")\n";
                    out_d += "    (= (distance " + points[i][j][k] + " " + points[i][j+1][k-1] + ") " + to_string(dst_pts * 1.4142) + ")\n";
                } 
                if(k>0) //ww
                {
                    out_l += "    (link " + points[i][j][k] + " " + points[i][j][k-1] + ")\n";
                    out_d += "    (= (distance " + points[i][j][k] + " " + points[i][j][k-1] + ") " + to_string(dst_pts * 1.0) + ")\n";
                } 

                //links && distances under level
                if(i>0)
                {
                    if(j>0 && k>0) //nw
                    {
                        out_l += "    (link " + points[i][j][k] + " " + points[i-1][j-1][k-1] + ")\n";
                        out_d += "    (= (distance " + points[i][j][k] + " " + points[i-1][j-1][k-1] + ") " + to_string(sqrt(2*dst_pts*dst_pts+dst_lvl*dst_lvl)) + ")\n";
                    } 
                    if(j>0) //nn
                    {
                        out_l += "    (link " + points[i][j][k] + " " + points[i-1][j-1][k] + ")\n";
                        out_d += "    (= (distance " + points[i][j][k] + " " + points[i-1][j-1][k] + ") " + to_string(sqrt(dst_pts*dst_pts+dst_lvl*dst_lvl)) + ")\n";
                    } 
                    if(j>0 && k<w-1) //ne
                    {
                        out_l += "    (link " + points[i][j][k] + " " + points[i-1][j-1][k+1] + ")\n";
                        out_d += "    (= (distance " + points[i][j][k] + " " + points[i-1][j-1][k+1] + ") " + to_string(sqrt(2*dst_pts*dst_pts+dst_lvl*dst_lvl)) + ")\n";
                    } 
                    if(k<w-1) //ee
                    {
                        out_l += "    (link " + points[i][j][k] + " " + points[i-1][j][k+1] + ")\n";
                        out_d += "    (= (distance " + points[i][j][k] + " " + points[i-1][j][k+1] + ") " + to_string(sqrt(dst_pts*dst_pts+dst_lvl*dst_lvl)) + ")\n";
                    } 
                    if(j<h-1 && k<w-1) //se
                    {
                        out_l += "    (link " + points[i][j][k] + " " + points[i-1][j+1][k+1] + ")\n";
                        out_d += "    (= (distance " + points[i][j][k] + " " + points[i-1][j+1][k+1] + ") " + to_string(sqrt(2*dst_pts*dst_pts+dst_lvl*dst_lvl)) + ")\n";
                    } 
                    if(j<h-1) //ss
                    {
                        out_l += "    (link " + points[i][j][k] + " " + points[i-1][j+1][k] + ")\n";
                        out_d += "    (= (distance " + points[i][j][k] + " " + points[i-1][j+1][k] + ") " + to_string(sqrt(dst_pts*dst_pts+dst_lvl*dst_lvl)) + ")\n";
                    } 
                    if(j<h-1 && k>0) //sw
                    {
                        out_l += "    (link " + points[i][j][k] + " " + points[i-1][j+1][k-1] + ")\n";
                        out_d += "    (= (distance " + points[i][j][k] + " " + points[i-1][j+1][k-1] + ") " + to_string(sqrt(2*dst_pts*dst_pts+dst_lvl*dst_lvl)) + ")\n";
                    } 
                    if(k>0) //ww
                    {
                        out_l += "    (link " + points[i][j][k] + " " + points[i-1][j][k-1] + ")\n";
                        out_d += "    (= (distance " + points[i][j][k] + " " + points[i-1][j][k-1] + ") " + to_string(sqrt(dst_pts*dst_pts+dst_lvl*dst_lvl)) + ")\n";
                    }
                    //centered
                    out_l += "    (link " + points[i][j][k] + " " + points[i-1][j][k] + ")\n";
                    out_d += "    (= (distance " + points[i][j][k] + " " + points[i-1][j][k] + ") " + to_string(dst_lvl) + ")\n";
                }

                //links && distances above level
                if(i<l-1)
                {
                    if(j>0 && k>0) //nw
                    {
                        out_l += "    (link " + points[i][j][k] + " " + points[i+1][j-1][k-1] + ")\n";
                        out_d += "    (= (distance " + points[i][j][k] + " " + points[i+1][j-1][k-1] + ") " + to_string(sqrt(2*dst_pts*dst_pts+dst_lvl*dst_lvl)) + ")\n";
                    } 
                    if(j>0) //nn
                    {
                        out_l += "    (link " + points[i][j][k] + " " + points[i+1][j-1][k] + ")\n";
                        out_d += "    (= (distance " + points[i][j][k] + " " + points[i+1][j-1][k] + ") " + to_string(sqrt(dst_pts*dst_pts+dst_lvl*dst_lvl)) + ")\n";
                    } 
                    if(j>0 && k<w-1) //ne
                    {
                        out_l += "    (link " + points[i][j][k] + " " + points[i+1][j-1][k+1] + ")\n";
                        out_d += "    (= (distance " + points[i][j][k] + " " + points[i+1][j-1][k+1] + ") " + to_string(sqrt(2*dst_pts*dst_pts+dst_lvl*dst_lvl)) + ")\n";
                    } 
                    if(k<w-1) //ee
                    {
                        out_l += "    (link " + points[i][j][k] + " " + points[i+1][j][k+1] + ")\n";
                        out_d += "    (= (distance " + points[i][j][k] + " " + points[i+1][j][k+1] + ") " + to_string(sqrt(dst_pts*dst_pts+dst_lvl*dst_lvl)) + ")\n";
                    } 
                    if(j<h-1 && k<w-1) //se
                    {
                        out_l += "    (link " + points[i][j][k] + " " + points[i+1][j+1][k+1] + ")\n";
                        out_d += "    (= (distance " + points[i][j][k] + " " + points[i+1][j+1][k+1] + ") " + to_string(sqrt(2*dst_pts*dst_pts+dst_lvl*dst_lvl)) + ")\n";
                    } 
                    if(j<h-1) //ss
                    {
                        out_l += "    (link " + points[i][j][k] + " " + points[i+1][j+1][k] + ")\n";
                        out_d += "    (= (distance " + points[i][j][k] + " " + points[i+1][j+1][k] + ") " + to_string(sqrt(dst_pts*dst_pts+dst_lvl*dst_lvl)) + ")\n";
                    } 
                    if(j<h-1 && k>0) //sw
                    {
                        out_l += "    (link " + points[i][j][k] + " " + points[i+1][j+1][k-1] + ")\n";
                        out_d += "    (= (distance " + points[i][j][k] + " " + points[i+1][j+1][k-1] + ") " + to_string(sqrt(2*dst_pts*dst_pts+dst_lvl*dst_lvl)) + ")\n";
                    } 
                    if(k>0) //ww
                    {
                        out_l += "    (link " + points[i][j][k] + " " + points[i+1][j][k-1] + ")\n";
                        out_d += "    (= (distance " + points[i][j][k] + " " + points[i+1][j][k-1] + ") " + to_string(sqrt(dst_pts*dst_pts+dst_lvl*dst_lvl)) + ")\n";
                    }
                    //centered
                    out_l += "    (link " + points[i][j][k] + " " + points[i+1][j][k] + ")\n";
                    out_d += "    (= (distance " + points[i][j][k] + " " + points[i+1][j][k] + ") " + to_string(dst_lvl) + ")\n";
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
                "     " << out_list_p << " - point\n"
                "     d1 d2 - drone\n"
                "     dir0 dir45 dir90 dir135 dir180 dir225 dir270 dir315 - direction\n"
                "  )""\n"
                "  ""\n"
                "  (:init""\n"
                "    (= (cost) 0)""\n"
                "    (drone_pos d1 " << d1_pos << ")""\n"
                "    (drone_pos d2 " << d2_pos << ")""\n"
                "    (drone_dir d1 " << d1_dir << ")""\n"
                "    (drone_dir d2 " << d2_dir << ")""\n\n"
                "    (rotation dir0 dir45)""\n"
                "    (rotation dir45 dir0)""\n"
                "    (rotation dir45 dir90)""\n"
                "    (rotation dir90 dir45)""\n"
                "    (rotation dir90 dir135)""\n"
                "    (rotation dir135 dir90)""\n"
                "    (rotation dir135 dir180)""\n"
                "    (rotation dir180 dir135)""\n"
                "    (rotation dir180 dir225)""\n"
                "    (rotation dir225 dir180)""\n"
                "    (rotation dir225 dir270)""\n"
                "    (rotation dir270 dir225)""\n"
                "    (rotation dir270 dir315)""\n"
                "    (rotation dir315 dir270)""\n"
                "    (rotation dir315 dir0)""\n"
                "    (rotation dir0 dir315)""\n\n";

        for(int i=0; i<ntarget; i++)
                pddl_out << "    (rotation_point " << targets[i] << ")\n";
        pddl_out << "\n";
        
        pddl_out << out_l << out_d;

        pddl_out << "  )""\n"
                "  (:goal""\n";

        if(ntarget==1)
            pddl_out << "    (picture "<< targets[0] << " " << directions[0] << ")\n";
        else
        {
            pddl_out << "    (and \n";
            for(int i=0; i<ntarget; i++)
                pddl_out << "      (picture " << targets[i] << " " << directions[i] << ")\n";
            pddl_out << "    )\n";
        }
        
        pddl_out << "  )""\n"
                "  (:metric minimize""\n"
                "    (cost)""\n"
                "  )""\n"
                ")""\n";
    }

    //WRITE ON PS2 FILE
    if(ps2)
    {
        ofstream ps2_out;
        ps2_out.open(ps2_file);
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