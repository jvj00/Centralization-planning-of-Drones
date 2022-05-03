#include <iostream>
#include <string.h>

#define MAX_HEIGHT 100
#define MAX_WIDTH 100
#define MAX_VISIT_POINTS 100

using namespace std;

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

int main(int argc, char** argv)
{
    int h, w, ntarget=0;
    double cost;
    string points[MAX_HEIGHT][MAX_WIDTH];
    string out_l="",out_d="", out_list_p="";
    string d1_pos="", d2_pos="", d1_dir="", d2_dir="", targets[MAX_VISIT_POINTS], directions[MAX_VISIT_POINTS];

    //PARSING ARGUMENTS
    if(argc>=10 && argc%2==0 && is_number_int(argv[1]) && is_number_int(argv[2]) && is_number(argv[3]))
    {
        h = atoi(argv[1]);
        w = atoi(argv[2]);
        cost = stod(argv[3]);
        d1_pos = argv[4];
        d2_pos = argv[5];
        d1_dir = argv[6];
        d2_dir = argv[7];
        for(int i=8;i<argc && i<MAX_VISIT_POINTS*2+8;i+=2)
        {
            targets[ntarget]=argv[i];
            directions[ntarget]=argv[i+1];
            ntarget++;
        }
    }
    else if(argc==2 && (!((string)argv[1]).compare("--help") || !((string)argv[1]).compare("-h")))
    {
        cout << "This tool generate a 2D map of points, giving height and width in points as input." << endl;
        cout << "Usage: ./map_tool <height> <width> <distance_pts> <drone1_position> <drone2_position> <drone1_direction> <drone2_direction> <target1 direction1 target2 direction2 ...>" << endl;
        exit(0);
    }
    else
    {
        cerr << "Wrong usage of this tool. Type --help or -h to see the guide" << endl; 
        exit(1);
    }

    //MAIN PROGRAM
    for(int i=0;i<h;i++)
        for(int j=0;j<w;j++)
            points[i][j] = "p" + to_string((i*w)+j);

    for(int i=0;i<h;i++)
        for(int j=0;j<w;j++)
        {
            //points
            out_list_p += points[i][j] + " ";

            //links && distances
            if(i>0 && j>0) //nw
            {
                out_l += "    (link " + points[i][j] + " " + points[i-1][j-1] + ")\n";
                out_d += "    (= (distance " + points[i][j] + " " + points[i-1][j-1] + ") " + to_string(cost * 1.4142) + ")\n";
            } 
            if(i>0) //nn
            {
                out_l += "    (link " + points[i][j] + " " + points[i-1][j] + ")\n";
                out_d += "    (= (distance " + points[i][j] + " " + points[i-1][j] + ") " + to_string(cost * 1.0) + ")\n";
            } 
            if(i>0 && j<w-1) //ne
            {
                out_l += "    (link " + points[i][j] + " " + points[i-1][j+1] + ")\n";
                out_d += "    (= (distance " + points[i][j] + " " + points[i-1][j+1] + ") " + to_string(cost * 1.4142) + ")\n";
            } 
            if(j<w-1) //ee
            {
                out_l += "    (link " + points[i][j] + " " + points[i][j+1] + ")\n";
                out_d += "    (= (distance " + points[i][j] + " " + points[i][j+1] + ") " + to_string(cost * 1.0) + ")\n";
            } 
            if(i<h-1 && j<w-1) //se
            {
                out_l += "    (link " + points[i][j] + " " + points[i+1][j+1] + ")\n";
                out_d += "    (= (distance " + points[i][j] + " " + points[i+1][j+1] + ") " + to_string(cost * 1.4142) + ")\n";
            } 
            if(i<h-1) //ss
            {
                out_l += "    (link " + points[i][j] + " " + points[i+1][j] + ")\n";
                out_d += "    (= (distance " + points[i][j] + " " + points[i+1][j] + ") " + to_string(cost * 1.0) + ")\n";
            } 
            if(i<h-1 && j>0) //sw
            {
                out_l += "    (link " + points[i][j] + " " + points[i+1][j-1] + ")\n";
                out_d += "    (= (distance " + points[i][j] + " " + points[i+1][j-1] + ") " + to_string(cost * 1.4142) + ")\n";
            } 
            if(j>0) //ww
            {
                out_l += "    (link " + points[i][j] + " " + points[i][j-1] + ")\n";
                out_d += "    (= (distance " + points[i][j] + " " + points[i][j-1] + ") " + to_string(cost * 1.0) + ")\n";
            } 
        }            

    cout << "(define (problem planning_map)""\n"
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
            "    (drone_dir d1 " << d2_dir << ")""\n"
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
            cout << "    (rotation_point " << targets[i] << ")\n";
    cout << "\n";
      
    cout << out_l << out_d;

    cout << "  )""\n"
            "  (:goal""\n";

    if(ntarget==1)
        cout << "    (picture "<< targets[0] << " " << directions[0] << ")\n";
    else
    {
        cout << "    (and \n";
        for(int i=0; i<ntarget; i++)
            cout << "      (picture " << targets[i] << " " << directions[i] << ")\n";
        cout << "    )\n";
    }
    
    cout << "  )""\n"
            "  (:metric minimize""\n"
            "    (+ (total-time) (cost))""\n"
            "  )""\n"
            ")""\n";

    return 0;
}