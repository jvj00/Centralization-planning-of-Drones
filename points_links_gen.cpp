#include <iostream>
#include <string.h>
#include <math.h>

#define MAX_HEIGHT 100
#define MAX_WIDTH 100
#define MAX_VISIT_POINTS 100
#define MAX_LEVELS 5
#define LEVEL_DST 30

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
    int h, w, l, ntarget=0;
    double dst_pts, dst_lvl;
    string points[MAX_LEVELS][MAX_HEIGHT][MAX_WIDTH];
    string out_l="",out_d="", out_list_p="";
    string d1_pos="", d2_pos="", d1_dir="", d2_dir="", targets[MAX_VISIT_POINTS], directions[MAX_VISIT_POINTS];

    //PARSING ARGUMENTS
    if(argc>=12 && argc%2==0 && is_number_int(argv[1]) && is_number_int(argv[2]) && is_number_int(argv[3]) && is_number(argv[4]) && is_number(argv[5]))
    {
        h = atoi(argv[1]);
        w = atoi(argv[2]);
        l = atoi(argv[3]);
        dst_pts = stod(argv[4]);
        dst_lvl = stod(argv[5]);
        d1_pos = argv[6];
        d2_pos = argv[7];
        d1_dir = argv[8];
        d2_dir = argv[9];
        for(int i=10;i<argc && i<MAX_VISIT_POINTS*2+10;i+=2)
        {
            targets[ntarget]=argv[i];
            directions[ntarget]=argv[i+1];
            ntarget++;
        }
    }
    else if(argc==2 && (!((string)argv[1]).compare("--help") || !((string)argv[1]).compare("-h")))
    {
        cout << "This tool generate a 2D map of points, giving height and width in points as input." << endl;
        cout << "Usage: ./map_tool <height> <width> <levels> <distance_pts> <distance_lvl> <drone1_position> <drone2_position> <drone1_direction> <drone2_direction> <target1 direction1 target2 direction2 ...>" << endl;
        exit(0);
    }
    else
    {
        cerr << "Wrong usage of this tool. Type --help or -h to see the guide" << endl; 
        exit(1);
    }

    //MAIN PROGRAM
    for(int i=0;i<l;i++)
        for(int j=0;j<h;j++)
            for(int k=0;k<w;k++)
                points[i][j][k] = "p" + to_string((i*w*h)+(j*w)+k);

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
            "    (cost)""\n"
            "  )""\n"
            ")""\n";

    return 0;
}