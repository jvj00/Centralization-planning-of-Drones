#include <iostream>
#include <fstream>
#include <algorithm>
#include <string.h>
#include <math.h>

using namespace std;

#define MAX_HEIGHT 100
#define MAX_WIDTH 100
#define MAX_VISIT_POINTS 100
#define MAX_LEVELS 5

#define X 0
#define Y 1
#define Z 2
#define DIR 3
#define DBL_MAX 1.7976931348623157E+308


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

void compute_drones_pos(int l, int h, int w, string points[MAX_LEVELS][MAX_HEIGHT][MAX_WIDTH], double points_coord[MAX_LEVELS][MAX_HEIGHT][MAX_WIDTH][3], double* d1_coord, double* d2_coord, string* d1_pos, string* d2_pos)
{
    double d1_min_value=DBL_MAX, d2_min_value=DBL_MAX;
    for(int i=0;i<l;i++)
        for(int j=0;j<h;j++)
            for(int k=0;k<w;k++)
            {
                double d1_dist=sqrt(pow(d1_coord[X]-points_coord[i][j][k][X],2)+pow(d1_coord[Y]-points_coord[i][j][k][Y],2)+pow(d1_coord[Z]-points_coord[i][j][k][Z],2));
                double d2_dist=sqrt(pow(d2_coord[X]-points_coord[i][j][k][X],2)+pow(d2_coord[Y]-points_coord[i][j][k][Y],2)+pow(d2_coord[Z]-points_coord[i][j][k][Z],2));
                if(d1_dist<d1_min_value)
                {
                    d1_min_value=d1_dist;
                    *d1_pos=points[i][j][k];
                }
                if(d2_dist<d2_min_value)
                {
                    d2_min_value=d2_dist;
                    *d2_pos=points[i][j][k];
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


int main(int argc, char** argv)
{
    int h, w, l, ntarget=0, index_arguments=1;
    bool pddl=false, ps2=false, json=false;
    char *pddl_file, *ps2_file, *json_file;
    double dst_pts, dst_lvl, low_lvl;
    string points[MAX_LEVELS][MAX_HEIGHT][MAX_WIDTH];
    double points_coord[MAX_LEVELS][MAX_HEIGHT][MAX_WIDTH][3];
    string out_l="",out_d="", out_list_p="", out_coordinates="";
    string d1_pos="", d2_pos="", targets[MAX_VISIT_POINTS];
    double d1_coord[3], d2_coord[3], targets_coord[MAX_VISIT_POINTS][3];

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
    if(argc>=15+index_arguments && (argc-(15+index_arguments))%3==0 && is_number_int(argv[index_arguments]) && is_number_int(argv[1+index_arguments]) && is_number_int(argv[2+index_arguments]) && is_number(argv[3+index_arguments]) && is_number(argv[4+index_arguments]) && is_number(argv[5+index_arguments]) && is_number(argv[6+index_arguments]) && is_number(argv[7+index_arguments]) && is_number(argv[8+index_arguments]) && is_number(argv[9+index_arguments]) && is_number(argv[10+index_arguments]) && is_number(argv[11+index_arguments]))
    {
        h = atoi(argv[index_arguments++]);
        w = atoi(argv[index_arguments++]);
        l = atoi(argv[index_arguments++]);
        dst_pts = stod(argv[index_arguments++]);
        dst_lvl = stod(argv[index_arguments++]);
        low_lvl = stod(argv[index_arguments++]);
        d1_coord[X] = stod(argv[index_arguments++]);
        d1_coord[Y] = stod(argv[index_arguments++]);
        d1_coord[Z] = stod(argv[index_arguments++]);
        d2_coord[X] = stod(argv[index_arguments++]);
        d2_coord[Y] = stod(argv[index_arguments++]);
        d2_coord[Z] = stod(argv[index_arguments++]);
        for(int i=index_arguments;i<argc && i<MAX_VISIT_POINTS*3+index_arguments;i+=3)
        {
            if(!is_number(argv[i]) || !is_number(argv[i+1]) || !is_number(argv[i+2])) common_error();
            targets_coord[ntarget][X]=stod(argv[i]);
            targets_coord[ntarget][Y]=stod(argv[i+1]);
            targets_coord[ntarget][Z]=stod(argv[i+2]);
            ntarget++;
        }
    }
    //HELP & ERRORS
    else if(argc==2 && (!((string)argv[1]).compare("--help") || !((string)argv[1]).compare("-h")))
    {
        cout << "This tool generate a 2D map of points, giving height and width in points as input." << endl << endl;
        cout << "Usage: ./map_tool [options] <height> <width> <levels> <distance_pts> <distance_lvl> <lowest_lvl> <d1_x> <d1_y> <d1_z> <d2_x> <d2_y> <d2_z> <t1_x t1_y t1_z, t2_x t2_y t2_z ...>" << endl;
        cout << endl;
        cout << "Parameters:" << endl;
        cout << "   <height(pts)> height 2D (y) in points of the map" << endl;
        cout << "   <width(pts)> width 2D (x) in points of the map" << endl;
        cout << "   <levels> # of levels (z) of the map" << endl;
        cout << "   <distance_pts> distance in meters between points of the same level" << endl;
        cout << "   <distance_lvl> distance in meters between levels" << endl;
        cout << "   <lowest_lvl> heigth in meters of the lowest level" << endl;
        cout << "   <d1_x> <d1_y> <d1_z> position (x,y,z) of the first drone. E.g.: 22.1 23.4 10.0" << endl;
        cout << "   <d2_x> <d2_y> <d2_z> same, but with second drone" << endl;
        cout << "   <t1_x t1_y t1_z, t2_x t2_y t2_z ...> list of target position (x,y,z) of the planning. E.g.: 30.1 40.2 25.1" << endl;
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

    //COMPUTES POINTS & COORDINATES 2 POINTS AND DIRECTIONS
    for(int i=0;i<l;i++)
        for(int j=0;j<h;j++)
            for(int k=0;k<w;k++)
            {
                points[i][j][k] = "p" + to_string((i*w*h)+(j*w)+k);
                points_coord[i][j][k][X]=k*dst_pts;
                points_coord[i][j][k][Y]=j*dst_pts;
                points_coord[i][j][k][Z]=i*dst_lvl+low_lvl;
                out_coordinates += "{\"name\":\"" + points[i][j][k] + "\",\"x\":" + to_string(points_coord[i][j][k][X]) + ",\"y\":" + to_string(points_coord[i][j][k][Y]) + ",\"z\":" + to_string(points_coord[i][j][k][Z]) + "}";
                if(i!=l-1 || j!=h-1 || k!=w-1)
                    out_coordinates += ",";
                
            }
    compute_drones_pos(l,h,w, points, points_coord, d1_coord, d2_coord, &d1_pos, &d2_pos);
    compute_targets_pos(l,h,w, ntarget, points, points_coord, targets_coord, targets);    

    //COMPUTES LINKS & DISTANCES
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
                "    (drone_pos d2 " << d2_pos << ")""\n\n";
        
        pddl_out << out_l << out_d;

        pddl_out << "  )""\n"
                "  (:goal""\n";

        if(ntarget==1)
            pddl_out << "    (picture "<< targets[0] << ")\n";
        else
        {
            pddl_out << "    (and \n";
            for(int i=0; i<ntarget; i++)
                pddl_out << "      (picture " << targets[i] << ")\n";
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