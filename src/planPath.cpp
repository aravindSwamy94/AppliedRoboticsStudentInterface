/**
 *  @file   planPath.cpp
 *  @brief  Contains the implementation of planPath functions
 *  @author Aravind Swaminathan
 *  @date   10-Jan-2020 
 ***********************************************/
#include "planPath.hpp"

/** @brief Class which reads and stores the configuration params related to planPath functions  */
class config_Params_planPath{
    public:

        int rrts_scaling_factor;
        double rrts_step_size;
        int rrts_max_iter;
        double rrts_goal_bias;
        double rrts_turn_radius;
        double rrts_neighbour_factor;

        double rrtsompl_max_solve_time;
        int rrtsompl_planner_type;

        bool use_rrt;
        bool use_rrt_star;
        bool use_rrt_star_ompl;
        int border_radius;
        int mission;
       
        double mission2_threshold_distance;

        bool save_global_path;
        bool save_local_path;
        string save_path_location;
        string global_path_file;
        string local_path_file;

        int offset_radius;
        bool use_clothoids;
        int kmax;
        int npts;
        double dist_bet_points;
        string config_folder;
        /**
        *  @brief read all params from config file and assign it to class variables
        *  @param config_folder config folder where the params are available
        */
        void read(const std::string& config_folder){
            std::string file_path = config_folder + "/config_params.cfg";
            const char *  configFile =file_path.c_str() ;
            Configuration *  cfg = Configuration::create();
            const char *     scope = "";
            try{
                cfg->parse(configFile);

                /*RRT Star own implementation params*/
                rrts_scaling_factor = cfg->lookupInt(scope, "rrts_scaling_factor");
                rrts_step_size = cfg->lookupFloat(scope, "rrts_step_size");
                rrts_max_iter = cfg->lookupInt(scope, "rrts_max_iter");
                rrts_goal_bias = cfg->lookupFloat(scope, "rrts_goal_bias");
                rrts_turn_radius = cfg->lookupFloat(scope, "rrts_turn_radius");
                rrts_neighbour_factor = cfg->lookupFloat(scope, "rrts_neighbour_factor");
                offset_radius = cfg->lookupInt(scope, "offset_radius");

                /*RRT Star OMPL Implematation*/
                rrtsompl_max_solve_time = cfg->lookupFloat(scope, "rrtsompl_max_solve_time");
                rrtsompl_planner_type = cfg->lookupInt(scope, "rrtsompl_planner_type");

                /*General plan path params*/
                use_rrt = cfg->lookupBoolean(scope, "use_rrt");
                use_rrt_star = cfg->lookupBoolean(scope, "use_rrt_star");
                use_rrt_star_ompl = cfg->lookupBoolean(scope, "use_rrt_star_ompl");
                border_radius = cfg->lookupInt(scope, "border_radius");
                mission = cfg->lookupInt(scope, "mission");

                /*Mission 2 computation parameters*/
                mission2_threshold_distance = cfg->lookupFloat(scope, "mission2_threshold_distance");
    
                /* Local planner params */
                use_clothoids = cfg->lookupBoolean(scope, "use_clothoids");
                kmax = cfg->lookupInt(scope, "kmax");      // Max angle of curvature
                npts = cfg->lookupInt(scope, "npts");  // Standard discretization unit of arcs
                dist_bet_points = cfg->lookupFloat(scope, "cl_dist_bet_points");

                save_global_path = cfg->lookupBoolean(scope, "save_global_path");
                save_local_path = cfg->lookupBoolean(scope, "save_local_path");
                save_path_location = cfg->lookupString(scope,"save_path_location");
                global_path_file = save_path_location + "global_path.csv";
                local_path_file = save_path_location + "local_path.csv";

                this->config_folder = config_folder; 
            } catch(const ConfigurationException & ex) {
                cerr << ex.c_str() << endl;
                cfg->destroy();
                
            }
        }
};

/** Function to store the path information computed to a file. Save location hardcoded
    @param x - Path struct 
    @return None
    */
void print_path(Path path,config_Params_planPath config_params, bool global){
    ofstream myfile;
    if(global)
        myfile.open(config_params.global_path_file.c_str());
    else
        myfile.open(config_params.local_path_file.c_str());
    myfile<<"s,x,y,theta,kappa" <<endl;
    for (int iter = 0 ; iter<path.points.size();iter++){
        myfile<<  path.points[iter].s<<","<<path.points[iter].x << "," <<path.points[iter].y<<","<<path.points[iter].theta<<","<<path.points[iter].kappa <<endl;
    }
    myfile<<endl;
    myfile<<endl;
}

namespace student{

    bool getCurvature(int step,Path& path)
    {
	    if (path.points.size() < 5)
		    return false;

	    Pose pplus, pminus;
	    double f1stDerivative, f2ndDerivative;

	    double curvature2D;
	    for (int i = 0; i < path.points.size(); i++)
	    {
		    const Pose& pos = path.points[i];

		    int maxStep = step;
		    maxStep = std::min(std::min(step, i), (int)path.points.size() - 1 - i);
		    if (maxStep == 0)
		    {
			    path.points[i].kappa = 0.0;//std::numeric_limits<double>::infinity();
			    continue;
		    }

		    int iminus = i - maxStep;
		    int iplus = i + maxStep;

		    pminus = path.points[iminus < 0 ? iminus + path.points.size() : iminus];
		    pplus = path.points[iplus > path.points.size() ? iplus - path.points.size() : iplus];

		    f1stDerivative = (pplus.y - pminus.y) / (pplus.x - pminus.x);
		    f2ndDerivative = ((pplus.y - pos.y) - (pos.y - pminus.y)) / (pplus.x - pminus.x);
		    curvature2D = 1 /( pow(sqrt(1 + pow(f1stDerivative,2)),3) / (f2ndDerivative));
		    path.points[i].kappa = curvature2D;
	    }
	    return true;
    }

    bool sort_pair_mission2(const std::pair<int,Polygon>& a, const std::pair<int,Polygon>& b){
        return (a.first < b.first);
    }
    /**
    *  @brief Expand obstacles region to avoid collision
    *  @details Using clipper libray , the execution of enlarging the obstacles is easier with AddPath() and Execute
    *  () API's
    *  @param ob All Obstacles polygon to be expanded
    *  @param offet_radius The radius by which the obstacle is to be expanded
    *  @return  Exapanded obstacles with the given radius
    */

    std::vector<Polygon> obstacleOffsetting(const std::vector<Polygon> ob,int offset_radius){
        std::vector<Polygon> offsettedObstacles;
        for(int i = 0; i < ob.size(); i++){
            ClipperLib::Path srcPoly;
            ClipperLib::Paths newPoly; 
            ClipperLib::ClipperOffset co;
            Polygon temp;
            const double INT_ROUND = 1000; // Scaling constant 
            // Push all points of obstacle polygon to clipper lib method
            for(size_t a = 0; a < ob[i].size(); ++a){
                double x = ob[i][a].x * INT_ROUND;
                double y = ob[i][a].y * INT_ROUND;
                srcPoly << ClipperLib::IntPoint(x, y);
            }
            // If not a closed polygon
            if(ob[i].size() == 3)
            {
                co.AddPath(srcPoly, ClipperLib::jtSquare, ClipperLib::etClosedLine);
            }
            else // If it is a closed polygon
            {
                co.AddPath(srcPoly, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
            }

            //Execute Clipper offset and return the new set of enlarged obstacles
            co.Execute(newPoly, offset_radius);
            for(const ClipperLib::Path &path: newPoly){
                for(const ClipperLib::IntPoint &pt: path){
                    double x = (double)pt.X / INT_ROUND;
                    double y = (double)pt.Y / INT_ROUND;
                    temp.emplace_back(x, y);
                }
            }

            offsettedObstacles.emplace_back(temp);
        }
        return offsettedObstacles;
    }

    /**
    *  @brief Resize the border for avoiding collision with border
    *  @details manual computation of resizing borders
    *  @param borders border which is given as polygon
    *  @param resize the resizing factor 
    *  @return  resized border
    */

    Polygon resizeBorders(const Polygon &borders,  int resize){

        resize = resize/1000; 
        Polygon re_border;
        double MAX_X = (borders[1].x);
        double MAX_Y = (borders[3].y);
        double MIN_X = (borders[0].x);
        double MIN_Y = (borders[0].y);
        for(auto &pt: borders){
            double b_x = pt.x;
            double b_y = pt.y;
            if(b_x < MAX_X/2){ // If the x is on the first half of the arena
                b_x += resize; 
            }else{ // else on second half of arena
                b_x -= resize;
            }

            if(b_y < MAX_Y/2){ // if y on the bottom half of arena
                b_y += resize;
            }else{ // else on top of arena
                b_y -= resize;
            }
            re_border.push_back(Point(b_x, b_y));
        }

        return re_border;
    }

    /**
    *  @brief Sample the borders with multiple points
    *  @details interpolate the border points, to check if generated path is closer to sampled borders
    *  @param borders border which is given as polygon
    *  @return  interpolated border points
    */

    Polygon sample_borders(Polygon &borders){
        double MAX_X = (borders[1].x);
        double MAX_Y = (borders[3].y);
        double MIN_X = (borders[0].x);
        double MIN_Y = (borders[0].y);
        double sample_step_size = 0.1; // sampling step size
        Polygon sampled_borders; 
        int num_of_points = ((MAX_X - MIN_X)/sample_step_size); // num of points to be generated
        for (int i=0;i<num_of_points+1;i++){ // generating points for lower horizoontal line
            Point temp;
            temp.x = MIN_X + i*(sample_step_size);
            temp.y = MIN_Y;
            sampled_borders.emplace_back(temp);
        }
        num_of_points = ((MAX_Y - MIN_Y)/sample_step_size);
        for (int i=0;i<num_of_points+1;i++){// generating points for right vertical line
            Point temp;
            temp.x = MAX_X ;
            temp.y = MIN_Y + i*(sample_step_size);
            sampled_borders.emplace_back(temp);
        }
        num_of_points = ((MAX_X - MIN_X)/sample_step_size);
        for (int i=0;i<num_of_points+1;i++){// generating points for upper horizoontal line
            Point temp;
            temp.x = MAX_X - i*(sample_step_size);
            temp.y = MAX_Y;
            sampled_borders.emplace_back(temp);
        }
        num_of_points = ((MAX_Y - MIN_Y)/sample_step_size);
        for (int i=0;i<num_of_points+1;i++){// generating points for left vertical line
            Point temp;
            temp.x = MIN_X ;
            temp.y = MAX_Y - i*(sample_step_size);
            sampled_borders.emplace_back(temp);
        }

        return sampled_borders;
    }

    /**
    *  @brief get centroid of any polygon
    *  @details sum_of_all_vertices/ size_of_vertices
    *  @param poly Inpput polygon
    *  @return center as a std::pair 
    */
    
    std::pair<double, double> get_center(const Polygon &poly) {
        double cx = 0;
        double cy = 0;
        for (int pt = 0; pt < poly.size(); pt++){
            cx += poly[pt].x;
            cy += poly[pt].y;
        }
        cx /= poly.size();
        cy /= poly.size();
        return std::make_pair(cx, cy);
    }

    /**
    *  @brief function to check if a given point is inside polygon 
    *  @details using boost::within library
    *  @param poly input polygon
    *  @param pt input point to check if this is inside the poly
    *  @return true/false - insidePolygon/NotInsidePolygon 
    */
    bool pointInsidePolygon(Polygon poly, Point pt){

        boost_point point(pt.x, pt.y);
        boost_polygon Poly = convertPolygonToBoostPolygon(poly);
        if (boost::geometry::within(point, Poly)) 
            return true;
        else
            return false;     
    }


    /**
    *  @brief function to compute the gate angle 
    *  @details gate angle is very important and cannot be same as other cases. Because the recatngle of gate can be 
    *  different locations(Left,Right,Bottom,Top) and oriented(Horizontal, Vertical) in different way. A combination 
    *  of the locations and orientation are possible.
    *  @param borders the border locations
    *  @param gateX gate center X
    *  @param gateY gate center Y
    *  @return the approach angle of the final gate 
    */
    double compute_angle_gate(Polygon borders,double gateX,double gateY){

        double gateAngle;    
        if (fabs(gateX - borders[0].x) < fabs(gateX - borders[1].x)){// Left
            if (fabs(gateY - borders[0].y) < fabs(gateY - borders[3].y)){ // Bottom-left        
                if (fabs(gateY - borders[0].y) < fabs(gateX - borders[0].x)){// Horizontal orientaton
                    gateAngle = -M_PI/2;
                } else {// Vertical Oreintation
                    gateAngle = M_PI;
                }
            } else {// Top-left
                if (fabs(gateY - borders[3].y) < fabs(gateX - borders[0].x)){// Horizontal Oreintation
                    gateAngle = M_PI/2;
                } else {// Vertical orientation
                    gateAngle = M_PI;
                }
            }
        } else {// Right
            if (fabs(gateY - borders[0].y) < fabs(gateY - borders[3].y)){// Bottom-right
                if (fabs(gateY - borders[0].y) < fabs(gateX - borders[1].x)){// Horizontal
                    gateAngle = -M_PI/2;
                } else {// Vertical
                    gateAngle = 0;
                }
            } else {// Top-right
                if (fabs(gateY - borders[3].y) < fabs(gateX - borders[1].x)){// Horizontal
                    gateAngle = M_PI/2;
                } else {// Vertical
                    gateAngle = 0;
                }
            }    
        }
        return gateAngle;                
    }

        
    double internal_angle(double angle1, double angle2){
        if(angle1-angle2 >=0 && angle1-angle2 < M_PI)
            return angle1-angle2;
        else if(angle1-angle2 <0 && angle2-angle1 < M_PI)
            return angle2-angle1;
        else if(angle1-angle2 >=0 && angle1-angle2 > M_PI)
            return 2*M_PI-(angle1-angle2);
        else if(angle1-angle2 <0 && angle2-angle1 > M_PI)
            return 2*M_PI-(angle2-angle1);
        return angle2;
    }

    /**
    *  @brief function to compute the approach angle between two nodes
    *  @details The motion planning generates points betwwen source, victims and to gate. This points are then given 
    *  to local planner(Clothoids/Dubins). This function will compute the approach angle between two points. Also a 
    *  special logic to get the approach angle for victims. The logic is to use the line segment angle difference. 
    *  Three points are taken, the angle between the first line segment(first and second points) and next line 
    *  segments(second and third points) are used to compute thh approach angle of second point.
    *  @param first First point 
    *  @param second Second point 
    *  @param third Third point 
    *  @return the approach angle of the second point 
    */

    double get_angle(Pose first, Pose second, Pose third) {
        double temp = 0;
        double distance1 = sqrt(pow(second.x - first.x, 2) + pow(second.y - first.y, 2)); // distance difference
        double distance2 = sqrt(pow(third.x - second.x, 2) + pow(third.y - second.y, 2));
        double angle1 = atan2((first.y - second.y), (first.x - second.x));// angle difference
        double angle2 = atan2((third.y - second.y), (third.x - second.x));
        angle1<0? angle1 = 2*M_PI+angle1: angle1+=0; 
        angle2<0? angle2 = 2*M_PI+angle2: angle2+=0;
        angle1 < angle2 ? temp = std::fmod((((angle1+angle2)/2) + M_PI/2), 2*M_PI): temp = std::fmod((((angle1+angle2)/2) - M_PI/2), 2*M_PI);;
        temp<0?temp = 2*M_PI+temp: temp+=0; 

        if(internal_angle(angle1, angle2) < 0.20){ // If the line segments doesnt have big difference, then they are on almost same path and clothoids/dubins can handle easily
	        return temp;
        }
        double final_angle = temp;
        double distanceFactor = (distance1 > distance2) ? 1 - distance2/distance1 : 1 - distance1/distance2;// calculate the distance factor for angle difference in first and second segment 
        double a_minus = std::fmod(temp - (internal_angle(angle2,temp) * distanceFactor), 2*M_PI);// angle which is differnce of existing angle difference and angle difference between second segment and new slope
        double a_plus = std::fmod(temp + (internal_angle(angle2,temp) * distanceFactor), 2*M_PI);
// angle which is sum of existing angle difference and angle difference between second segment and new slope
        double angle_a1_minus = internal_angle(angle2,a_minus);
        double angle_a1_plus = internal_angle(angle2,a_plus);
        if(distance1 > distance2){ 
            if(angle_a1_plus > angle_a1_minus){
                final_angle = a_minus;
            }else{
                final_angle = a_plus;
            }
        }else if (distance2 > distance1){
            if(angle_a1_plus > angle_a1_minus){
                final_angle = a_plus;
            }else{
                final_angle = a_minus;
            }
        }

        final_angle = std::fmod(final_angle, 2*M_PI);
        return final_angle;
    }


    /**
    *  @brief function to check if the generated path is colliding with borders and obstacles 
    *  @details This is an additional function which checcks if the path generated by Clothoids/Dubins is colliding 
    *  with the obstacles and borders. Here sampled borders are used to check the distance between path and borders 
    *  @param path The path generated by Clothoids/Dubins
    *  @param sampled_borders the borders points which are sampled
    *  @param obstacle_list The list of obstacle and their points 
    *  @param obs_radius Radius of each obstacle
    *  @param obs_center Center of each obstacle 
    *  @return true/false- Colliding/Not Colliding 
    */

    bool check_collison_with_borders_and_obstacles(Path path,Polygon borders,Polygon sampled_borders, std::vector<Polygon> obstacle_list,std::vector<double> obs_radius, std::vector<Point> obs_center){

        bool collision = false;
        for(int j=0; j<path.points.size(); j++){

            /*
                Check for collison with borders 
            */
            bool borderResult = pointInsidePolygon(borders, Point(path.points.at(j).x,path.points.at(j).y)); //check path point is inside the border 
            for(int i=0; i<sampled_borders.size(); i++){ 
                double borderDistance = sqrt(pow((path.points.at(j).x-sampled_borders[i].x),2)+pow((path.points.at(j).y-sampled_borders[i].y),2));
                if(borderResult != true or borderDistance < (0.02)){ //Check if path point is away from border  
                    collision = true;          
                    break;
                }
            }
            /*
                Check for collisions with obstacle
            */
            for(int i=0; i<obstacle_list.size(); i++){
                bool obstacleResult = pointInsidePolygon(obstacle_list.at(i), Point(path.points.at(j).x,path.points.at(j).y));//Check if path point is inside the obstacle
                double obstacleDistance = sqrt(pow((path.points.at(j).x-obs_center.at(i).x),2)+pow((path.points.at(j).y-obs_center.at(i).y),2)); // Check for distance from the obstacle center                   
                if(obstacleResult == true or obstacleDistance < (obs_radius.at(i))){ 
                    collision = true;
                    break; 
                }
            }           
        }
        return collision;

    }

    /**
    *  @brief implmentation of RRT Star function
    *  @details This is an own implementation of RRT Star with few references.  
    *  @param x The robot location x
    *  @param y The robot location y
    *  @param theta The robot orientation theta
    *  @param path the output path variable
    *  @param localGoals the local goals which includes the victims and gate
    *  @param borders original borders 
    *  @param sampled_borders the borders points which are sampled
    *  @param obstacle_list The list of obstacle and their points 
    *  @param obs_radius Radius of each obstacle
    *  @param obs_center Center of each obstacle 
    *  @param config_params Configuration parameters related to mission2 victim computation   
    */
    void RRT_Star(const float x, const float y,const float theta, Path& path, std::vector<Point>& localGoals, const Polygon& borders,Polygon& sampled_borders, const std::vector<Polygon>& obstacle_list, std::vector<double> obs_radius, std::vector<Point> obs_center,config_Params_planPath config_params){

        auto start_rrt = high_resolution_clock::now();
        RRTSTAR *rrtstar;
        rrtstar = new RRTSTAR;
        int MAX_X = (borders[1].x*config_params.rrts_scaling_factor);
        int MIN_X = (borders[0].x*config_params.rrts_scaling_factor);
        int MAX_Y = (borders[3].y*config_params.rrts_scaling_factor);
        int MIN_Y = (borders[0].y*config_params.rrts_scaling_factor);
        int width =  MAX_X - MIN_X;
        int height = MAX_Y -MIN_Y;
        rrtstar->set_max_iter(config_params.rrts_max_iter); // set maximum iterations for one trial
        rrtstar->set_step_size (config_params.rrts_step_size*config_params.rrts_scaling_factor); // set step size(distance betwwen rrt* points)
        assert(rrtstar->step_size > 0); 
        assert(rrtstar->max_iter > 0);
        rrtstar->set_bot_radius(config_params.offset_radius); // set the bot radius
        rrtstar->setWorldInfo(width,height); // set the world infor (rectanlge info)
        rrtstar->set_goalBias(config_params.rrts_goal_bias*config_params.rrts_scaling_factor); // Set goal bias
        rrtstar->set_turn_radius(config_params.rrts_turn_radius*config_params.rrts_scaling_factor); // Robot turn radius for Dubins 
        rrtstar->set_rrt_star_neighbour_factor(config_params.rrts_neighbour_factor*config_params.rrts_scaling_factor); // Rewiring search parameters
        rrtstar->initialize(); // Initialize RRT*

        double gateX = localGoals[localGoals.size()-1].x;
        double gateY = localGoals[localGoals.size()-1].y;

        for (int i=0;i<obs_center.size();i++){
            rrtstar->obstacles->addObstacle(obs_radius[i]*config_params.rrts_scaling_factor, Vector2f(obs_center[i].x*config_params.rrts_scaling_factor, obs_center[i].y*config_params.rrts_scaling_factor)); // Add all obstacles to world 
        }
        int goal = 1;
        Path temp_path;
        while(goal < localGoals.size()){
            // Set the start pose of algorithm
            if(goal == 1)
                rrtstar->setStartPose(x*config_params.rrts_scaling_factor,y*config_params.rrts_scaling_factor,theta);
            else
                rrtstar->setStartPose(temp_path.points.back().x*config_params.rrts_scaling_factor,temp_path.points.back().y * config_params.rrts_scaling_factor, temp_path.points.back().theta);
            // Set the goal pose of algorithm
            rrtstar->setGoalPose((localGoals[goal].x*config_params.rrts_scaling_factor),(localGoals[goal].y*config_params.rrts_scaling_factor));
            rrtstar->path.clear();
            // RRTSTAR Algorithm
            for(int i = 0; i < rrtstar->max_iter; i++){
                Node *q = rrtstar->getRandomNode(); // get random node
                if (q) {
                    Node *qNearest = rrtstar->nearest(q->position);// compute nearest node
                    if (rrtstar->distance(q->position, qNearest->position) > rrtstar->step_size) { // if node is above step_size limit
                        Vector3f newConfigPosOrient;
                        newConfigPosOrient = rrtstar->newConfig(q, qNearest); // compute the new pose of the location 
                        Vector2f newConfigPos(newConfigPosOrient.x(), newConfigPosOrient.y());
                        if (!rrtstar->obstacles->isSegmentInObstacle(newConfigPos, qNearest->position)) { // If the new config is not colliding with any obstacles.
                            
                            Node *qNew = new Node;
                            qNew->position = newConfigPos;
                            qNew->orientation = newConfigPosOrient.z();
                            // rewiring of Nodes after generation of a node
                            vector<Node *> Qnear;
                            rrtstar->near(qNew->position, rrtstar->step_size*rrtstar->rrt_star_neighbour_factor, Qnear);// Check for nodes near to newly generated node
                            Node *qMin = qNearest;
                            double cmin = rrtstar->Cost(qNearest) + rrtstar->PathCost(qNearest, qNew);// compute cost 
                            for(int j = 0; j < Qnear.size(); j++){// compute the node whcih has minimum cost in the near nodes list
                                Node *qNear = Qnear[j];
                                if(!rrtstar->obstacles->isSegmentInObstacle(qNear->position, qNew->position) &&
                                        (rrtstar->Cost(qNear)+rrtstar->PathCost(qNear, qNew)) < cmin ){// check if new node is not colliding with obstacles and cost is minimum 
                                    
                                    qMin = qNear; cmin = rrtstar->Cost(qNear)+rrtstar->PathCost(qNear, qNew);
                                }
                            }
                            rrtstar->add(qMin, qNew); // add this newly computed to the rrt list
                            for(int j = 0; j < Qnear.size(); j++){
                                Node *qNear = Qnear[j];
                                if(!rrtstar->obstacles->isSegmentInObstacle(qNew->position, qNear->position) &&
                                        (rrtstar->Cost(qNew)+rrtstar->PathCost(qNew, qNear)) < rrtstar->Cost(qNear)){
                                    // Rewire the nodes 
                                    Node *qParent = qNear->parent;
                                    // Remove connection between qParent and qNear
                                    qParent->children.erase(std::remove(qParent->children.begin(), qParent->children.end(), qNear), qParent->children.end());

                                    // Add connection between qNew and qNear
                                    qNear->cost = rrtstar->Cost(qNew) + rrtstar->PathCost(qNew, qNear);
                                    qNear->parent = qNew;
                                    qNew->children.push_back(qNear);
                                }
                            }
                        }
                    }
                }
                if (rrtstar->reached()) { // If goal is reached
                    cout<< "Goal " <<goal<< " Reached!"<<endl;
                    break; // stop the rrt star exploration
                }
            }
            Node *q;
            if (rrtstar->reached()) {
                q = rrtstar->lastNode;
            }
            else
            {            
                cout<<"Exceeded max iterations! , path not found for Local goal number"<< goal <<endl;
            }
            while (q != NULL) { // traverse back from destination node to soure node
                rrtstar->path.push_back(q);
                q = q->parent;
            }
            std::reverse(rrtstar->path.begin(),rrtstar->path.end());
            for(int i =0;i<rrtstar->path.size();i++){
                Pose path_point;
                path_point.x = rrtstar->path[i]->position.x()/config_params.rrts_scaling_factor;
                path_point.y = rrtstar->path[i]->position.y()/config_params.rrts_scaling_factor;
                cout<<path_point.x<<","<<path_point.y<<endl;
                temp_path.points.emplace_back(path_point); // push this to temp path for clothoids/dubins path computation
            }
            goal = goal+1;
        }
        auto stop_rrt = high_resolution_clock::now();
        auto duration_rrt = duration_cast<milliseconds>(stop_rrt - start_rrt); 
        cout <<"RRT Star Planning time " << duration_rrt.count() << endl;

        if(config_params.save_global_path)
            print_path(temp_path,config_params,true);


        auto start_lp = high_resolution_clock::now();
        G2lib::G2solve3arc g2solve3arc; // G2 Clothoids solver
        Path clothoid_path;
        G2lib::ClothoidList CL; // Clothoid list for multiple clothoids
        Path dubins_path;  
        dubinsCurve dubins = {}; // dubins curve 
        double sx=0,sy=0,st=0,ex=0,ey=0,et=0;
        for(int iter=1;iter<temp_path.points.size();iter++)
        {
            if(iter == 1){
                sx = x;
                sy = y;   
                st = theta;
            }
            ex = temp_path.points[iter].x;
            ey = temp_path.points[iter].y;
            et = get_angle(temp_path.points[iter-1],temp_path.points[iter],temp_path.points[iter+1]); // get approach angle
            if(iter == temp_path.points.size()-1)
                et = compute_angle_gate(borders,gateX,gateY);            
            if(config_params.use_clothoids){
                g2solve3arc.build(sx, sy, st,0, ex, ey, et,0); // build clothoids
                G2lib::ClothoidCurve const & S0 = g2solve3arc.getS0(); // get 3 arcs of clothoids 
                G2lib::ClothoidCurve const & S1 = g2solve3arc.getS1();
                G2lib::ClothoidCurve const & SM = g2solve3arc.getSM();
                CL.push_back(S0);// Push 3 clothoids to list.
                CL.push_back(SM);
                CL.push_back(S1);
            }else{
                dubins_shortest_path(dubins, sx, sy, st, ex, ey, et, config_params.kmax); //find shortest path and discretize them
                dubins_path = getPath(dubins,config_params.npts);
                path.points.insert(path.points.end(),dubins_path.points.begin(),dubins_path.points.end());              
            }
            sx = ex;//path.points.back().x;
            sy = ey;//path.points.back().y;
            st = et;//path.points.back().theta;
        }
        if(config_params.use_clothoids){
            int num_of_points = (int)(CL.length()/config_params.dist_bet_points);
            for (int pp=0;pp<num_of_points;pp++)
            {
                Pose temp;
                double px=0,py=0,pth=0,pk=0;
                CL.evaluate(pp*config_params.dist_bet_points,pth,pk,px,py); // Use evaluate to find points in clothoids
                temp.s = pp*config_params.dist_bet_points;
                temp.theta = pth;
                temp.kappa = pk;
                temp.x = px;
                temp.y = py;
                path.points.emplace_back(temp);
            }
        }

        auto stop_lp = high_resolution_clock::now();
        auto duration_lp = duration_cast<milliseconds>(stop_lp - start_lp); 
        cout <<"Local Planner Planning time " << duration_lp.count() << endl;


        if(config_params.save_local_path)
            print_path(path,config_params,false);
    }

    /**
    *  @brief implmentation of RRT Star function using OMPL library
    *  @details This is an RRT Star using OMPL API's.  
    *  @param x The robot location x
    *  @param y The robot location y
    *  @param theta The robot orientation theta
    *  @param path the output path variable
    *  @param localGoals the local goals which includes the victims and gate
    *  @param borders original borders 
    *  @param sampled_borders the borders points which are sampled
    *  @param obstacle_list The list of obstacle and their points 
    *  @param obs_radius Radius of each obstacle
    *  @param obs_center Center of each obstacle 
    *  @param config_params Configuration parameters related to mission2 victim computation   
    */

    void RRT_Star_ompl(const float x, const float y,const float theta, Path& path, std::vector<Point>& localGoals, const Polygon& borders,Polygon& sampled_borders, const std::vector<Polygon>& obstacle_list, std::vector<double> obs_radius, std::vector<Point> obs_center,config_Params_planPath config_params){


        double MAX_X = (borders[1].x);
        double MIN_X = (borders[0].x);
        double MAX_Y = (borders[3].y);
        double MIN_Y = (borders[0].y);
        auto start_rrt = high_resolution_clock::now();
        auto space(std::make_shared<ob::RealVectorStateSpace>(2));

        //Set world info
        space->setBounds(0.0, std::max(MAX_X,MAX_Y));

        // Construct a space information instance for this state space
        auto si(std::make_shared<ob::SpaceInformation>(space));
        vector<Polygon> new_obstacle_list;
        for(int iter=0 ; iter<obstacle_list.size();iter++)
        {
            if(obstacle_list[iter].size())
                new_obstacle_list.emplace_back(obstacle_list[iter]);
            cout<<endl;
        }

        // Set the object used to check which states in the space are valid
        si->setStateValidityChecker(std::make_shared<ValidityChecker>(si,new_obstacle_list));

        si->setup();

        int goal_iter = 1;

        Path temp_path;
        double gateX = localGoals[localGoals.size()-1].x;
        double gateY = localGoals[localGoals.size()-1].y;

        ob::ScopedState<> start(space);
        ob::ScopedState<> goal(space);

        while(goal_iter < localGoals.size()){

            //if(goal_iter == 2 ) break;

            // set the start point in world
            if(goal_iter == 1 ){
                start->as<ob::RealVectorStateSpace::StateType>()->values[0] = x;
                start->as<ob::RealVectorStateSpace::StateType>()->values[1] = y;
            }
            else{
                start->as<ob::RealVectorStateSpace::StateType>()->values[0] = temp_path.points.back().x;
                start->as<ob::RealVectorStateSpace::StateType>()->values[1] = temp_path.points.back().y;            
            }
            //set the goal point in world 
            goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = localGoals[goal_iter].x;
            goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = localGoals[goal_iter].y;


            // Create a problem instance
            auto pdef(std::make_shared<ob::ProblemDefinition>(si));

            // Create the optimization objective specified by us.
            pdef->setOptimizationObjective(allocateObjective(si, OBJECTIVE_PATHLENGTH));

            // Set the start and goal states
            pdef->setStartAndGoalStates(start, goal);
            
            ob::PlannerPtr optimizingPlanner;
            // Construct the optimal planner specified by us in config file.
            if(config_params.rrtsompl_planner_type == 1)
                optimizingPlanner = allocatePlanner(si, PLANNER_PRMSTAR);
            else if (config_params.rrtsompl_planner_type==2)
                optimizingPlanner = allocatePlanner(si, PLANNER_RRTSTAR);
			
            // Set the problem instance for our planner to solve
            optimizingPlanner->setProblemDefinition(pdef);
            optimizingPlanner->setup();

            // attempt to solve the planning problem in the given runtime
            ob::PlannerStatus solved = optimizingPlanner->solve(config_params.rrtsompl_max_solve_time);

            if (solved)
            {
                std::cout
                << optimizingPlanner->getName()
                << " found a solution of length "
                << pdef->getSolutionPath()->length()
                << " with an optimization objective value of "
                << pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()) << std::endl;

                std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
                og::PathGeometric path_ompl( dynamic_cast< const og::PathGeometric& >(*pdef->getSolutionPath()));
                const std::vector<ob::State*> &states = path_ompl.getStates();
                ob::State *state;
                //cout<<"num of path points are" << states.size()<<endl;

                Pose robot_pos;
                robot_pos.x = x;
                robot_pos.y = y;
                if(goal_iter == 1 )
                {
                    temp_path.points.emplace_back(robot_pos);
                }
                for( size_t i = 1 ; i < states.size( ) ; ++i )
                {
                    state = states[i]->as< ob::State >();
                    Pose temp;
                    temp.x = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
                    temp.y = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
                    temp_path.points.emplace_back(temp);
                }
            }    
            else{
                cout<<"could not find a solution for goal"<<goal <<endl;
            }
           goal_iter = goal_iter +1;
        }
        auto stop_rrt = high_resolution_clock::now();
        auto duration_rrt = duration_cast<milliseconds>(stop_rrt - start_rrt); 
        cout <<"RRT Star Planning time " << duration_rrt.count() << endl;
        getCurvature(1, temp_path);
        if(config_params.save_global_path)
            print_path(temp_path,config_params,true);

        auto start_lp = high_resolution_clock::now();
        G2lib::G2solve3arc g2solve3arc; // G2 solver
        Path clothoid_path;
        G2lib::ClothoidList CL; // Clothoid list for handling multiple clothoids
        Path dubins_path;
        dubinsCurve dubins = {}; // dubins curve
        double sx=0,sy=0,st=0,sk=0,ex=0,ey=0,et=0,ek=0;
        for(int iter=1;iter<temp_path.points.size();iter++)
        {

            if(iter == 1){
                sx = x;
                sy = y;   
                st = theta;
                sk = 0;
            }
            ex = temp_path.points[iter].x;
            ey = temp_path.points[iter].y;
            et = get_angle(temp_path.points[iter-1],temp_path.points[iter],temp_path.points[iter+1]); // get approach angle between points
            ek = temp_path.points[iter].kappa;
            if(iter == temp_path.points.size()-1)
                et = compute_angle_gate(borders,gateX,gateY);            
            if(config_params.use_clothoids){
                g2solve3arc.build(sx, sy, st,sk, ex, ey, et,ek); // build clothoids
                G2lib::ClothoidCurve const & S0 = g2solve3arc.getS0();// get the three arcs of clothoids
                G2lib::ClothoidCurve const & S1 = g2solve3arc.getS1();
                G2lib::ClothoidCurve const & SM = g2solve3arc.getSM();
                CL.push_back(S0);// push all clothoids to Clothoid list 
                CL.push_back(SM);
                CL.push_back(S1);
            }else{
                dubins_shortest_path(dubins, sx, sy, st, ex, ey, et, config_params.kmax); // get dubins curve and discretize them     
                dubins_path = getPath(dubins,config_params.npts);
                path.points.insert(path.points.end(),dubins_path.points.begin(),dubins_path.points.end());
            }
            sx = ex;//path.points.back().x;
            sy = ey;//path.points.back().y;
            st = et;//path.points.back().theta;
            sk = ek;
        }

        if(config_params.use_clothoids){
            int num_of_points = (int)(CL.length()/config_params.dist_bet_points);
            for (int pp=0;pp<num_of_points;pp++)
            {
                Pose temp;
                double px=0,py=0,pth=0,pk=0;
                CL.evaluate(pp*config_params.dist_bet_points,pth,pk,px,py); // use evalute to compute points in clothoid
                temp.s = pp*config_params.dist_bet_points;
                temp.theta = pth;
                temp.kappa = pk;
                temp.x = px;
                temp.y = py;
                path.points.emplace_back(temp);
            }
        }

        auto stop_lp = high_resolution_clock::now();
        auto duration_lp = duration_cast<milliseconds>(stop_lp - start_lp); 
        cout <<"Local Planner Planning time " << duration_lp.count() << endl;
        if(config_params.use_clothoids){
            path.points.pop_back();
            path.points.pop_back();
            path.points.pop_back();
        }
        else{
            path.points.pop_back();
        }
        if(config_params.save_local_path)
            print_path(path,config_params,false); // print path to path file

    }


    /**
    *  @brief implmentation of Mission targets for mision 2
    *  @details This is very simple logic, where RRT* path is generated from source and gate . And a threshold value 
    *  is used to check the distance of victims from path. If the distance is high the cost is computed accordingly 
    *  and that vicitm is rejected to be saved. Also the victim is then sorted based on their occurance in path 
    *  direction. This helps save time for robot.
    *  @param x The robot location x
    *  @param y The robot location y
    *  @param theta The robot orientation theta
    *  @param path the output path variable
    *  @param localGoals the local goals which includes the victims and gate
    *  @param borders original borders 
    *  @param sampled_borders the borders points which are sampled
    *  @param obstacle_list The list of obstacle and their points 
    *  @param obs_radius Radius of each obstacle
    *  @param obs_center Center of each obstacle 
    *  @param config_params Configuration parameters related to mission2 victim computation     
    */

    std::vector<Point> compute_vicitim_mission2(const float x, const float y,const float theta,  const Polygon& borders,const std::vector<Polygon>& obstacle_list, std::pair<double, double> gateCenter, std::vector<std::pair<int,Polygon>> victim_list,config_Params_planPath config_params){


        double MAX_X = (borders[1].x);
        double MIN_X = (borders[0].x);
        double MAX_Y = (borders[3].y);
        double MIN_Y = (borders[0].y);
        auto space(std::make_shared<ob::RealVectorStateSpace>(2));

        //set world info
        space->setBounds(0.0, std::max(MAX_X,MAX_Y));

        // Construct a space information instance for this state space
        auto si(std::make_shared<ob::SpaceInformation>(space));
        vector<Polygon> new_obstacle_list;
        for(int iter=0 ; iter<obstacle_list.size();iter++)
        {
            cout<<"Obstacle size is "<< obstacle_list[iter].size()<<endl;
            if(obstacle_list[iter].size())
                new_obstacle_list.emplace_back(obstacle_list[iter]);
            cout<<endl;
        }

        // Set the object used to check which states in the space are valid
        si->setStateValidityChecker(std::make_shared<ValidityChecker>(si,new_obstacle_list));

        si->setup();

        ob::ScopedState<> start(space);
        ob::ScopedState<> goal(space);

        //set start pose
        start->as<ob::RealVectorStateSpace::StateType>()->values[0] = x;
        start->as<ob::RealVectorStateSpace::StateType>()->values[1] = y;

        //set goal pose
        goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = gateCenter.first;
        goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = gateCenter.second;

        // Create a problem instance
        auto pdef(std::make_shared<ob::ProblemDefinition>(si));

        // Create the optimization objective specified by our command-line argument.
        // This helper function is simply a switch statement.
        pdef->setOptimizationObjective(allocateObjective(si, OBJECTIVE_PATHLENGTH));

        // Set the start and goal states
        pdef->setStartAndGoalStates(start, goal);

        ob::PlannerPtr optimizingPlanner;
        // Construct the optimal planner specified by our command line argument.
        // This helper function is simply a switch statement.
        if(config_params.rrtsompl_planner_type == 1)
            optimizingPlanner = allocatePlanner(si, PLANNER_PRMSTAR);
        else if (config_params.rrtsompl_planner_type==2)
            optimizingPlanner = allocatePlanner(si, PLANNER_RRTSTAR);

        // Set the problem instance for our planner to solve
        optimizingPlanner->setProblemDefinition(pdef);
        optimizingPlanner->setup();
        boost_linestring temp_path;// easy for computing distance between point and line
        // attempt to solve the planning problem in the given runtime
        ob::PlannerStatus solved = optimizingPlanner->solve(config_params.rrtsompl_max_solve_time);
        std::vector<std::pair<int,Polygon>> updated_victims;
        if (solved)
        {

            cout<< "Finding a path for mission 2" <<endl;
            std::cout
            << optimizingPlanner->getName()
            << " found a solution of length "
            << pdef->getSolutionPath()->length()
            << " with an optimization objective value of "
            << pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()) << std::endl;

            std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
            og::PathGeometric path_ompl( dynamic_cast< const og::PathGeometric& >(*pdef->getSolutionPath()));
            const std::vector<ob::State*> &states = path_ompl.getStates();
            ob::State *state;

            boost_point robot_pos(x,y);
            temp_path.emplace_back(robot_pos);
            for( size_t i = 1 ; i < states.size( ) ; ++i )
            {
                state = states[i]->as< ob::State >(); 
                boost_point temp(state->as<ob::RealVectorStateSpace::StateType>()->values[0],state->as<ob::RealVectorStateSpace::StateType>()->values[1]); // push all rrt* points to linestring polygon 
                temp_path.emplace_back(temp);
            }
        }
        else{
            cout<<"could not find a solution for goal"<<goal <<endl;
        }

        for (int iter =0; iter<victim_list.size();iter++) {
            boost_polygon currentVictim = convertPolygonToBoostPolygon(victim_list[iter].second);
            if (bg::distance(currentVictim, temp_path) <= config_params.mission2_threshold_distance) {
                int index = -1;
                double minDist = 10000;
                for (int i = 0; i < temp_path.size(); i++) {
                    float distance = bg::distance(currentVictim, temp_path[i]); // distance betwwen victim and line segment
                    if (distance < minDist) {
                        minDist = distance;
                        index = i; // if victim is within limit, get the order of vicitm in traversing path direction
                    }            
                }
                updated_victims.push_back({index, victim_list[iter].second});
            }
        }
        sort(updated_victims.begin(), updated_victims.end(), sort_pair_mission2);// sort the vicitms based on their order 
        std::vector<Point> generalPath;
        generalPath.push_back(Point(x,y));
        for(int iter =0; iter< updated_victims.size();iter++){
            Polygon currentVictim = std::get<1>(updated_victims[iter]);
            std::pair<double, double> victimCenter = get_center(currentVictim); // push victim center as local goal 
            generalPath.push_back(Point(victimCenter.first,victimCenter.second));
        }
        generalPath.push_back(Point(gateCenter.first, gateCenter.second));
        return generalPath;
    }//End Function


    /**
    * @brief Plan a safe and fast path in the arena
    * @param  borders        border of the arena [m]
    * @param obstacle_list  list of obstacle polygon [m]
    * @param victim_list    list of pair victim_id and polygon [m]
    * @param gate           polygon representing the gate [m]
    * @param x              x position of the robot in the arena reference system
    * @param y              y position of the robot in the arena reference system
    * @param theta          yaw of the robot in the arena reference system
    * @param config_folder  A custom string from config file.
    */
        
    bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, Path& path,const std::string& config_folder){
        auto start_overall = high_resolution_clock::now();

        /*
            Read config file to get all the parameters
        */
        config_Params_planPath config_params;
        config_params.read(config_folder);

        std::vector<Polygon> obsctacle_offset = obstacleOffsetting(obstacle_list,config_params.offset_radius);
        Polygon new_borders = resizeBorders(borders,config_params.border_radius);
        Polygon sampled_borders =borders;// sample_borders(new_borders);
        /*
            Get gate center
        */
        std::pair<double, double> gateCenter = get_center(gate);

        /*
            Get center of all victims
        */
        std::vector<Point> victimsCenter;
        for (int i = 0; i < victim_list.size(); i++){
            Polygon currentVictim = std::get<1>(victim_list[i]);
            std::pair<double, double> victimCenter = get_center(currentVictim);
            victimsCenter.emplace_back(victimCenter.first, victimCenter.second);
        }

        /*
            Get center of all obstacles and approx radius of each obstacle(considering all obstalces as circles for easy computation)
        */
        std::vector<double> obs_radius;
        std::vector<Point> obs_center;            
        for (int i = 0; i < obsctacle_offset.size(); i++){
            Polygon currentObstacle = obsctacle_offset[i];
            std::pair<double, double> obstacleCenter = get_center(currentObstacle);
            obs_center.emplace_back(obstacleCenter.first, obstacleCenter.second);
            double maxRadius = 0.0;
            for (int point = 0; point < currentObstacle.size(); point++){
                double radius = sqrt(pow(obstacleCenter.first-currentObstacle[point].x,2) + pow(obstacleCenter.second-currentObstacle[point].y,2));
                if(radius > maxRadius){
                    maxRadius = radius;
                }  
            }
            obs_radius.emplace_back(maxRadius);
        }

        /*
                    Mission planning
        */
        std::vector<Point> generalPath;
        auto start_mission = high_resolution_clock::now();
        switch(config_params.mission){
            case 0: // Generate path between source and gate without considering obstacles 
                generalPath.clear();
                obsctacle_offset.clear();
                obs_radius.clear();
                obs_center.clear();
                generalPath.push_back(Point(x,y));
                generalPath.push_back(Point(gateCenter.first, gateCenter.second));
                break;
            case 1: // Save all victims
                generalPath.push_back(Point(x,y));
                for (int i = 0; i < victimsCenter.size(); i++){
                    generalPath.push_back(victimsCenter[i]); 
                }
                generalPath.push_back(Point(gateCenter.first, gateCenter.second));
                break;
            case 2: // Save time while saving vicitms
                generalPath = compute_vicitim_mission2(x,y,theta,new_borders, obsctacle_offset, gateCenter, victim_list,config_params);
                break;
            default:
                cout<<"Mission not available. Please check config file"<<endl;
                break;
        }
        auto stop_mission = high_resolution_clock::now();
        auto duration_mission = duration_cast<milliseconds>(stop_mission - start_mission); 
        cout <<"Mission Planning time " << duration_mission.count() << endl; 
            
//        if(config_params.use_rrt)
//            RRT(x,y,theta, path, generalPath,new_borders,sampled_borders, obsctacle_offset, obs_radius, obs_center, config_params);
        if (config_params.use_rrt_star)// use own implementation RRT*
            RRT_Star(x,y,theta, path, generalPath,new_borders,sampled_borders, obsctacle_offset, obs_radius, obs_center,config_params);
        if(config_params.use_rrt_star_ompl) // Use RRT* using OMPL Library
            RRT_Star_ompl(x,y,theta, path, generalPath,new_borders,sampled_borders, obsctacle_offset, obs_radius, obs_center,config_params);

        auto stop_overall = high_resolution_clock::now();
        auto duration_overall = duration_cast<milliseconds>(stop_overall - start_overall); 
        cout <<"Overall Planning time " << duration_overall.count() << endl;

    }

}

