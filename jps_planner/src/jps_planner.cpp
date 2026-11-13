#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <algorithm>
#include <cmath>
#include <vector>
#include <memory>
#include <cstdint>
#include <queue>
#include <chrono>
#include <unordered_map>
#include <functional>
#include <utility>

class planner : public rclcpp::Node{
public:
    planner() : Node("jps_planner"){
        allow_unknown_ = declare_parameter("allow_unknown",false);
        blocked_cost_threshold_ = declare_parameter("blocked_cost_threshold",50);
        start_x_ = declare_parameter("start_x",0.0);
        start_y_ = declare_parameter("start_y",0.0);
        costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/global_costmap/costmap",rclcpp::QoS(10).transient_local(),
            std::bind(&planner::costmapCallback,this,std::placeholders::_1)
        );
        goal_sub_ =create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose",10,std::bind(&planner::goalCallback,this,std::placeholders::_1)
        );
        path_pub_ = create_publisher<nav_msgs::msg::Path>("/jps_plan",10);
        plan_timer_ = create_wall_timer(std::chrono::milliseconds(200),std::bind(&planner::planloop,this));
        RCLCPP_INFO(
            get_logger(),"planner is ready,start x=%.2f,y=%.2f",start_x_,start_y_
        );
    }
    bool map_received_ = false;
    bool goal_received_ =false;
    double goal_x_ = 0.0;
    double goal_y_ =0.0;
    //接收地图
    void updatecostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        if(!msg||msg->info.width==0||msg->info.height==0||msg->info.resolution<=0) return;
        const size_t cell_count = static_cast<size_t>(msg->info.width)*msg->info.height;
        if(cell_count!=msg->data.size()) return;
        width_=static_cast<int>(msg->info.width);
        height_=static_cast<int>(msg->info.height);
        resolution_=msg->info.resolution;
        origin_x_=msg->info.origin.position.x;
        origin_y_=msg->info.origin.position.y;
        free_cells_.resize(cell_count);
        for(size_t i=0;i<cell_count;i++){
            const int8_t c=msg->data[i];
            const bool blocked = (c<0) ? (!allow_unknown_) :(c>=blocked_cost_threshold_);
            free_cells_[i]=(!blocked);
        }
        map_received_ = true ;
        RCLCPP_INFO(
            get_logger(),"map received,%d*%d,resolution=%.3f,origin(%.2f,%.2f)",
            width_,height_,resolution_,origin_x_,origin_y_
        );

    }
    //1能通行，0不能
    bool isfree(int x,int y) const{
        if(x<0||y<0||x>=width_||y>=height_){
            return false;
        }
        return free_cells_[static_cast<size_t>(y)*width_+x];
    }
    //栅格世界坐标互换
    void worldtogrid(double wx,double wy,int &gx,int &gy) const{
        gx = static_cast<int>(std::floor((wx-origin_x_)/resolution_));
        gy = static_cast<int>(std::floor((wy-origin_y_)/resolution_));
    }
    void gridtoworld(int gx,int gy,double &wx,double &wy) const{
        wx = origin_x_ + static_cast<double>(gx+0.5)*resolution_;
        wy = origin_y_ + static_cast<double>(gy+0.5)*resolution_;
    }
    //计算后续代价
    static double heuristic(int x,int y,int gx,int gy){
        const double dx =std::abs(x-gx);
        const double dy =std::abs(y-gy);
        return std::max(dx,dy)+(std::sqrt(2.0)-1.0)*std::min(dx,dy);
    }
    int width_=0;
    int height_=0;
    double resolution_=0.0;
    double origin_x_=0.0;
    double origin_y_=0.0;
    std::vector<uint8_t> free_cells_;
    int blocked_cost_threshold_ =90;
    bool allow_unknown_ =false;
    double start_x_ =0.0;
    double start_y_ =0.0;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr plan_timer_;

    struct PairHash{
        std::size_t operator()(const std::pair<int,int>  & point)const{
            const auto first_hash = std::hash<int>{}(point.first);
            const auto second_hash = std::hash<int>{}(point.second);
            return first_hash ^ (second_hash << 1);
        }
    };
    //判断(nx,ny)是否为跳点
    std::pair<int,int> jump(int x,int y ,int dx,int dy,int goal_x,int goal_y){
        const int nx =x+dx;
        const int ny =y+dy;
        if(!isfree(nx,ny)){
            return {-1,-1};
        }
        if(nx==goal_x&&ny==goal_y){
            return {nx,ny};
        }
        if(dx!=0&&dy!=0){
            if(!isfree(nx,ny-dy)&&isfree(nx+dx,ny-dy)){
                return {nx,ny};
            }
            if(!isfree(nx-dx,ny)&&isfree(nx-dx,ny+dy)){
                return {nx,ny};
            }
            if(jump(nx,ny,dx,0,goal_x,goal_y).first!=-1){
                return {nx,ny};
            }
            if(jump(nx,ny,0,dy,goal_x,goal_y).first!=-1){
                return {nx,ny};
            }
        }else if(dx!=0){
            if(!isfree(nx,ny+1)&&isfree(nx+dx,ny+1)){
                return {nx,ny};
            }
            if(!isfree(nx,ny-1)&&isfree(nx+dx,ny-1)){
                return {nx,ny};
            }
        }
        else{
            if(!isfree(nx+1,ny)&&isfree(nx+1,ny+dy)){
                return {nx,ny};
            }
            if(!isfree(nx-1,ny)&&isfree(nx-1,ny+dy)){
                return {nx,ny};
            }
        }
        return jump(nx,ny,dx,dy,goal_x,goal_y);
    }
    //当前跳点的候选跳点
    std::vector<std::pair<int,int>> candidate
    (int x,int y,int px,int py,int goal_x,int goal_y){
        std::vector<std::pair<int,int>> cjp;
        int dx=x-px;
        int dy=y-py;
        if(dx!=0){
            dx/=std::abs(dx);
        }
        if(dy!=0){
            dy/=std::abs(dy);
        }
        if(dx==0&&dy==0){
            for(int ddx=-1;ddx<=1;ddx++){
                for(int ddy=-1;ddy<=1;ddy++){
                    if(ddx==0&&ddy==0){
                        continue;
                    }
                    auto jp=jump(x,y,ddx,ddy,goal_x,goal_y);
                    if(jp.first!=-1){
                        cjp.push_back(jp);
                    }
                }
            }
        }else if(dx!=0&&dy!=0){
            if(isfree(x+dx,y)){
                auto jp=jump(x,y,dx,0,goal_x,goal_y);
                if(jp.first!=-1){
                    cjp.push_back(jp);
                }
            }
            if(isfree(x,y+dy)){
                auto jp=jump(x,y,0,dy,goal_x,goal_y);
                if(jp.first!=-1){
                    cjp.push_back(jp);
                }
            }
            if(isfree(x+dx,y+dy)){
                auto jp=jump(x,y,dx,dy,goal_x,goal_y);
                if(jp.first!=-1){
                    cjp.push_back(jp);
                }
            }
        }else if(dx!=0){
            if(isfree(x+dx,y)){
                auto jp=jump(x,y,dx,0,goal_x,goal_y);
                if(jp.first!=-1){
                    cjp.push_back(jp);
                }
            }
            if(!isfree(x,y+1)&&isfree(x+dx,y+1)){
                auto jp=jump(x,y,dx,1,goal_x,goal_y);
                if(jp.first!=-1){
                    cjp.push_back(jp);
                }
            }
            if(!isfree(x,y-1)&&isfree(x+dx,y-1)){
                auto jp=jump(x,y,dx,-1,goal_x,goal_y);
                if(jp.first!=-1){
                    cjp.push_back(jp);
                }
            }
        }else{
            if(isfree(x,y+dy)){
                auto jp=jump(x,y,0,dy,goal_x,goal_y);
                if(jp.first!=-1){
                    cjp.push_back(jp);
                }
            }
            if(!isfree(x+1,y)&&isfree(x+1,y+dy)){
                auto jp=jump(x,y,1,dy,goal_x,goal_y);
                if(jp.first!=-1){
                    cjp.push_back(jp);
                }
            }
            if(!isfree(x-1,y)&&isfree(x-1,y+dy)){
                auto jp=jump(x,y,-1,dy,goal_x,goal_y);
                if(jp.first!=-1){
                    cjp.push_back(jp);
                }
            }
        }
        return cjp;
    }
    //生成路径
    std::vector<std::pair<int,int>> search(int start_x,int start_y,int goal_x,int goal_y){
        std::priority_queue<std::pair<double,std::pair<int,int>>,
                            std::vector<std::pair<double,std::pair<int,int>>>,
                            std::greater<std::pair<double,std::pair<int,int>>>> open;
        std::unordered_map<std::pair<int,int>,std::pair<int,int>,PairHash> parent; 
        std::unordered_map<std::pair<int,int>,double,PairHash> g_cost;
        open.push({0.0,{start_x, start_y}});
        parent[{start_x,start_y}] ={start_x,start_y};
        g_cost[{start_x,start_y}] =0.0;
        while(!open.empty()){
            auto current = open.top().second;
            open.pop();
            if(current.first==goal_x&&current.second==goal_y){
                std::vector<std::pair<int,int>> path;
                auto node =current;
                while(!(node.first==start_x&&node.second==start_y)){
                    path.push_back(node);
                    node=parent[node];
                }
                path.push_back({start_x,start_y});
                std::reverse(path.begin(),path.end());
                return path;
            }
            int px =parent[current].first;
            int py =parent[current].second;
            auto cjp = candidate(current.first,current.second,px,py,goal_x,goal_y);
            for(auto & nxt : cjp){
                double g_1 =std::sqrt
                ((nxt.first-current.first)*(nxt.first-current.first)+
                (nxt.second-current.second)*(nxt.second-current.second));
                double g=g_1+g_cost[current];
                if(g_cost.find(nxt)==g_cost.end()||g<g_cost[nxt]){
                    g_cost[nxt]=g;
                    double f=g+heuristic(nxt.first,nxt.second,goal_x,goal_y);
                    open.push({f,nxt});
                    parent[nxt]=current;
                }
            }
        }
        return {};
    }
    //生成世界坐标路径
    void publishpath(const std::vector<std::pair<int,int>> &grid_path){
        if(grid_path.empty()){
            return;
        }
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp=this->now();
        path_msg.header.frame_id ="map";
        const double e = 0.1;
        const double z = 0.02;
        std::vector<geometry_msgs::msg::PoseStamped> interpolated;
        interpolated.reserve(grid_path.size()*2);
        double prev_wx =0.0;
        double prev_wy =0.0;
        gridtoworld(grid_path[0].first,grid_path[0].second,prev_wx,prev_wy);
        geometry_msgs::msg::PoseStamped prev_pose;
        prev_pose.header=path_msg.header;
        prev_pose.pose.position.x=prev_wx;
        prev_pose.pose.position.y=prev_wy;
        prev_pose.pose.position.z=z;
        prev_pose.pose.orientation.w=1.0;
        interpolated.push_back(prev_pose);
        for(size_t i=1;i<grid_path.size();i++){
            double curr_wx=0.0;
            double curr_wy=0.0;
            gridtoworld(grid_path[i].first,grid_path[i].second,curr_wx,curr_wy);
            double dx =curr_wx-prev_wx;
            double dy =curr_wy-prev_wy;
            double length =std::hypot(dx,dy);
            if(length>e){
                int num_insertions =static_cast<int>(std::floor(length/e));
                double step_x = dx/(num_insertions+1);
                double step_y = dy/(num_insertions+1);
                for(int j=1;j<=num_insertions;j++){
                    geometry_msgs::msg::PoseStamped mid_pose;
                    mid_pose.header=path_msg.header;
                    mid_pose.pose.position.x=prev_wx+j*step_x;
                    mid_pose.pose.position.y=prev_wy+j*step_y;
                    mid_pose.pose.position.z=z;
                    mid_pose.pose.orientation.w=1.0;
                    interpolated.push_back(mid_pose);
                }
            }
            geometry_msgs::msg::PoseStamped curr_pose;
            curr_pose.header=path_msg.header;
            curr_pose.pose.position.x=curr_wx;
            curr_pose.pose.position.y=curr_wy;
            curr_pose.pose.position.z=z;
            curr_pose.pose.orientation.w=1.0;
            interpolated.push_back(curr_pose);
            prev_wx=curr_wx;
            prev_wy=curr_wy;
        }

        const size_t point_count = interpolated.size();
        std::vector<bool> direction_changes(point_count, false);
        for(size_t i=1;i+1<point_count;i++){
            const double in_x = interpolated[i].pose.position.x -
                interpolated[i-1].pose.position.x;
            const double in_y = interpolated[i].pose.position.y -
                interpolated[i-1].pose.position.y;
            const double out_x = interpolated[i+1].pose.position.x -
                interpolated[i].pose.position.x;
            const double out_y = interpolated[i+1].pose.position.y -
                interpolated[i].pose.position.y;
            const double in_length = std::hypot(in_x,in_y);
            const double out_length = std::hypot(out_x,out_y);
            if(in_length==0.0||out_length==0.0){
                continue;
            }
            const double direction_difference = std::hypot(
                in_x/in_length-out_x/out_length,
                in_y/in_length-out_y/out_length);
            direction_changes[i] = direction_difference>1e-6;
        }
        std::vector<size_t> corners;
        for(size_t i=1;i+1<point_count;i++){
            if(direction_changes[i]){
                corners.push_back(i);
            }
        }
        std::vector<bool> smoothed_window(point_count, false);
        for(size_t corner_index=0;corner_index<corners.size();corner_index++){
            const size_t i = corners[corner_index];
            const size_t parent_corner = corner_index==0 ? 0 : corners[corner_index-1];
            const size_t child_corner = corner_index+1==corners.size() ?
                point_count-1 : corners[corner_index+1];
            size_t first_index = parent_corner+1;
            size_t last_index = child_corner-1;
            if(first_index>=i||last_index<=i){
                continue;
            }
            const size_t max_smoothing_points = 20;
            if(last_index-first_index+1>max_smoothing_points){
                const size_t points_before = std::min(i-first_index,(max_smoothing_points-1)/2);
                const size_t points_after = std::min(last_index-i,max_smoothing_points/2);
                first_index = i-points_before;
                last_index = i+points_after;
            }
            bool overlaps = false;
            for(size_t index=first_index;index<=last_index;index++){
                if(smoothed_window[index]){
                    overlaps = true;
                    break;
                }
            }
            if(overlaps){
                continue;
            }
            const auto & start = interpolated[first_index].pose.position;
            const auto & control = interpolated[i].pose.position;
            const auto & end = interpolated[last_index].pose.position;
            const size_t sample_count = last_index-first_index+1;
            std::vector<std::pair<double,double>> curve;
            curve.reserve(sample_count);
            for(size_t j=0;j<sample_count;j++){
                const double t = static_cast<double>(j)/(sample_count-1);
                const double u = 1.0-t;
                curve.emplace_back(
                    u*u*start.x+2.0*u*t*control.x+t*t*end.x,
                    u*u*start.y+2.0*u*t*control.y+t*t*end.y);
            }
            bool collision = false;
            const double check_step = resolution_*0.5;
            for(size_t j=1;j<curve.size()&&!collision;j++){
                const double dx = curve[j].first-curve[j-1].first;
                const double dy = curve[j].second-curve[j-1].second;
                const double length = std::hypot(dx,dy);
                const int steps = std::max(
                    1,static_cast<int>(std::ceil(length/check_step)));
                for(int k=0;k<=steps;k++){
                    const double t = static_cast<double>(k)/steps;
                    int gx=0;
                    int gy=0;
                    worldtogrid(
                        curve[j-1].first+t*dx,
                        curve[j-1].second+t*dy,
                        gx,gy);
                    if(!isfree(gx,gy)){
                        collision = true;
                        break;
                    }
                }
            }
            if(collision){
                continue;
            }
            for(size_t j=0;j<curve.size();j++){
                const size_t index = first_index+j;
                interpolated[index].pose.position.x = curve[j].first;
                interpolated[index].pose.position.y = curve[j].second;
                smoothed_window[index] = true;
            }
        }
        path_msg.poses=std::move(interpolated);
        path_pub_->publish(path_msg);
        const auto & first = path_msg.poses.front();
        const auto & last = path_msg.poses.back();
        RCLCPP_INFO(
            get_logger(),"path publish,%zu poses,(%.3f,%.3f)->(%.3f,%.3f),frame='%s'.",
            path_msg.poses.size(),first.pose.position.x,first.pose.position.y,last.pose.position.x,last.pose.position.y,
            path_msg.header.frame_id.c_str()
        );
    }
    //回调
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        updatecostmap(msg);
    }
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        goal_x_ = msg->pose.position.x;
        goal_y_ = msg->pose.position.y;
        goal_received_ = true;
        RCLCPP_INFO(
            get_logger(),"goal received.frame='%s',position=(%.3f,%.3f)",msg->header.frame_id.c_str(),goal_x_,goal_y_
        );
    }
    //路径规划全流程并计算路长
    void planloop(){
        if(!map_received_ || !goal_received_){
            return;
        }
        int start_gx =0;
        int start_gy =0;
        int goal_gx =0;
        int goal_gy =0;
        worldtogrid(start_x_,start_y_,start_gx,start_gy);
        worldtogrid(goal_x_,goal_y_,goal_gx,goal_gy);
        const auto path = search(start_gx,start_gy,goal_gx,goal_gy);
        if(path.empty()){
            RCLCPP_INFO(
                get_logger(),"no path"
            );
            goal_received_ =false;
            return;
        }
        double path_start_x_ =0.0;
        double path_start_y_ =0.0;
        gridtoworld(path.front().first,path.front().second,path_start_x_,path_start_y_);
        double previous_x =path_start_x_;
        double previous_y =path_start_y_;
        double path_length_ =0.0;
        for(size_t i=1;i<path.size();i++){
            double point_x =0.0;
            double point_y =0.0;
            gridtoworld(path[i].first,path[i].second,point_x,point_y);
            path_length_ += std::hypot(previous_x-point_x,previous_y-point_y);
            previous_x=point_x;
            previous_y=point_y;
        }
        RCLCPP_INFO(
            get_logger(),"path found (%.3f,%.3f)->(%.3f,%.3f) , path length = %.3f",
            path_start_x_,path_start_y_,previous_x,previous_y,path_length_
        );
        publishpath(path);
        goal_received_ =false;
    }
    
};
int main(int argc,char ** argv){
    rclcpp::init(argc,argv);
    auto node =std::make_shared<planner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
