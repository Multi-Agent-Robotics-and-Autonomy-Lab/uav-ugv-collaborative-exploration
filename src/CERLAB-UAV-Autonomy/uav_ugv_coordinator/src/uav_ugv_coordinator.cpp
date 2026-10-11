#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>  // ADD THIS
#include <memory>
#include <thread>
#include <cmath>

// AutoFlight (we use the flightBase helpers via dynamicExploration)
#include <autonomous_flight/simulation/dynamicExploration.h>

enum class State {
  INITIAL_TAKEOFF,
  INITIAL_SCAN,
  TRIGGER_PLANNER,      // Request new paths from DEP
  WAIT_FOR_PATHS,       // Wait for BOTH UAV and UGV paths
  EXECUTE_DUAL_PATHS,   // Run both robots simultaneously
  RENDEZVOUS_LAND,      // UAV lands on UGV
  LOOP_RESET,           // Takeoff again to restart cycle
  MISSION_COMPLETE      // ADD THIS - Exploration finished
};

class Coordinator {
public:
  Coordinator(ros::NodeHandle& nh)
  : nh_(nh),
    mb_("/move_base", true),
    exploration_complete_(false)  // ADD THIS
  {
    ROS_INFO("[MAIN] Constructing Coordinator...");

    // --- Parameters ---
    nh_.param("map_frame",            map_frame_,            std::string("map"));
    nh_.param("scan_hold_sec",        scan_hold_sec_,        1.0);
    nh_.param("ugv_nav_timeout_sec",  ugv_nav_timeout_sec_,  5.0);
    nh_.param("reach_tol_xyz",        reach_tol_xyz_,        0.1);

    // Topics
    nh_.param("request_op_topic",     request_op_topic_,     std::string("/global_planner/request_op"));
    nh_.param("uav_path_topic",       uav_path_topic_,       std::string("/uav/op_path")); 
    nh_.param("ugv_path_topic",       ugv_path_topic_,       std::string("/ugv/op_path"));    
    nh_.param("ext_waypoints_topic",  ext_waypoints_topic_,  std::string("/uav_ugv_coordinator/waypoints"));
    nh_.param("completion_topic",     completion_topic_,     std::string("/exploration/complete"));  // ADD THIS

    // --- Subscribers ---
    uav_pose_sub_     = nh_.subscribe("/CERLAB/quadcopter/pose", 1, &Coordinator::uavPoseCb, this);
    uav_path_sub_     = nh_.subscribe(uav_path_topic_, 1, &Coordinator::uavPathCb, this);
    ugv_path_sub_     = nh_.subscribe(ugv_path_topic_, 1, &Coordinator::ugvPathCb, this);
    completion_sub_   = nh_.subscribe(completion_topic_, 1, &Coordinator::completionCb, this);  // ADD THIS

    

    // --- Publishers ---
    ext_waypoints_pub_ = nh_.advertise<nav_msgs::Path>(ext_waypoints_topic_, 1, true);
    request_op_pub_    = nh_.advertise<std_msgs::Empty>(request_op_topic_, 1, true);

    ROS_INFO("[Coordinator] Waiting for move_base server...");
    mb_.waitForServer();

    // ---- Construct dynamicExploration Helper ----
    ROS_INFO("[Coordinator] Initializing AutoFlight...");
    ros::NodeHandle nh_global;
    de_ = std::make_shared<AutoFlight::dynamicExploration>(nh_global);

    // NOTE: flightBase's constructor already starts publishTarget() and calls
    // detach() on it. detach() makes joinable() return false, so the guard below
    // always passed and we span up a SECOND 200Hz publisher thread racing the
    // first one on the same poseTgt_/stateTgt_. The target publisher is owned by
    // flightBase; do not start another one here.

    ROS_INFO("[Coordinator] Ready. Starting Mission.");
    change(State::INITIAL_TAKEOFF);
  }

  ~Coordinator(){
    if (de_) {
      try { de_->stop(); } catch(...) {}
      if (de_->targetPubWorker_.joinable()) de_->targetPubWorker_.detach();
    }
  }

  void spinOnce() {

    // ============================================================
    // STATE EXECUTION GUARD - Prevents duplicate execution
    // ============================================================
    static State last_executed_state = State::INITIAL_TAKEOFF;
    static bool state_executed = false;
    
    if (state_ != last_executed_state) {
        // State changed - reset the guard
        last_executed_state = state_;
        state_executed = false;
    }
    // ============================================================
    
    // Check for exploration completion
    if (exploration_complete_ && state_ != State::MISSION_COMPLETE) {
        ROS_WARN("[Coordinator] Exploration complete! Transitioning...");
        
        exploration_end_time_ = ros::Time::now();
        exploration_time_ = (exploration_end_time_ - exploration_start_time_).toSec();
        
        ROS_INFO("[Coordinator] Exploration duration: %.2f s (%.2f min)", 
                 exploration_time_, exploration_time_ / 60.0);
        
        landing_triggered_ = false;
        land_stability_start_ = ros::Time(0);
        
        change(State::MISSION_COMPLETE);
        return;
    }

    switch (state_) {
      // ---------------------------------------------------------
      // ONE-SHOT STATES (Execute only once per entry)
      // ---------------------------------------------------------
      case State::INITIAL_TAKEOFF: {
        if (state_executed) return;  // Already executed
        state_executed = true;
        
        if (!waitForUavPose(5.0)) return;
        
        if (!mission_started_) {
            mission_start_time_ = ros::Time::now();
            mission_started_ = true;
            ROS_INFO("================================================");
            ROS_INFO("[Coordinator] MISSION START");
            ROS_INFO("================================================");
        }
        
        ros::Time takeoff_start = ros::Time::now();
        ROS_INFO("[Coordinator] Initial Takeoff...");
        de_->takeoff();
        ros::Duration(5.0).sleep();
        takeoff_time_ = (ros::Time::now() - takeoff_start).toSec();
        
        change(State::INITIAL_SCAN);
      } break;

      case State::INITIAL_SCAN: {
        if (state_executed) return;
        state_executed = true;
        
        ros::Time scan_start = ros::Time::now();
        ROS_INFO("[Coordinator] Performing 360 Scan...");
        
        const double yaw_rate = 0.5; 
        de_->moveToOrientation(-M_PI/2, yaw_rate); de_->waitTime(scan_hold_sec_);
        de_->moveToOrientation(-M_PI,   yaw_rate); de_->waitTime(scan_hold_sec_);
        de_->moveToOrientation( M_PI/2, yaw_rate); de_->waitTime(scan_hold_sec_);
        de_->moveToOrientation( 0.0,    yaw_rate); de_->waitTime(scan_hold_sec_);
        
        scan_time_ = (ros::Time::now() - scan_start).toSec();

        de_->initExplore(); 
        de_->registerCallback();
        
        exploration_start_time_ = ros::Time::now();
        ROS_INFO("[Coordinator] Exploration START");
        de_->getPlanner()->startExplorationTimer();
        
        change(State::TRIGGER_PLANNER);
      } break;

      case State::TRIGGER_PLANNER: {
        if (state_executed) return;
        state_executed = true;
        
        cycle_start_time_ = ros::Time::now();
        ROS_INFO("[Coordinator] Cycle %d START", completed_cycles_ + 1);
        ROS_INFO("[Coordinator] Requesting New Plans...");
        
        have_uav_path_ = false;
        have_ugv_path_ = false;
        uav_replanned_during_exec_ = false;
        
        if (de_) {
            de_->triggerExplorationReplan();
        }
        
        change(State::WAIT_FOR_PATHS);
      } break;

      // ---------------------------------------------------------
      // CONTINUOUS STATES (Execute every spinOnce call)
      // ---------------------------------------------------------
      case State::WAIT_FOR_PATHS: {
        if (have_uav_path_ && have_ugv_path_) {
            ROS_INFO("[Coordinator] Received BOTH paths. Starting Execution.");
            change(State::EXECUTE_DUAL_PATHS);
        } else {
            ROS_INFO_THROTTLE(2.0, "[Coordinator] Waiting... UAV:%d UGV:%d", 
                              have_uav_path_, have_ugv_path_);
        }
      } break;

      case State::EXECUTE_DUAL_PATHS: {
        if (!execution_started_) {
            ext_waypoints_pub_.publish(uav_path_);
            
            ugv_wp_idx_ = 0;
            sendNextUgvWaypoint();
            
            execution_started_ = true;
            uav_finished_ = false;
            ugv_finished_ = false;
            
            last_uav_pos_ = last_pose_.pose.position;
            stuck_timer_ = ros::Time::now();

            ROS_INFO("[Coordinator] Dual execution STARTED.");
        }

        bool abort_needed = false;
        
        if (uav_replanned_during_exec_) {
            ROS_WARN("[Coordinator] UAV replan detected! Aborting.");
            abort_needed = true;
        }

        if (abort_needed) {
            mb_.cancelGoal();
            execution_started_ = false; 
            change(State::TRIGGER_PLANNER);
            return;
        }

        // Monitor UGV
        if (!ugv_finished_) {
            auto st = mb_.getState();
            if (st.isDone()) {
                if (ugv_wp_idx_ + 1 < ugv_path_.poses.size()) {
                    ugv_wp_idx_++;
                    sendNextUgvWaypoint();
                } else {
                    ROS_INFO("[Coordinator] UGV finished.");
                    ugv_finished_ = true;
                }
            } 
            else if (timedOut(ros::Duration(ugv_nav_timeout_sec_))) {
                 ROS_WARN("[Coordinator] UGV timeout. Skipping...");
                 mb_.cancelGoal();
                 if (ugv_wp_idx_ + 1 < ugv_path_.poses.size()) {
                    ugv_wp_idx_++;
                    sendNextUgvWaypoint();
                 } else {
                    ugv_finished_ = true;
                 }
            }
        }

        // Monitor UAV
        if (!uav_finished_) {
            geometry_msgs::Point target = uav_path_.poses.back().pose.position;
            if (distXYZ(last_pose_.pose.position, target) < reach_tol_xyz_) {
                ROS_INFO("[Coordinator] UAV finished.");
                uav_finished_ = true;
            }
        }

        // Check completion
        if (uav_finished_ && ugv_finished_) {
            ROS_INFO("[Coordinator] Both finished. Landing...");
            execution_started_ = false;
            landing_triggered_ = false;
            land_stability_start_ = ros::Time(0);
            change(State::RENDEZVOUS_LAND);
        }
      } break;

      case State::RENDEZVOUS_LAND: {
        mb_.cancelGoal(); 

        if (!landing_triggered_) {
             ROS_INFO("[Coordinator] Landing...");
             de_->abortTrajectory();   // stop trajExeCB fighting the descent
             de_->land();
             landing_triggered_ = true; 
        }
        
        if (have_pose_) {
            double current_z = last_pose_.pose.position.z;
            
            if (current_z < 0.75) {
                if (land_stability_start_.isZero()) {
                    land_stability_start_ = ros::Time::now();
                }
                
                if (ros::Time::now() - land_stability_start_ > ros::Duration(2.0)) {
                    double cycle_duration = (ros::Time::now() - cycle_start_time_).toSec();
                    cycle_durations_.push_back(cycle_duration);
                    completed_cycles_++;

                    // Calculate cumulative and average
                    double total_cycles_time = 0.0;
                    for (double t : cycle_durations_) {
                        total_cycles_time += t;
                    }

                    ROS_WARN("================================================");
                    ROS_WARN("[Cycle] This cycle: %.2f s (%.2f min)", 
                            cycle_duration, cycle_duration / 60.0);
                    ROS_WARN("[Cycle] Total exploration so far: %.2f s (%.2f min)", 
                            total_cycles_time, total_cycles_time / 60.0);
                    ROS_WARN("[Cycle] Average cycle time: %.2f s", 
                            total_cycles_time / completed_cycles_);
                    ROS_WARN("================================================");
                    
                    ROS_INFO("[Coordinator] Touchdown at Z=%.2f", current_z);
                    ros::Duration(5.0).sleep();
                    
                    change(State::LOOP_RESET);
                }
            } else {
                land_stability_start_ = ros::Time(0);
            }
        }
      } break;

      case State::LOOP_RESET: {
        // ============================================================
        // CRITICAL: Only execute ONCE per state entry
        // ============================================================
        if (state_executed) return;
        state_executed = true;
        // ============================================================
        
        ROS_ERROR("[Coordinator] === LOOP_RESET EXECUTING ===");
        ROS_INFO("[Coordinator] Taking off for next cycle...");
        
        // Clear any leftover trajectory from the previous cycle FIRST, otherwise
        // trajExeCB immediately overrides the takeoff hover hold with an expired
        // trajectory sample and the drone flies off to the origin.
        de_->abortTrajectory();
        de_->takeoff();
        ros::Duration(5.0).sleep();
        
        ROS_ERROR("[Coordinator] === LOOP_RESET COMPLETE ===");
        change(State::TRIGGER_PLANNER);
      } break;

      case State::MISSION_COMPLETE: {
        mb_.cancelGoal();
        
        if (!landing_triggered_) {
            landing_start_time_ = ros::Time::now();
            
            ROS_WARN("========================================");
            ROS_WARN("[Coordinator] MISSION COMPLETE!");
            ROS_WARN("[Coordinator] Landing...");
            ROS_WARN("========================================");
            
            de_->abortTrajectory();
            de_->land();
            landing_triggered_ = true;
            land_stability_start_ = ros::Time(0);
        }
        
        if (have_pose_) {
            double current_z = last_pose_.pose.position.z;
            
            if (current_z < 0.75) {
                if (land_stability_start_.isZero()) {
                    land_stability_start_ = ros::Time::now();
                }
                
                if (ros::Time::now() - land_stability_start_ > ros::Duration(2.0)) {
                    landing_time_ = (ros::Time::now() - landing_start_time_).toSec();
                    double total_mission_time = (ros::Time::now() - mission_start_time_).toSec();
                    
                    // ... print final statistics ...
                    
                    ros::Duration(3.0).sleep();
                    ros::shutdown();
                }
            } else {
                land_stability_start_ = ros::Time(0);
            }
        }
      } break;
    }
}

private:
  // ----------------- Callbacks -----------------
  void uavPoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    last_pose_ = *msg; have_pose_ = true;
  }

  void uavPathCb(const nav_msgs::Path::ConstPtr& msg){
    if (!msg || msg->poses.empty()) return;
    
    if (state_ == State::EXECUTE_DUAL_PATHS) {
        uav_replanned_during_exec_ = true;
    }

    uav_path_ = *msg; 
    have_uav_path_ = true;
    // ROS_INFO("[Coordinator] Received UAV path (%zu nodes).", uav_path_.poses.size());
  }

  void ugvPathCb(const nav_msgs::Path::ConstPtr& msg){
    if (!msg || msg->poses.empty()) return;
    ugv_path_ = *msg; 
    have_ugv_path_ = true;
    // ROS_INFO("[Coordinator] Received UGV path (%zu nodes).", ugv_path_.poses.size());
  }

  // ============================================================
  // ADD: Completion Callback
  // ============================================================
  void completionCb(const std_msgs::Bool::ConstPtr& msg){
    if (msg->data) {
        ROS_WARN("[Coordinator] Received exploration completion signal!");
        exploration_complete_ = true;
    }
  }
  // ============================================================

  // ----------------- Helpers -----------------
  bool waitForUavPose(double timeout_sec){
    ros::Time t0 = ros::Time::now(); ros::Rate r(10);
    while (ros::ok() && (ros::Time::now()-t0).toSec() < timeout_sec){
      if (have_pose_) return true;
      ros::spinOnce(); r.sleep();
    }
    return have_pose_;
  }

  void sendNextUgvWaypoint() {
    if (ugv_path_.poses.empty() || ugv_wp_idx_ >= ugv_path_.poses.size()) return;

    move_base_msgs::MoveBaseGoal g;
    g.target_pose = ugv_path_.poses[ugv_wp_idx_];
    g.target_pose.header.frame_id = map_frame_;
    g.target_pose.header.stamp = ros::Time::now();
    g.target_pose.pose.position.z = 0.0; 

    if (ugv_wp_idx_ + 1 < ugv_path_.poses.size()) {
        const auto& current_pt = ugv_path_.poses[ugv_wp_idx_].pose.position;
        const auto& next_pt    = ugv_path_.poses[ugv_wp_idx_ + 1].pose.position;
        double yaw = std::atan2(next_pt.y - current_pt.y, next_pt.x - current_pt.x);
        g.target_pose.pose.orientation = yawToQuat(yaw);
    } 
    else {
        if (!uav_path_.poses.empty()) {
            g.target_pose.pose.orientation = uav_path_.poses.back().pose.orientation;
        } else {
            g.target_pose.pose.orientation.w = 1.0; 
        }
    }

    mb_.sendGoal(g);
    state_ts_ = ros::Time::now();
    // ROS_INFO("[Coordinator] UGV Waypoint %zu/%zu sent.", ugv_wp_idx_+1, ugv_path_.poses.size());
  }

  static geometry_msgs::Quaternion yawToQuat(double yaw) {
    geometry_msgs::Quaternion q;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
  }

  static double distXYZ(const geometry_msgs::Point& a, const geometry_msgs::Point& b){
    const double dx=a.x-b.x, dy=a.y-b.y, dz=a.z-b.z;
    return std::sqrt(dx*dx+dy*dy+dz*dz);
  }

  void change(State s){ state_=s; state_ts_=ros::Time::now(); }
  bool timedOut(const ros::Duration& d) const { return (ros::Time::now()-state_ts_)>d; }

private:
  ros::NodeHandle nh_;
  std::string map_frame_{"map"};
  double scan_hold_sec_, ugv_nav_timeout_sec_, reach_tol_xyz_;

  // Topics
  std::string request_op_topic_, uav_path_topic_, ugv_path_topic_, ext_waypoints_topic_;
  std::string completion_topic_; 

  // Objects
  ros::Subscriber uav_pose_sub_, uav_path_sub_, ugv_path_sub_;
  ros::Subscriber completion_sub_;  
  ros::Publisher  ext_waypoints_pub_, request_op_pub_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> mb_;
  std::shared_ptr<AutoFlight::dynamicExploration> de_;

    // Mission timing
  bool mission_started_{false};
  ros::Time mission_start_time_;

  ros::Time landing_start_time_;
  
  double takeoff_time_{0.0};
  double scan_time_{0.0};
  double landing_time_{0.0};

  // Timing
  ros::Time exploration_start_time_;
  ros::Time exploration_end_time_;
  double exploration_time_;

  ros::Time cycle_start_time_;
  std::vector<double> cycle_durations_;
  int completed_cycles_{0};
  // State
  State state_;
  ros::Time state_ts_;
  bool landing_triggered_{false};
  ros::Time land_stability_start_;
  bool exploration_complete_{false}; 

  // Data
  geometry_msgs::PoseStamped last_pose_; bool have_pose_{false};
  nav_msgs::Path uav_path_; bool have_uav_path_{false}; bool uav_finished_{false};
  nav_msgs::Path ugv_path_; bool have_ugv_path_{false}; bool ugv_finished_{false};
  
  // Replan / Stuck Detection
  bool uav_replanned_during_exec_{false};
  geometry_msgs::Point last_uav_pos_;
  ros::Time stuck_timer_;

  size_t ugv_wp_idx_{0};
  bool execution_started_{false};
};

int main(int argc, char** argv){
  ros::init(argc, argv, "uav_ugv_coordinator");
  ros::NodeHandle nh("~");
  Coordinator c(nh);
  ros::Rate r(20);
  while (ros::ok()){
    c.spinOnce();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}