#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <memory>
#include <thread>
#include <cmath>

#include <autonomous_flight/simulation/dynamicExploration.h>

// =============================================================
// NEW Sequential State Machine:
//
//  INITIAL_TAKEOFF
//    → INITIAL_SCAN
//    → INITIAL_LAND          (UAV lands on UGV after first scan)
//    → TRIGGER_PLANNER       (request DEP plan: UGV utility path + UAV OP)
//    → WAIT_FOR_PATHS        (wait for both paths from DEP)
//    → EXECUTE_UGV_PATH      (UGV drives to target; UAV rides on top)
//    → UAV_TAKEOFF           (UAV lifts off from UGV position)
//    → EXECUTE_UAV_PATH      (UAV flies OP path; UGV stays)
//    → RENDEZVOUS_LAND       (UAV lands back on UGV)
//    → TRIGGER_PLANNER       (cycle repeats)
// =============================================================

enum class State {
  INITIAL_TAKEOFF,
  INITIAL_SCAN,
  INITIAL_LAND,
  TRIGGER_PLANNER,
  WAIT_FOR_PATHS,
  EXECUTE_UGV_PATH,
  UAV_TAKEOFF,
  EXECUTE_UAV_PATH,
  RENDEZVOUS_LAND
};

class Coordinator {
public:
  Coordinator(ros::NodeHandle& nh)
  : nh_(nh),
    mb_("/move_base", true)
  {
    ROS_INFO("[COORD] Constructing Coordinator...");

    // --- Parameters ---
    nh_.param("map_frame",            map_frame_,            std::string("map"));
    nh_.param("scan_hold_sec",        scan_hold_sec_,        1.0);
    nh_.param("ugv_nav_timeout_sec",  ugv_nav_timeout_sec_,  10.0);
    nh_.param("reach_tol_xyz",        reach_tol_xyz_,        0.3);
    nh_.param("takeoff_height",       takeoff_height_,       1.5);

    // Topics
    nh_.param("request_op_topic",    request_op_topic_,    std::string("/global_planner/request_op"));
    nh_.param("uav_path_topic",      uav_path_topic_,      std::string("/uav/op_path"));
    nh_.param("ugv_path_topic",      ugv_path_topic_,      std::string("/ugv/op_path"));
    nh_.param("ext_waypoints_topic", ext_waypoints_topic_, std::string("/uav_ugv_coordinator/waypoints"));

    // --- Subscribers ---
    uav_pose_sub_ = nh_.subscribe("/CERLAB/quadcopter/pose", 1, &Coordinator::uavPoseCb, this);
    uav_path_sub_ = nh_.subscribe(uav_path_topic_, 1, &Coordinator::uavPathCb, this);
    ugv_path_sub_ = nh_.subscribe(ugv_path_topic_, 1, &Coordinator::ugvPathCb, this);

    // --- Publishers ---
    ext_waypoints_pub_ = nh_.advertise<nav_msgs::Path>(ext_waypoints_topic_, 1, true);
    request_op_pub_    = nh_.advertise<std_msgs::Empty>(request_op_topic_, 1, true);

    ROS_INFO("[COORD] Waiting for move_base server...");
    mb_.waitForServer();

    ROS_INFO("[COORD] Initializing AutoFlight...");
    ros::NodeHandle nh_global;
    de_ = std::make_shared<AutoFlight::dynamicExploration>(nh_global);

    if (!de_->targetPubWorker_.joinable()) {
      de_->targetPubWorker_ = std::thread(&AutoFlight::flightBase::publishTarget, de_.get());
    }

    ROS_INFO("[COORD] Ready. Starting Mission.");
    change(State::INITIAL_TAKEOFF);
  }

  ~Coordinator(){
    if (de_) {
      try { de_->stop(); } catch(...) {}
      if (de_->targetPubWorker_.joinable()) de_->targetPubWorker_.detach();
    }
  }

  void spinOnce() {
    switch (state_) {

      // =========================================================
      // PHASE 1: INITIAL TAKEOFF + SCAN + LAND
      // =========================================================

      case State::INITIAL_TAKEOFF: {
        if (!waitForUavPose(5.0)) return;

        ROS_INFO("[COORD] Initial Takeoff...");
        de_->takeoff();
        ros::Duration(5.0).sleep();

        change(State::INITIAL_SCAN);
      } break;

      case State::INITIAL_SCAN: {
        ROS_INFO("[COORD] Performing 360 scan...");
        const double yr = 0.5;
        de_->moveToOrientation(-M_PI/2, yr); de_->waitTime(scan_hold_sec_);
        de_->moveToOrientation(-M_PI,   yr); de_->waitTime(scan_hold_sec_);
        de_->moveToOrientation( M_PI/2, yr); de_->waitTime(scan_hold_sec_);
        de_->moveToOrientation( 0.0,    yr); de_->waitTime(scan_hold_sec_);

        de_->initExplore();
        de_->registerCallback();

        change(State::INITIAL_LAND);
      } break;

      case State::INITIAL_LAND: {
        // Send land command once
        if (!landing_triggered_) {
          ROS_INFO("[COORD] Initial scan complete. Landing UAV onto UGV...");
          de_->land();
          landing_triggered_ = true;
        }

        // Wait for UAV to touch down (stable below 0.75m for 2s)
        if (have_pose_) {
          double z = last_pose_.pose.position.z;
          if (z < 0.75) {
            if (land_stability_start_.isZero())
              land_stability_start_ = ros::Time::now();

            if (ros::Time::now() - land_stability_start_ > ros::Duration(2.0)) {
              ROS_INFO("[COORD] UAV landed (Z=%.2f). Starting exploration cycles.", z);
              ros::Duration(2.0).sleep();
              // Reset flags and move to planning
              landing_triggered_    = false;
              land_stability_start_ = ros::Time(0);
              change(State::TRIGGER_PLANNER);
            }
          } else {
            land_stability_start_ = ros::Time(0); // reset if it bounces
          }
        }
      } break;

      // =========================================================
      // PHASE 2: PLANNING
      // =========================================================

      case State::TRIGGER_PLANNER: {
        ROS_INFO("[COORD] Requesting new DEP plan (UGV utility path + UAV OP)...");

        have_uav_path_ = false;
        have_ugv_path_ = false;

        cycle_start_time_ = ros::Time::now();
        ROS_INFO("[Coordinator] Cycle %d START", completed_cycles_ + 1);

        if (de_) {
            de_->triggerExplorationReplan();
        }

        

        change(State::WAIT_FOR_PATHS);
      } break;

      case State::WAIT_FOR_PATHS: {
        if (have_uav_path_ && have_ugv_path_) {
          ROS_INFO("[COORD] Both paths received. Sending UGV to target.");
          
          change(State::EXECUTE_UGV_PATH);
        } else {
          ROS_INFO_THROTTLE(2.0, "[COORD] Waiting for paths... UAV:%d UGV:%d",
                            have_uav_path_, have_ugv_path_);
        }
      } break;

      // =========================================================
      // PHASE 3: UGV DRIVES TO TARGET (UAV rides on top)
      // =========================================================

      case State::EXECUTE_UGV_PATH: {
        // --- A. START: send first waypoint ---
        if (!ugv_exec_started_) {
          ugv_wp_idx_ = 0;
          ugv_finished_ = false;
          sendNextUgvWaypoint();
          ugv_exec_started_ = true;
          ROS_INFO("[COORD] UGV path execution started (%zu waypoints).",
                   ugv_path_.poses.size());
        }

        // --- B. MONITOR UGV progress ---
        if (!ugv_finished_) {
          auto st = mb_.getState();
          if (st.isDone()) {
            if (ugv_wp_idx_ + 1 < ugv_path_.poses.size()) {
              ugv_wp_idx_++;
              sendNextUgvWaypoint();
            } else {
              ROS_INFO("[COORD] UGV reached target node.");
              ugv_finished_ = true;
            }
          } else if (timedOut(ros::Duration(ugv_nav_timeout_sec_))) {
            ROS_WARN("[COORD] UGV waypoint timeout. Skipping...");
            mb_.cancelGoal();
            if (ugv_wp_idx_ + 1 < ugv_path_.poses.size()) {
              ugv_wp_idx_++;
              sendNextUgvWaypoint();
            } else {
              ugv_finished_ = true;
            }
          }
        }

        // --- C. Once UGV is done → takeoff UAV ---
        if (ugv_finished_) {
          ugv_exec_started_ = false;
          ros::Duration(2.0).sleep(); // let UGV settle
          change(State::UAV_TAKEOFF);
        }
      } break;

      // =========================================================
      // PHASE 4: UAV TAKEOFF FROM UGV POSITION
      // =========================================================

      case State::UAV_TAKEOFF: {
        ROS_INFO("[COORD] UGV at target. UAV taking off...");
        de_->takeoff();
        ros::Duration(5.0).sleep();

        // Now publish the UAV OP path for execution
        ext_waypoints_pub_.publish(uav_path_);
        uav_finished_ = false;

        change(State::EXECUTE_UAV_PATH);
      } break;

      // =========================================================
      // PHASE 5: UAV EXECUTES OP PATH
      // =========================================================

      case State::EXECUTE_UAV_PATH: {
        // Monitor UAV reaching the final waypoint (= UGV position = rendezvous)
        if (!uav_path_.poses.empty()) {
          geometry_msgs::Point target = uav_path_.poses.back().pose.position;
          if (distXYZ(last_pose_.pose.position, target) < reach_tol_xyz_) {
            ROS_INFO("[COORD] UAV completed OP path. Proceeding to land.");
            uav_finished_ = true;
            landing_triggered_    = false;
            land_stability_start_ = ros::Time(0);
            change(State::RENDEZVOUS_LAND);
          }
        }
      } break;

      // =========================================================
      // PHASE 6: UAV LANDS BACK ON UGV
      // =========================================================

      case State::RENDEZVOUS_LAND: {
        mb_.cancelGoal(); // Ensure UGV is stopped

        if (!landing_triggered_) {
          ROS_INFO("[COORD] Sending land command...");
          de_->land();
          landing_triggered_ = true;
        }

        if (have_pose_) {
          double z = last_pose_.pose.position.z;
          if (z < 0.75) {
            if (land_stability_start_.isZero())
              land_stability_start_ = ros::Time::now();

            if (ros::Time::now() - land_stability_start_ > ros::Duration(2.0)) {
              ROS_INFO("[COORD] UAV landed (Z=%.2f). Cycle complete. Replanning...", z);
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
      
              ros::Duration(3.0).sleep();
              landing_triggered_    = false;
              land_stability_start_ = ros::Time(0);
              change(State::TRIGGER_PLANNER); // ← Next cycle
            }
          } else {
            land_stability_start_ = ros::Time(0);
          }
        }
      } break;
    }
  }

private:
  // ---- Callbacks ----
  void uavPoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    last_pose_ = *msg; have_pose_ = true;
  }

  void uavPathCb(const nav_msgs::Path::ConstPtr& msg){
    if (!msg || msg->poses.empty()) return;
    uav_path_     = *msg;
    have_uav_path_ = true;
    ROS_INFO("[COORD] UAV path received (%zu nodes).", uav_path_.poses.size());
  }

  void ugvPathCb(const nav_msgs::Path::ConstPtr& msg){
    if (!msg || msg->poses.empty()) return;
    ugv_path_     = *msg;
    have_ugv_path_ = true;
    ROS_INFO("[COORD] UGV path received (%zu nodes).", ugv_path_.poses.size());
  }

  // ---- Helpers ----
  bool waitForUavPose(double timeout_sec){
    ros::Time t0 = ros::Time::now(); ros::Rate r(10);
    while (ros::ok() && (ros::Time::now()-t0).toSec() < timeout_sec){
      if (have_pose_) return true;
      ros::spinOnce(); r.sleep();
    }
    return have_pose_;
  }

  void sendNextUgvWaypoint(){
    if (ugv_path_.poses.empty() || ugv_wp_idx_ >= ugv_path_.poses.size()) return;

    move_base_msgs::MoveBaseGoal g;
    g.target_pose                    = ugv_path_.poses[ugv_wp_idx_];
    g.target_pose.header.frame_id    = map_frame_;
    g.target_pose.header.stamp       = ros::Time::now();
    g.target_pose.pose.position.z    = 0.0;

    // Face toward next waypoint; at the last point face the UAV end orientation
    if (ugv_wp_idx_ + 1 < ugv_path_.poses.size()) {
      const auto& cur  = ugv_path_.poses[ugv_wp_idx_].pose.position;
      const auto& nxt  = ugv_path_.poses[ugv_wp_idx_+1].pose.position;
      double yaw = std::atan2(nxt.y - cur.y, nxt.x - cur.x);
      g.target_pose.pose.orientation = yawToQuat(yaw);
    } else {
      g.target_pose.pose.orientation.w = 1.0;
    }

    mb_.sendGoal(g);
    state_ts_ = ros::Time::now();
    // ROS_INFO("[COORD] UGV waypoint %zu/%zu sent.",
    //          ugv_wp_idx_+1, ugv_path_.poses.size());
  }

  static geometry_msgs::Quaternion yawToQuat(double yaw){
    geometry_msgs::Quaternion q;
    q.z = std::sin(yaw*0.5);
    q.w = std::cos(yaw*0.5);
    return q;
  }

  static double distXYZ(const geometry_msgs::Point& a, const geometry_msgs::Point& b){
    double dx=a.x-b.x, dy=a.y-b.y, dz=a.z-b.z;
    return std::sqrt(dx*dx+dy*dy+dz*dz);
  }

  void change(State s){ state_=s; state_ts_=ros::Time::now(); }
  bool timedOut(const ros::Duration& d) const { return (ros::Time::now()-state_ts_)>d; }

private:
  ros::NodeHandle nh_;
  std::string map_frame_{"map"};
  double scan_hold_sec_, ugv_nav_timeout_sec_, reach_tol_xyz_, takeoff_height_;

  std::string request_op_topic_, uav_path_topic_, ugv_path_topic_, ext_waypoints_topic_;

  ros::Subscriber uav_pose_sub_, uav_path_sub_, ugv_path_sub_;
  ros::Publisher  ext_waypoints_pub_, request_op_pub_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> mb_;
  std::shared_ptr<AutoFlight::dynamicExploration> de_;

  // State machine
  State     state_;
  ros::Time state_ts_;

  // Landing helpers
  bool      landing_triggered_{false};
  ros::Time land_stability_start_;

  // Pose
  geometry_msgs::PoseStamped last_pose_;
  bool have_pose_{false};

  ros::Time cycle_start_time_;
  std::vector<double> cycle_durations_;
  int completed_cycles_{0};
  // UAV path
  nav_msgs::Path uav_path_;
  bool have_uav_path_{false};
  bool uav_finished_{false};

  // UGV path
  nav_msgs::Path ugv_path_;
  bool have_ugv_path_{false};
  bool ugv_finished_{false};
  bool ugv_exec_started_{false};
  size_t ugv_wp_idx_{0};
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