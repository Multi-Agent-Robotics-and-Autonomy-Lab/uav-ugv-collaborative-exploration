/*
	FILE: flightBase.h
	-------------------------
	implementation of flight base
*/
#include <autonomous_flight/simulation/flightBase.h>


namespace AutoFlight{
	flightBase::flightBase(const ros::NodeHandle& nh) : nh_(nh){
    	// parameters    	
		if (not this->nh_.getParam("autonomous_flight/takeoff_height", this->takeoffHgt_)){
			this->takeoffHgt_ = 2.0;

			cout << "[AutoFlight]: No takeoff height param found. Use default: 1.0 m." << endl;
		}
		else{
			cout << "[AutoFlight]: Takeoff Height: " << this->takeoffHgt_ << "m." << endl;
		}

		// subscriber
		this->odomSub_ = this->nh_.subscribe("/CERLAB/quadcopter/odom", 1000, &flightBase::odomCB, this);
		this->clickSub_ = this->nh_.subscribe("/move_base_simple/goal", 1000, &flightBase::clickCB, this);
		

		// publisher
		this->posePub_ = this->nh_.advertise<geometry_msgs::PoseStamped>("/CERLAB/quadcopter/setpoint_pose", 1000);
		this->statePub_ = this->nh_.advertise<tracking_controller::Target>("/autonomous_flight/target_state", 1000);
		this->landPub_ = this->nh_.advertise<std_msgs::Empty>("/CERLAB/quadcopter/land", 1);



		ros::Rate r (10);
		while (ros::ok() and not this->odomReceived_){
			ros::spinOnce();
			r.sleep();
		}
		cout << "[AutoFlight]: Odom and mavros topics are ready." << endl;

		// Seed the pose target with the CURRENT pose ("hold position") before the
		// 200Hz publisher thread starts. Without this, poseTgt_ is all zeros and
		// the drone is commanded to the world origin the instant the thread runs.
		{
			std::lock_guard<std::mutex> lk(this->targetMtx_);
			this->poseTgt_.pose = this->odom_.pose.pose;
			this->poseTgt_.header.frame_id = "map";
			this->poseTgtInit_ = true;
		}
	
    	// Tareget publish thread
		this->targetPubWorker_ = std::thread(&flightBase::publishTarget, this);
		this->targetPubWorker_.detach();


		// state update callback
		this->stateUpdateTimer_ = this->nh_.createTimer(ros::Duration(0.033), &flightBase::stateUpdateCB, this);
	}


	void flightBase::publishTarget(){
		ros::Rate r (200);
		while (ros::ok()){
			// Take a STABLE snapshot under the lock, then publish outside it.
			// Publishing directly from the shared members races with
			// updateTarget()/updateTargetWithState() and overruns the
			// serialization buffer (StreamOverrunException -> abort).
			bool poseMode, posePublishable;
			geometry_msgs::PoseStamped poseCopy;
			tracking_controller::Target stateCopy;
			{
				std::lock_guard<std::mutex> lk(this->targetMtx_);
				poseMode = this->poseControl_;
				// Safety gate 1: never command a pose target that was never set
				// (a zero poseTgt_ would fly the drone to the world origin).
				// Safety gate 2: do NOT publish setpoint_pose while landing - the
				// plugin's TargetPoseCallback unconditionally forces
				// navi_state = FLYING_MODEL, which would cancel the LANDING_MODEL
				// set by land() within ~5ms so the drone could never come down.
				posePublishable = this->poseTgtInit_ && !this->landing_in_progress_;
				if (poseMode) poseCopy  = this->poseTgt_;
				else          stateCopy = this->stateTgt_;
			}

			if (poseMode){
				if (posePublishable) this->posePub_.publish(poseCopy);
			}
			else{
				this->statePub_.publish(stateCopy);
			}
			r.sleep();
		}
	}


	void flightBase::clickCB(const geometry_msgs::PoseStamped::ConstPtr& cp){
		this->goal_ = *cp;
		this->goal_.pose.position.z = this->takeoffHgt_;
		if (not this->firstGoal_){
			this->firstGoal_ = true;
		}

		if (not this->goalReceived_){
			this->goalReceived_ = true;
		}
	}


	void flightBase::odomCB(const nav_msgs::OdometryConstPtr& odom){
		this->odom_ = *odom;
		this->lastOdomTime_ = ros::Time::now();   // freshness, used by takeoff()
		if (this->odomReceived_ == false){
			this->odomReceived_ = true;
		}
	}

	void flightBase::stateUpdateCB(const ros::TimerEvent&){
		Eigen::Vector3d currVelBody (this->odom_.twist.twist.linear.x, this->odom_.twist.twist.linear.y, this->odom_.twist.twist.linear.z);
		Eigen::Vector4d orientationQuat (this->odom_.pose.pose.orientation.w, this->odom_.pose.pose.orientation.x, this->odom_.pose.pose.orientation.y, this->odom_.pose.pose.orientation.z);
		Eigen::Matrix3d orientationRot = AutoFlight::quat2RotMatrix(orientationQuat);
		this->currVel_ = orientationRot * currVelBody;

		// Guard against a garbage/non-finite velocity (e.g. a zero quaternion in
		// odom makes quat2RotMatrix produce NaN). Never let it reach the planner.
		if (not this->currVel_.allFinite()){
			ROS_WARN_THROTTLE(2.0, "[FlightBase] Non-finite velocity from odom - ignoring sample.");
			return;
		}

		ros::Time currTime = ros::Time::now();
		if (this->stateUpdateFirstTime_){
			this->currAcc_ = Eigen::Vector3d (0.0, 0.0, 0.0);
			this->prevVel_ = this->currVel_;   // was left UNINITIALISED (Eigen does
			                                   // not zero-init) -> garbage accel on
			                                   // the very next call.
			this->prevStateTime_ = currTime;
			this->stateUpdateFirstTime_ = false;
		}
		else{
			double dt = (currTime - this->prevStateTime_).toSec();

			// Under sim time this 30Hz timer can fire twice inside the same /clock
			// tick, giving dt == 0 -> (v-v)/0 = NaN (or inf). That NaN propagates
			// into the B-spline start conditions, A* casts it to int (INT_MIN) and
			// you get "Ran out of pool, index=-2147483598". Skip degenerate dt.
			if (dt < 1e-4){
				return;   // keep previous currAcc_; try again next tick
			}

			Eigen::Vector3d acc = (this->currVel_ - this->prevVel_)/dt;
			if (acc.allFinite()){
				this->currAcc_ = acc;
			}
			else{
				ROS_WARN_THROTTLE(2.0, "[FlightBase] Non-finite acceleration (dt=%.6f) - holding previous.", dt);
			}
			this->prevVel_ = this->currVel_;
			this->prevStateTime_ = currTime;
		}
	}


	// void flightBase::takeoff(){
	// 	if (this->odom_.pose.pose.position.z >= 1.2){
	// 		// std::cout << "[odom pose z]: " << this->odom_.pose.pose.position.z << std::endl;
	// 		this->hasTakeoff_ = true;
	// 	}

	// 	geometry_msgs::PoseStamped ps;
	// 	ps.pose = this->odom_.pose.pose;
	// 	// std::cout << "[odom pose z]: " << this->odom_.pose.pose.position.z << std::endl;
	// 	ps.pose.position.z = this->takeoffHgt_;
	// 	ps.pose.orientation.w = 1.0;
	// 	this->updateTarget(ps);
	// 	ros::Rate r (30);
	// 	while (ros::ok() and std::abs(this->odom_.pose.pose.position.z - this->takeoffHgt_) >= 0.1 and not this->hasTakeoff_){
	// 		ros::spinOnce();
	// 		r.sleep();
	// 	}
	// 	this->hasTakeoff_ = true;
	// }
 
	// void flightBase::takeoff(){
	// 	static int takeoff_count = 0;
	// 	takeoff_count++;
		
	// 	ROS_WARN("[FlightBase] TAKEOFF called (count: %d), current Z: %.2fm, hasTakeoff_=%d", 
	// 			takeoff_count, this->odom_.pose.pose.position.z, this->hasTakeoff_);
		
	// 	// ============================================================
	// 	// FIX 1: Always reset hasTakeoff_ at start of takeoff
	// 	// ============================================================
	// 	this->hasTakeoff_ = false;
	// 	// ============================================================
		
	// 	// Check if already at target altitude
	// 	if (this->odom_.pose.pose.position.z >= this->takeoffHgt_ - 0.1){
	// 		ROS_WARN("[FlightBase] Already airborne (Z=%.2f >= %.2f). Skipping.",
	// 				this->odom_.pose.pose.position.z, this->takeoffHgt_);
	// 		this->hasTakeoff_ = true;
	// 		return;
	// 	}
		
	// 	geometry_msgs::PoseStamped ps;
	// 	ps.pose = this->odom_.pose.pose;
	// 	ps.pose.position.z = this->takeoffHgt_;
	// 	ps.pose.orientation.w = 1.0;
		
	// 	ROS_INFO("[FlightBase] Publishing takeoff target: Z=%.2f", this->takeoffHgt_);
	// 	this->updateTarget(ps);
		
	// 	ros::Rate r(30);
	// 	int wait_iterations = 0;
		
	// 	// ============================================================
	// 	// FIX 2: Add timeout and better logging
	// 	// ============================================================
	// 	ros::Time start_time = ros::Time::now();
	// 	const double TIMEOUT = 3.0;  // 10 second timeout
		
	// 	while (ros::ok() && std::abs(this->odom_.pose.pose.position.z - this->takeoffHgt_) >= 0.1 && !this->hasTakeoff_){
	// 		this->updateTarget(ps);
	// 		double current_z = this->odom_.pose.pose.position.z;
	// 		double error = std::abs(current_z - this->takeoffHgt_);
			
	// 		// Log progress
	// 		if (wait_iterations++ % 30 == 0) {  // Every second
	// 			ROS_INFO("[FlightBase] Takeoff progress: Z=%.2f (target=%.2f, error=%.2f)",
	// 					current_z, this->takeoffHgt_, error);
	// 		}
			
	// 		// Check timeout
	// 		if ((ros::Time::now() - start_time).toSec() > TIMEOUT) {
	// 			ROS_ERROR("[FlightBase] Takeoff TIMEOUT! Z=%.2f after %.1fs",
	// 					current_z, TIMEOUT);
	// 			break;
	// 		}
			
	// 		// Check if reached target
	// 		if (error < 0.1) {
	// 			ROS_INFO("[FlightBase] Target altitude reached!");
	// 			break;
	// 		}
			
	// 		ros::spinOnce();
	// 		r.sleep();
	// 	}
	// 	// ============================================================
		
	// 	this->hasTakeoff_ = true;
	// 	ROS_INFO("[FlightBase] Takeoff complete. Final Z: %.2f (target was %.2f)", 
	// 			this->odom_.pose.pose.position.z, this->takeoffHgt_);
		
	// 	// ============================================================
	// 	// FIX 3: Add stabilization wait
	// 	// ============================================================
	// 	ROS_INFO("[FlightBase] Stabilizing for 2 seconds...");
	// 	ros::Duration(2.0).sleep();
	// 	ROS_INFO("[FlightBase] Stabilization complete. Z: %.2f", this->odom_.pose.pose.position.z);
	// 	// ============================================================
	// }

	void flightBase::takeoff(){
		this->hasTakeoff_ = false;

		// The hover target below is built from odom_. If odometry is stale or was
		// never received, odom_ is (0,0,0) and we would command the drone to the
		// WORLD ORIGIN and then wedge. Wait for a fresh sample first.
		{
			ros::Rate wr(20);
			ros::Time waitStart = ros::Time::now();
			while (ros::ok()){
				const bool fresh = this->odomReceived_ &&
				                   !this->lastOdomTime_.isZero() &&
				                   (ros::Time::now() - this->lastOdomTime_).toSec() < 1.0;
				if (fresh) break;
				if ((ros::Time::now() - waitStart).toSec() > 5.0){
					ROS_ERROR("[FlightBase] takeoff ABORTED: no fresh odometry on "
					          "/CERLAB/quadcopter/odom. Refusing to fly to the origin.");
					return;   // hasTakeoff_ stays false - caller can detect failure
				}
				ros::spinOnce();
				wr.sleep();
			}
		}

		if (this->odom_.pose.pose.position.z >= 1.0){
			this->hasTakeoff_ = true;
			return;
		}
		
		geometry_msgs::PoseStamped ps;
		ps.pose = this->odom_.pose.pose;
		
		// ============================================================
		// FIX: Use lower target to account for overshoot
		// ============================================================
		ps.pose.position.z = 0.9;  // Lower from 1.4 to 1.1
		// The controller will overshoot to ~1.3-1.4m
		// ============================================================
		
		ps.pose.orientation.w = 1.0;
		
		this->updateTarget(ps);
		
		ros::Rate r(30);
		ros::Time start_time = ros::Time::now();
		
		while (ros::ok()){
			this->updateTarget(ps);
			
			double current_z = this->odom_.pose.pose.position.z;
			double error = std::abs(current_z - ps.pose.position.z);
			
			// Exit when CLOSE to target (not exact)
			if (current_z >= 0.9) {  // Exit early at 1.0m
				ROS_INFO("[FlightBase] Reached 1.0m, stopping ascent command");
				break;
			}
			
			if ((ros::Time::now() - start_time).toSec() > 5.0) {
				// Previously this fell through and set hasTakeoff_ = true anyway,
				// so the coordinator FSM advanced believing the drone was airborne
				// when it was still on the ground - and the mission silently wedged.
				ROS_ERROR("[FlightBase] takeoff TIMED OUT after 5s (z=%.2f, target=%.2f). "
				          "Not reporting takeoff.", current_z, ps.pose.position.z);
				return;   // hasTakeoff_ stays false
			}
			
			ros::spinOnce();
			r.sleep();
		}
		
		this->hasTakeoff_ = true;
		
		// ============================================================
		// ADD: Wait and monitor for overshoot
		// ============================================================
		ROS_INFO("[FlightBase] Stabilizing...");
		ros::Duration(3.0).sleep();  // Longer stabilization
		
		double final_z = this->odom_.pose.pose.position.z;
		ROS_INFO("[FlightBase] Takeoff complete. Final Z: %.2f", final_z);
		
		// If overshot, descend back
		if (final_z > 1.6) {
			ROS_WARN("[FlightBase] Overshoot detected (Z=%.2f)! Descending to 1.3m", final_z);
			
			ps.pose.position.z = 1.3;
			for (int i = 0; i < 90; ++i) {  // 3 seconds at 30Hz
				this->updateTarget(ps);
				ros::spinOnce();
				r.sleep();
			}
			
			ROS_INFO("[FlightBase] Corrected to Z: %.2f", this->odom_.pose.pose.position.z);
		}
		// ============================================================
	}


	void flightBase::run(){
		// flight test with circle
		double r; // radius
		double v; // circle velocity
    	
    	// track circle radius parameters    	
		if (not this->nh_.getParam("autonomous_flight/radius", r)){
			r = 2.0;
			cout << "[AutoFlight]: No circle radius param found. Use default: 2.0 m." << endl;
		}
		else{
			cout << "[AutoFlight]: Circle radius: " << r << "m." << endl;
		}

    	// track circle velocity parameters    	
		if (not this->nh_.getParam("autonomous_flight/circle_velocity", v)){
			v = 1.0;
			cout << "[AutoFlight]: No circle velocity param found. Use default: 1.0 m/s." << endl;
		}
		else{
			cout << "[AutoFlight]: Circle velocity: " << v << "m/s." << endl;
		}

		double z = this->odom_.pose.pose.position.z;
		geometry_msgs::PoseStamped startPs;
		startPs.pose.position.x = r;
		startPs.pose.position.y = 0.0;
		startPs.pose.position.z = z;
		this->updateTarget(startPs);
		
		cout << "[AutoFlight]: Go to target point..." << endl;
		ros::Rate rate (30);
		while (ros::ok() and std::abs(this->odom_.pose.pose.position.x - startPs.pose.position.x) >= 0.1){
			ros::spinOnce();
			rate.sleep();
		}
		cout << "[AutoFlight]: Reach target point." << endl;

		ros::Time startTime = ros::Time::now();
		while (ros::ok()){
			ros::Time currTime = ros::Time::now();
			double t = (currTime - startTime).toSec();
			double rad = v * t / r;
			double x = r * cos(rad);
			double y = r * sin(rad);
			double vx = -v * sin(rad);
			double vy = v * cos(rad);
			double vz = 0.0;
			double aNorm = v*v/r;
			Eigen::Vector3d accVec (x, y, 0);
			accVec = -aNorm * accVec / accVec.norm();
			double ax = accVec(0);
			double ay = accVec(1);
			double az = 0.0;

			// state target message
			tracking_controller::Target target;
			target.position.x = x;
			target.position.y = y;
			target.position.z = z;
			target.velocity.x = vx;
			target.velocity.y = vy;
			target.velocity.z = vz;
			target.acceleration.x = ax;
			target.acceleration.y = ay;
			target.acceleration.z = az;
			this->updateTargetWithState(target);
			ros::spinOnce();
			rate.sleep();
		}
	}

	void flightBase::stop(){
		geometry_msgs::PoseStamped ps;
		ps.pose = this->odom_.pose.pose;
		this->updateTarget(ps);		
	}

	// 
	bool flightBase::land(){
		this->landing_in_progress_ = true;
	
		ROS_INFO("[AutoFlight] Sending Native LAND Signal...");

		// 1. Publish the "Empty" message to trigger landing
		std_msgs::Empty land_msg;
		this->landPub_.publish(land_msg);

		// 2. Wait for the drone to actually descend
		// We assume the UGV deck is around 0.3m ~ 0.4m.
		// We wait until we are below 0.6m.
		
		ros::Rate r(10);
		ros::Time start_time = ros::Time::now();
		double timeout_sec = 20.0;

		while (ros::ok()) {
			ros::spinOnce();
			
			double current_z = this->odom_.pose.pose.position.z;

			// CHECK 1: Are we low enough? (Landed on UGV)
			if (current_z < 0.70) {
				ROS_INFO("[AutoFlight] Touchdown detected (Z=%.2f).", current_z);
				this->landing_in_progress_ = false;
				return true;
			}

			// CHECK 2: Timeout
			if ((ros::Time::now() - start_time).toSec() > timeout_sec) {
				ROS_WARN("[AutoFlight] Landing timed out (Current Z=%.2f). Proceeding anyway.", current_z);
				this->landing_in_progress_ = false;
				return true; 
			}

			r.sleep();
		}
		this->landing_in_progress_ = false;
		return false;
	}

	void flightBase::moveToOrientation(double yaw, double desiredAngularVel){
		double yawTgt = yaw;
		geometry_msgs::Quaternion orientation = AutoFlight::quaternion_from_rpy(0, 0, yaw);
		double yawCurr = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);		
		geometry_msgs::PoseStamped ps;
		ps.pose = this->odom_.pose.pose;
		ps.pose.orientation = orientation;

		double yawDiff = yawTgt - yawCurr; // difference between yaw
		double direction = 0;
		double yawDiffAbs = std::abs(yawDiff);
		if ((yawDiffAbs <= PI_const) and (yawDiff>0)){
			direction = 1.0; // counter clockwise
		} 
		else if ((yawDiffAbs <= PI_const) and (yawDiff<0)){
			direction = -1.0; // clockwise
		}
		else if ((yawDiffAbs > PI_const) and (yawDiff>0)){
			direction = -1.0; // rotate in clockwise direction
			yawDiffAbs = 2 * PI_const - yawDiffAbs;
		}
		else if ((yawDiffAbs > PI_const) and (yawDiff<0)){
			direction = 1.0; // counter clockwise
			yawDiffAbs = 2 * PI_const - yawDiffAbs;
		}

		double endTime = yawDiffAbs/desiredAngularVel;
		tracking_controller::Target target;
		geometry_msgs::PoseStamped psT;
		psT.pose = ps.pose;
		ros::Time startTime = ros::Time::now();
		ros::Time currTime = ros::Time::now();
		ros::Rate r (200);
		while (ros::ok() and not this->isReach(ps)){
			currTime = ros::Time::now();
			double t = (currTime - startTime).toSec();

			if (t >= endTime){ 
				psT = ps;
			}
			else{
				double currYawTgt = yawCurr + (double) direction * t/endTime * yawDiffAbs;
				geometry_msgs::Quaternion quatT = AutoFlight::quaternion_from_rpy(0, 0, currYawTgt);
				psT.pose.orientation = quatT;
				
			}

			// this->updateTarget(psT);
			target.position.x = psT.pose.position.x;
			target.position.y = psT.pose.position.y;
			target.position.z = psT.pose.position.z;
			target.yaw = AutoFlight::rpy_from_quaternion(psT.pose.orientation);
			this->updateTargetWithState(target);
			// cout << "here" << endl;
			ros::spinOnce();
			r.sleep();
		}
	}

	void flightBase::updateTarget(const geometry_msgs::PoseStamped& ps){ // global frame
		std::lock_guard<std::mutex> lk(this->targetMtx_);
		this->poseTgt_ = ps;
		this->poseTgt_.header.frame_id = "map";
		this->poseTgtInit_ = true;
		this->poseControl_ = true;
	}

	void flightBase::updateTargetWithState(const tracking_controller::Target& target){
		std::lock_guard<std::mutex> lk(this->targetMtx_);
		this->stateTgt_ = target;
		this->poseControl_ = false;
	}

	bool flightBase::isReach(const geometry_msgs::PoseStamped& poseTgt, bool useYaw){
		double targetX, targetY, targetZ, targetYaw, currX, currY, currZ, currYaw;
		targetX = poseTgt.pose.position.x;
		targetY = poseTgt.pose.position.y;
		targetZ = poseTgt.pose.position.z;
		targetYaw = AutoFlight::rpy_from_quaternion(poseTgt.pose.orientation);
		currX = this->odom_.pose.pose.position.x;
		currY = this->odom_.pose.pose.position.y;
		currZ = this->odom_.pose.pose.position.z;
		currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		
		bool reachX, reachY, reachZ, reachYaw;
		reachX = std::abs(targetX - currX) < 0.1;
		reachY = std::abs(targetY - currY) < 0.1;
		reachZ = std::abs(targetZ - currZ) < 0.15;
		if (useYaw){
			reachYaw = std::abs(targetYaw - currYaw) < 0.1;
		}
		else{
			reachYaw = true;
		}
		// cout << reachX << reachY << reachZ << reachYaw << endl;
		if (reachX and reachY and reachZ and reachYaw){
			return true;
		}
		else{
			return false;
		}
	}

	bool flightBase::isReach(const geometry_msgs::PoseStamped& poseTgt, double dist, bool useYaw){
		double targetX, targetY, targetZ, targetYaw, currX, currY, currZ, currYaw;
		targetX = poseTgt.pose.position.x;
		targetY = poseTgt.pose.position.y;
		targetZ = poseTgt.pose.position.z;
		targetYaw = AutoFlight::rpy_from_quaternion(poseTgt.pose.orientation);
		currX = this->odom_.pose.pose.position.x;
		currY = this->odom_.pose.pose.position.y;
		currZ = this->odom_.pose.pose.position.z;
		currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		
		bool reachX, reachY, reachZ, reachYaw;
		reachX = std::abs(targetX - currX) < dist;
		reachY = std::abs(targetY - currY) < dist;
		reachZ = std::abs(targetZ - currZ) < dist;
		if (useYaw){
			reachYaw = std::abs(targetYaw - currYaw) < 0.1;
		}
		else{
			reachYaw = true;
		}

		if (reachX and reachY and reachZ and reachYaw){
			return true;
		}
		else{
			return false;
		}
	}
}