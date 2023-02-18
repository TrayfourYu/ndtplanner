
bool AstarPlanar::search()
{
  // -- Start Astar search ----------
  // If the openlist is empty, search failed
     int search_count = 0;
  while (!openlist_.empty()) {

    // Terminate the search if the count reaches a certain value
    search_count++;
    if (search_count > search_count_num) {
      ROS_WARN("Exceed time limit");
      search_count = 0;
      return false;
    }

    // Pop minimum cost node from openlist
    SimpleNode sn;
    sn = openlist_.top();
    openlist_.pop();
    nodes_[sn.index_y][sn.index_x][sn.index_theta].status = STATUS::CLOSED;

    // Expand include from this node
    AstarNode *current_node = &nodes_[sn.index_y][sn.index_x][sn.index_theta];

    // ROS_WARN_STREAM("X IS"<<sn.index_x*map_info_.resolution<<",,,,Y IS"<<sn.index_y*map_info_.resolution<<",,,,THETA IS"<<sn.index_theta*map_info_.resolution);

// if(calcstep(sn)!=max_step || calcstep(sn)!=min_step)
// {
    ROS_WARN_STREAM("!!!!!!!!!!!!search_count IS "<<search_count<<"!!!!!");
// }
    // for each update
    for (const auto &state : state_update_table_[sn.index_theta]) {
      // Next state
      double next_x     = current_node->x + state.shift_x;
      double next_y     = current_node->y + state.shift_y;
      double next_theta = astar::modifyTheta(current_node->theta + state.rotation);
      double move_cost  = state.step;

// #if  DEBUG
      // Display search process
      geometry_msgs::Pose p;
      p.position.x = next_x;
      p.position.y = next_y;
      tf::quaternionTFToMsg(tf::createQuaternionFromYaw(next_theta), p.orientation);
      //p = astar::transformPose(p, map2ogm_);
      debug_pose_array_.poses.push_back(p);
// #endif

      // Increase curve cost
      if (state.curve)
        move_cost *= curve_weight_;

      if (state.rip)
        move_cost *= rip_weight_;

      // Increase reverse cost
      if (current_node->back != state.back)
        move_cost *= reverse_weight_;

      // Calculate index of the next state
      SimpleNode next;
      next.index_x     = next_x / map_info_.resolution;
      next.index_y     = next_y / map_info_.resolution;
      next.index_theta = sn.index_theta + state.index_theta;
      // Avoid invalid index
      next.index_theta = (next.index_theta + angle_size) % angle_size;

      // Check if the index is valid
      if (isOutOfRange(next.index_x, next.index_y) || detectCollision(next))
        continue;

      AstarNode *next_node = &nodes_[next.index_y][next.index_x][next.index_theta];
      double next_hc       =  nodes_[next.index_y][next.index_x][0].hc;
      double next_dis       =  nodes_[next.index_y][next.index_x][0].dis;

      double dis_cost = next_dis;
      dis_cost *= dis_weight_;

      // Calculate euclid distance heuristic cost
      if (!use_wavefront_heuristic_)
        next_hc = astar::calcDistance(next_x, next_y, goal_pose_local_.pose.position.x, goal_pose_local_.pose.position.y);

      // GOAL CHECK
      if (isGoal(next_x, next_y, goal_radius_)) {
        search_count = 0;
        next_node->status = STATUS::OPEN;
        next_node->x      = next_x;
        next_node->y      = next_y;
        next_node->theta  = next_theta;
        next_node->gc     = current_node->gc + move_cost + dis_cost;
        next_node->hc     = next_hc;
        next_node->dis     = next_dis;
        next_node->back   = state.back;
        next_node->parent = current_node;

        setPath(next);
        return true;
      }

      // NONE
      if (next_node->status == STATUS::NONE) {
        next_node->status = STATUS::OPEN;
        next_node->x      = next_x;
        next_node->y      = next_y;
        next_node->theta  = next_theta;
        next_node->gc     = current_node->gc + move_cost + dis_cost;
        next_node->hc     = next_hc;
        next_node->dis     = next_dis;
        next_node->back   = state.back;
        next_node->parent = current_node;

        next.cost = next_node->gc + h_weight_ * next_node->hc;
        openlist_.push(next);
        continue;
      }

      // OPEN or CLOSED
      if (next_node->status == STATUS::OPEN || next_node->status == STATUS::CLOSED) {
        if (current_node->gc + move_cost + dis_cost + next_hc < next_node->gc + next_hc) {   
          next_node->status = STATUS::OPEN;
          next_node->x      = next_x;
          next_node->y      = next_y;
          next_node->theta  = next_theta;
          next_node->gc     = current_node->gc + move_cost + dis_cost;
          next_node->hc     = next_hc;
          next_node->dis     = next_dis;
          next_node->back   = state.back;
          next_node->parent = current_node;

          next.cost = next_node->gc + h_weight_ * next_node->hc;
          openlist_.push(next);
          continue;
        }
      }

      if (search_count == 0)
      {
        ROS_WARN("Openlist is Empty!");
        break;
      }

    } // state update

  }

  // Failed to find path
  ROS_WARN("global planner: search all the nodes, and Openlist is Empty!");
  return false;
}

void createStateUpdateTable()
{
    state_update_table_.resize(angle_size);
    for (int k = 0; k < angle_size; k++)
    { 
        state_update_table_[k].resize(primitive_num+rip_num);
    }

    double descretized_angle = 2.0 * M_PI / angle_size;

    for (int k = 0; k < angle_size; k++)
    {
        double robot_angle = descretized_angle * k;
        double step = min_step;
        // ROS_WARN_STREAM("!!!!!!!!!!!!!!!!!!!!!!!step is"<<step);
        // Calculate x and y shift to next state
        NodeUpdate nu;
        // forward
        nu.shift_x     = step * std::cos(robot_angle);
        nu.shift_y     = step * std::sin(robot_angle);
        nu.rotation    = 0;
        nu.index_theta = 0;
        nu.step        = step;
        nu.curve       = false;
        nu.back        = false;
        state_update_table_[k][0] = nu;

        for (int l = 1; l < (primitive_num-1)/2+1; l++) {
            double turning_radius =step / (descretized_angle*l);

            // Calculate right and left circle
            // Robot moves along these circles
            double right_circle_center_x = turning_radius * std::sin(robot_angle);
            double right_circle_center_y = turning_radius * std::cos(robot_angle) * -1.0;
            double left_circle_center_x  = right_circle_center_x * -1.0;
            double left_circle_center_y  = right_circle_center_y * -1.0;

            // forward right
            nu.shift_x     = right_circle_center_x + turning_radius * std::cos(M_PI_2 + robot_angle - l*descretized_angle);
            nu.shift_y     = right_circle_center_y + turning_radius * std::sin(M_PI_2 + robot_angle - l*descretized_angle);
            nu.rotation    = descretized_angle * - l;
            nu.index_theta = -l;
            nu.step        = step;
            nu.curve       = true;
            nu.back        = false;
            state_update_table_[k][2*l-1] = nu;

            // forward left
            nu.shift_x     = left_circle_center_x + turning_radius * std::cos(-1.0 * M_PI_2 + robot_angle + l*descretized_angle);
            nu.shift_y     = left_circle_center_y + turning_radius * std::sin(-1.0 * M_PI_2 + robot_angle + l*descretized_angle);
            nu.rotation    = descretized_angle * l;
            nu.index_theta = l;
            nu.step        = step;
            nu.curve       = true;
            nu.back        = false;
            state_update_table_[k][2*l] = nu;

        }

        for(int i=1; i<rip_num/2+1;i++)
        {
            nu.shift_x     = 0.0;
            nu.shift_y     = 0.0;
            nu.rotation    = descretized_angle*2.0*i;
            nu.index_theta = 2*i;
            nu.step        = step;
            nu.curve       = false;
            nu.rip = true;
            nu.back        = false;
            state_update_table_[k][primitive_num-1+2*i-1] = nu;

            nu.shift_x     = 0.0;
            nu.shift_y     = 0.0;
            nu.rotation    = -descretized_angle*2.0*i;
            nu.index_theta = -2*i;
            nu.step        = step;
            nu.curve       = false;
            nu.rip = true;
            nu.back        = false;
            state_update_table_[k][primitive_num-1+2*i] = nu;
        }

    }
}