#include "anymal_wrapper.hpp"



ANYmalWrapper::ANYmalWrapper() : HaptiQuadWrapperBase() {

    bag_sub = nh.subscribe(
        "/state_estimator/anymal_state",
        10,
        &ANYmalWrapper::bagCallback,
        this
    );

}



void ANYmalWrapper::bagCallback(const anymal_msgs::AnymalState::ConstPtr& msg) {

    if (!description_received) {
        ROS_ERROR_STREAM("Robot description was not yet received");
        return;
    }

    if (first_message) {

        last_stamp = msg->header.stamp; 
        first_message = false;
        return;

    }


    for (size_t i=0; i<msg->contacts.size(); i++) {

        Eigen::VectorXd tmp = Eigen::VectorXd::Zero(6);       

        tmp << msg->contacts[i].wrench.force.x,
                msg->contacts[i].wrench.force.y,
                msg->contacts[i].wrench.force.z,
                msg->contacts[i].wrench.torque.x,
                msg->contacts[i].wrench.torque.y,
                msg->contacts[i].wrench.torque.z;

        GT_F[msg->contacts[i].name] = tmp;

    }

    GT_F["base_wrench"] = Eigen::VectorXd::Zero(6);



    for (size_t i=0; i<msg->joints.position.size(); i++) {

        msg_position_dict[msg->joints.name[i]] =    msg->joints.position[i];
        msg_velocity_dict[msg->joints.name[i]] =    msg->joints.velocity[i];
        msg_torques_dict[msg->joints.name[i]] =     msg->joints.effort[i];
        
    }

    observer.updateJointStates(msg_position_dict, msg_velocity_dict, msg_torques_dict);

    Eigen::Quaterniond orientation = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                                    msg->pose.pose.orientation.x,
                                                    msg->pose.pose.orientation.y,
                                                    msg->pose.pose.orientation.z);

    Eigen::VectorXd v0 = Eigen::VectorXd::Zero(6);
    v0 << msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z,
            msg->twist.twist.angular.x,
            msg->twist.twist.angular.y,
            msg->twist.twist.angular.z; 


    observer.updateBaseState(v0, orientation);

    current_stamp = msg->header.stamp;
    dt = (current_stamp - last_stamp).toSec();

    std::tie(r_int, r_ext) = observer.getResiduals(dt);

    publishResiduals();

    estimator.updateJacobians(msg_position_dict, observer.getF(), observer.getIC());

    F = estimator.calculateForces(r_int, r_ext, orientation);

    std::tie(gt_r_int, gt_r_ext) = estimator.calculateResidualsFromForces(GT_F);
    err_int = gt_r_int - r_int;
    err_ext = gt_r_ext - r_ext;

    publishForces();
    publishResidualErrors();

    last_stamp = msg->header.stamp;


}







int main(int argc, char **argv) {
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    ros::init(argc, argv, "haptiquad_anymal");
    ANYmalWrapper wrapper;
    ros::spin();
    return 0;
}