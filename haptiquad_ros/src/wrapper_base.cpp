#include "wrapper_base.hpp"

HaptiQuadWrapperBase::HaptiQuadWrapperBase() : nh("~") {

    description_sub = nh.subscribe(
        "/floating_base_description",
        10,
        &HaptiQuadWrapperBase::descriptionCallback,
        this
    ); 

    gains_sub = nh.subscribe(
        "gains",
        10,
        &HaptiQuadWrapperBase::gainsCallback,
        this
    );

    friction_sub = nh.subscribe(
        "friction",
        10,
        &HaptiQuadWrapperBase::frictionCallback,
        this
    );

    residual_publisher = nh.advertise<haptiquad_msgs::ResidualsStamped>("residuals", 10);
    residual_error_publisher = nh.advertise<haptiquad_msgs::ResidualErrorStamped>("residual_errors", 10);
    forces_publisher = nh.advertise<haptiquad_msgs::EstimatedForces>("estimated_forces", 10);

    //PARAMETERS
    nh.param<int>("estimator/num_contacts", num_contacts, 0);
    nh.param<float>("observer/k_int", k_int, 1.0);
    nh.param<float>("observer/k_ext", k_ext, 1.0);
    nh.param<bool>("observer/rescale", rescale, false);
    nh.param<double>("observer/expected_dt", expected_dt, 0.0);
    nh.param<double>("observer/threshold", threshold, 0.0); 
    nh.param<std::string>("estimator/base_link_name", base_link_name, "base");
    nh.param<bool>("estimator/calculate_residual_error", calculate_residual_error, false);
    nh.param<double>("evaluation/mass_scaling", mass_scaling, 1.0);
    nh.param<double>("evaluation/inertia_scaling", inertia_scaling, 1.0);
    nh.param<double>("evaluation/drop_prob", drop_prob, 0.0);

    nh.param<double>("evaluation/drop_prob_good", drop_prob_good_, 0.0);
    nh.param<double>("evaluation/drop_prob_bad", drop_prob_bad_, 0.0);
    nh.param<double>("evaluation/p_good_to_bad", p_good_to_bad_, 0.0);
    nh.param<double>("evaluation/p_bad_to_good", p_bad_to_good_, 0.0);
    in_good_state_ = true;

    nh.param<bool>("observer/friction/friction", friction, false);
    nh.param<double>("observer/friction/F_s", F_s, 0.0);    
    nh.param<double>("observer/friction/F_c", F_c, 0.0);    

    observer.setFrictionParameters(friction, F_s, F_c); 
    estimator.setBaseFrame(base_link_name);

    rng_.seed(std::random_device{}());
    uniform_dist_ = std::uniform_real_distribution<double>(0.0, 1.0);

    ROS_DEBUG_STREAM("Initialized base wrapper");

}




void HaptiQuadWrapperBase::descriptionCallback(const std_msgs::String::ConstPtr& msg) {

    if (description_received) {
        ROS_WARN_STREAM("Description already received!");
        return;
    }

    pinocchio::Model model;
    pinocchio::urdf::buildModelFromXML(msg->data, model);


    for (auto & inertia : model.inertias)
    {
        inertia.mass() *= mass_scaling;
        inertia.inertia() *= inertia_scaling;  
    }

    ROS_DEBUG_STREAM("Received pinocchio model with " << model.nv << " DOFs");
    ROS_DEBUG_STREAM("======================================================");
    ROS_DEBUG_STREAM("Model inertias:");
    for (size_t i=0; i<model.inertias.size(); i++) {
        ROS_DEBUG_STREAM("\n" << model.inertias[i] << "\n");
        ROS_DEBUG_STREAM("----------------------------------");
    }
    ROS_DEBUG_STREAM("=======================================================");

    observer.initModel(model);
    observer.setInternalGain(k_int);
    observer.setExternalGain(k_ext);

    if (rescale) {
        observer.enableTimeScaling(expected_dt, threshold);
    }

    estimator.initModel(model);
    estimator.setNumContacts(num_contacts);

    residual_msg.r_int.resize(model.nv-6);
    residual_msg.r_ext.resize(6);
    residual_msg.names.resize(model.nv-6);


    residual_error_msg.err_int.resize(model.nv-6);
    residual_error_msg.err_ext.resize(6);
    residual_error_msg.names.resize(model.nv-6);


    forces_msg.forces.resize(num_contacts + 1);
    forces_msg.names.resize(num_contacts + 1);

    

    joint_names.resize(model.nv-6);
    ROS_DEBUG_STREAM("Joint names:");
    
    for (size_t i=0; i<joint_names.size(); i++) {
        joint_names[i] = model.names[i+2];
        ROS_DEBUG_STREAM(joint_names[i]);
    }
    ROS_DEBUG_STREAM("=======================================================");

    residual_msg.names = joint_names;
    residual_error_msg.names = joint_names;

    estimator.findFeetFrames(joint_names);

    feet_frames = estimator.getFeetFrames();
    ROS_DEBUG_STREAM("Feet frames:");
    for (int i=0; i<num_contacts; i++) {
        forces_msg.names[i] = feet_frames[i];
        ROS_DEBUG_STREAM(feet_frames[i]);
    }
    forces_msg.names[num_contacts] = "base_wrench";
    ROS_DEBUG_STREAM("========================================================");

    description_received = true;
    ROS_INFO_STREAM("Description received!");

}



void HaptiQuadWrapperBase::gainsCallback(const haptiquad_msgs::ObserverGains::ConstPtr& msg) {

    ROS_DEBUG_STREAM("Received gains");
    observer.setInternalGain(msg->k_int);
    observer.setExternalGain(msg->k_ext);

}


void HaptiQuadWrapperBase::frictionCallback(const haptiquad_msgs::FrictionParameters::ConstPtr& msg) {

    ROS_DEBUG_STREAM("Received friction parameters");
    observer.setFrictionParameters(msg->use_friction, msg->f_s, msg->f_c);
}




void HaptiQuadWrapperBase::publishResiduals() {

    for (int i=0; i<r_int.size(); i++) {
        residual_msg.r_int[i] = r_int(i);
        
    }

    for (int i=0; i<r_ext.size(); i++) {
        residual_msg.r_ext[i] = r_ext(i);
    }

    residual_msg.header.stamp = current_stamp;
    residual_publisher.publish(residual_msg);

}




void HaptiQuadWrapperBase::publishResidualErrors() {

    for (int i=0; i<err_int.size(); i++) {
        residual_error_msg.err_int[i] = err_int(i);
        
    }

    for (int i=0; i<err_ext.size(); i++) {
        residual_error_msg.err_ext[i] = err_ext(i);
    }

    residual_error_msg.header.stamp = current_stamp;
    residual_error_publisher.publish(residual_error_msg);

}





void HaptiQuadWrapperBase::publishForces() {

    forces_msg.header.stamp = current_stamp;
    forces_msg.header.frame_id = "world";

    for (int i=0; i<num_contacts; i++) {

        forces_msg.forces[i].force.x =  F[feet_frames[i]][0];
        forces_msg.forces[i].force.y =  F[feet_frames[i]][1];
        forces_msg.forces[i].force.z =  F[feet_frames[i]][2];
        forces_msg.forces[i].torque.x = F[feet_frames[i]][3];
        forces_msg.forces[i].torque.y = F[feet_frames[i]][4];
        forces_msg.forces[i].torque.z = F[feet_frames[i]][5];

    }

    forces_msg.forces[num_contacts].force.x = F["base_wrench"][0];
    forces_msg.forces[num_contacts].force.y = F["base_wrench"][1];
    forces_msg.forces[num_contacts].force.z = F["base_wrench"][2];
    forces_msg.forces[num_contacts].torque.x = F["base_wrench"][3];
    forces_msg.forces[num_contacts].torque.y = F["base_wrench"][4];
    forces_msg.forces[num_contacts].torque.z = F["base_wrench"][5];


    forces_publisher.publish(forces_msg);


}





double HaptiQuadWrapperBase::computeAverageProcessingTime() {

    if (processing_times.empty()) {
        return 0.0;
    }
    double sum = 0.0;
    for (const auto& d : processing_times) {
        sum += d.count();
    }
    return sum / processing_times.size();
}







bool HaptiQuadWrapperBase::burstLossDropMessage()
{
    double r = uniform_dist_(rng_);
    double current_drop_prob = in_good_state_ ? drop_prob_good_ : drop_prob_bad_;

    if (in_good_state_)
    {
        if (r < p_good_to_bad_) in_good_state_ = false;
    }
    else
    {
        if (r < p_bad_to_good_) in_good_state_ = true;
    }

    // Decide whether to drop the message
    double drop_prob = in_good_state_ ? drop_prob_good_ : drop_prob_bad_;
    if (uniform_dist_(rng_) < drop_prob)
    {
        ROS_WARN("Message dropped (burst model)!");
        return true;
    }

    return false;
}