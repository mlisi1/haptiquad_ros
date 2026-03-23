#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#define BOOST_MPL_LIMIT_LIST_SIZE 40

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <haptiquad/momentum_observer.hpp>
#include <haptiquad/force_estimator.hpp>

#include <std_msgs/String.h>

#include <haptiquad_msgs/ResidualsStamped.h>
#include <haptiquad_msgs/ResidualErrorStamped.h>
#include <haptiquad_msgs/ObserverGains.h>
#include <haptiquad_msgs/EstimatedForces.h>
#include <haptiquad_msgs/FrictionParameters.h>


#include <Eigen/Dense>
#include <vector>
#include <chrono>
#include <numeric>
#include <random>


class HaptiQuadWrapperBase {

    public:

        HaptiQuadWrapperBase();

    private:

        void descriptionCallback(const std::string xml_description);
        void gainsCallback(const haptiquad_msgs::ObserverGains::ConstPtr& msg);
        void frictionCallback(const haptiquad_msgs::FrictionParameters::ConstPtr& msg);   

    protected:
    
        void publishResiduals();
        void publishForces();
        void publishResidualErrors();
        double computeAverageProcessingTime();
        bool burstLossDropMessage();


    protected:

        bool description_received = false;
        bool first_message, gt_first_message = true;
        std::string xml_description;

        ros::Subscriber description_sub;
        ros::Subscriber gains_sub;
        ros::Subscriber friction_sub;


        ros::Publisher residual_publisher;
        ros::Publisher residual_error_publisher;
        ros::Publisher forces_publisher;

        haptiquad_msgs::ResidualsStamped residual_msg;
        haptiquad_msgs::EstimatedForces forces_msg;
        haptiquad_msgs::ResidualErrorStamped residual_error_msg;

        std::map<std::string, double> msg_position_dict;
        std::map<std::string, double> msg_velocity_dict;
        std::map<std::string, double> msg_torques_dict;

        ros::Time last_stamp, current_stamp;
        double dt;

        Eigen::VectorXd r_int, r_ext;
        Eigen::VectorXd gt_r_int, gt_r_ext;
        Eigen::VectorXd err_int, err_ext;
        std::map<std::string, Eigen::VectorXd> F;
        std::map<std::string, Eigen::VectorXd> GT_F;

        std::vector<std::string> joint_names, feet_frames;

        //Performance monitoring
        std::chrono::high_resolution_clock::time_point start_time, end_time;
        std::chrono::duration<double, std::milli> processing_time;
        std::vector<std::chrono::duration<double, std::milli>> processing_times;
        

        haptiquad::MomentumObserver observer;
        haptiquad::ForceEstimator estimator;

        //ROS node handle
        ros::NodeHandle nh, pnh;

        //PARAMETERS
        std::string base_link_name;
        bool calculate_residual_error;

        //Time rescaling
        int num_contacts = 0;
        float k_int, k_ext = 1.0;
        bool rescale = false;
        double expected_dt = 0.0;
        double threshold = 0.0;  

        //Friction
        bool friction = false;
        double F_s    = 0.0;    // Static (stiction) friction torque [Nm]
        double F_c    = 0.0;    // Coulomb (dynamic) friction torque [Nm]
        double sigma0 = 0.0; // Stiffness coefficient [Nm/rad]
        double sigma1 = 0.0;  // Damping coefficient [Nm·s/rad]
        double sigma2 = 0.0;    // Viscous friction coefficient [Nm·s/rad]
        double alpha  = 0.0;    // Transition parameter [s/rad]


        //Mass/Inertia scaling (for evaluation purposes)
        double mass_scaling = 1.0;
        double inertia_scaling = 1.0;



        //Message Drop Probabilities (for testing robustness)
        double drop_prob = 0.0;
        std::mt19937 rng_;
        std::uniform_real_distribution<double> uniform_dist_;


        // Gilbert–Elliott burst-loss model parameters
        double drop_prob_good_;         // Low drop probability in Good state
        double drop_prob_bad_;          // High drop probability in Bad state
        double p_good_to_bad_;          // Chance to switch from Good to Bad
        double p_bad_to_good_;          // Chance to switch from Bad to Good
        bool in_good_state_;



};
