#include "wrapper_base.hpp"

#include <anymal_msgs/AnymalState.h>


class ANYmalWrapper : public HaptiQuadWrapperBase {

    public:

        ANYmalWrapper();

        void bagCallback(const anymal_msgs::AnymalState::ConstPtr& msg);


    private:

        ros::Subscriber bag_sub;

};