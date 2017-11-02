#ifndef QUAD_SIMULATOR_PARSER_H
#define QUAD_SIMULATOR_PARSER_H
/* This header is subclassed from parser.h and will include data from Quadcopter simulator for testing GUI. It will also provide interfaces for commanding the Simulated Quadcopter.
 * This class uses internal locking to ensure the quaddata is available to Qt thread without bumping into ros serial. This is enough since the order of working is not important to us. Read internal vs external locking: http://www.boost.org/doc/libs/1_55_0/doc/html/thread/synchronization.html
 */
#include <parsernode/parser.h> //main parser base class
#include <quad_simulator_parser/quad_simulator.h> //base class for this

#include <tf/transform_broadcaster.h>

using namespace std;
using namespace gcop;
using namespace Eigen;
namespace quad_simulator_parser{
class QuadSimParser: public quad_simulator::QuadSimulator
{
    
private:
    // NodeHandle
    ros::NodeHandle nh_;
    //Subscribers:
    ros::Subscriber joy_sub_;
    ros::Publisher global_ref_pub;
    ros::Publisher gps_pub;
    tf::TransformBroadcaster tf_broadcaster;

    //Timer to keep decreasing thrust gain kt
    ros::Timer kt_decrease_timer_;
    //Gps Timer:
    ros::Timer gps_pub_timer_;
    //TF Timer:
    ros::Timer tf_timer_;
protected:
    void ktTimerCallback(const ros::TimerEvent& );
    void gpsTimerCallback(const ros::TimerEvent& );
    void tfTimerCallback(const ros::TimerEvent&);

public:
    QuadSimParser();
    ~QuadSimParser()
    {
        disarm();//release sdk control
    }
    //Extend functions from Parser:
    virtual void initialize();
};
}
#endif // DJI_PARSHER_H
