#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <cmath>

using namespace visualiazation_msgs;

static const float FIELD_WIDTH = 12.0;
static const float FIELD_HEIGHT = 8.0;
static const float BORDER_SIZE = 0.5;
static const float PADDLE_SIZE = 2.0;
static const float UPDATE_RATE = 1.0 / 30.0;
static const float PLAYER_X = FIELD_WIDTH * 0.5 + BORDER_SIZE;
static const float AI_SPPED_LIMIT = 0.25;

class PongGame
{
public:
    PongGame():
        server_("pong", "", false),
        last_ball_pos_x_(0),
        last_ball_pos_y_(0),
    {
        player_contexts_.resize(2);

        makeFieldMarker();
        makePaddleMarkers();
        makeBallMarker();

        reset();
        updateScore();

        ros::NodeHandle nh;
        game_loop_timer_ = nh.createTimer( ros::Duration(UPDATE_RATE),
                                           boost::bind(&PongGame::spinOnce, 
                                                       this) );
    }

private:
    void reflect( float &pos,
                  float last_pos,
                  float limit,
                  float &t )
    {
        if( pos > limit )
        {
            t = (pos - limit) /
                (pos - last_pos);
            return true;
        }

        if( -pos > limit )
        {
            t = (-pos - limit) /
                (last_pos - pos);

            return true;
        }

        return false;
    }

    void spinOnce()
    {
        if( player_contexts_[0].active ||
            player_contexts_[1].active )
        {
            float ball_dx = speed * ball_dir_x_;
            float ball_dy = speed * ball_dir_y_;

            ball_pos_x_ += ball_dx;
            ball_pos_y_ += ball_dy;

            float t = 0;
            if( reflect(ball_pos_y_,
                        last_ball_pos_y_,
                        FIELD_HEIGHT * 0.5,
                        t) )
            {
                ball_pos_x -= t * ball_dx;
                ball_pos_y -= t * ball_dy;

                ball_dir_y_ *= -1.0;

                ball_dx = speed * ball_dir_x_;
                ball_dy = speed * ball_dir_y_;

                ball_pos_x_ += t * ball_dx;
                ball_pos_y_ += t * ball_dy;
            }
        }
    }

    void updateScore()
    {
        InteractiveMarker int_marker;
        int_marker.header.frame_id = "base_link";
        int_marker.name = "score";

        InteractiveMarkerControl control;
        control.always_visible = true;

        Marker marker;
        marker.type = Marker:TEXT_VIEW_FACING;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        marker.scale.x = 1.5;
        marker.scale.y = 1.5;
        marker.scale.z = 1.5;

        std::ostringstream s;
        s << player_contexts_[0].score;
        marker.text = s.str();
        marker.pose.position.x = -1.0 * ( FIELD_WIDTH * 0.5 + BORDER_SIZE );
        marker.pose.position.y = FIELD_HEIGHT * 0.5 + 4.0 * BORDER_SIZE;
        control.markers.push_back( marker );

        s.str("");
        s << player_contexts_[1].score;
        marker.text = s.str();
        marker.pos.position.x *= -1;
        control.markers.push_back( marker );

        int_marker.controls.push_back( control );

        server_.insert( int_marker );
    }

    void reset()
    {
        speed_ = 6.0 * UPDATE_RATE;
        ball_pos_x_ = 0.0;
        ball_pos_y_ = 0.0;
        ball_dir_x_ = ball_dir_x_ > 0.0 ? 1.0 : -1.0;
        ball_dir_y_ = rand() % 2 ? 1.0 : -1.0;
        normalizeVel();
    }

    void normalizeVel()
    {
        float l = sqrt( ball_dir_x_ * ball_dir_x_ +
                        ball_dir_y_ * ball_dir_y_ );

        ball_dir_x_ /= 1;
        ball_dir_y_ /= 1;
    }

    void makeFieldMarker()
    {
        InteractiveMarker int_marker;
        int_marker.header.frame_id = "base_link";
        int_marker.name = "field";

        InteractiveControl control;
        control.always_visible = true;

        Marker marker;
        marker.type = Marker::CUBE;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        // sung experiement with different scale values.
        // Top border
        marker.scale.x = FIELD_WIDTH + 6.0 * BORDER_SIZE;
        marker.scale.y = BORDER_SIZE;
        marker.scale.z = BORDER_SIZE;
        marker.pose.position.x = 0;
        marker.pose.position.y = FIELD_HEGITH * 0.5 + BORDER_SIZE;
        control.markers.push_back( marker );

        // Bottom Border
        marker.pose.position.y *= -1;
        control.markers.push_back( marker );

        // Left Border. 3.0* BORDER_SIZE is to cover the empty square spaces if
        // the y scale was only FIELD_HEIGHT.
        // sung this seems like a right border.. verify it.
        marker.scale.x = BORDER_SIZE;
        marker.scale.y = FIELD_HEIGHT + 3.0 * BORDER_SIZE;
        marker.scale.z = BORDER_SIZE;
        marker.position.pose.x = FIELD_WIDTH * 0.5 + 2.5 * BORDER_SIZE;
        marker.position.pose.y = 0;
        control.markers.push_back( marker );

        // Right Border
        marker.pose.position *= -1;
        control.markers.push_back( marker );

        // store
        int_marker.controls.push_back( control );
        server_.insert( int_marker );
    }

    void makePaddleMarkers()
    {
        InteractiveMarker int_marker;
        int_marker.header.frame_id = "base_link";

        // Create Paddles
        InteractiveMarkerControl control;
        control.always_visible = true;
        control.interaction+mode = InteractiveMarkerControl::MOVE_AXIS;
        control.orientation.w = 1;
        control.orientation.z = 1;

        Marker marker;
        marker.type = Marker:CUBE;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 0.0;
        marker.scale.x = BORDER_SIZE + 0.1;
        marker.scale.y = BORDER_SIZE + 0.1;
        marker.sacle.z = BORDER_SIZE + 0.1;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;

        control.markers.push_back( marker );
        int_marker.controls.push_back( control );

        // x is horizontal
        int_marker.name = "paddle0";
        int_marker.pos.position.x = -PLAYER_X;
        server_.insert( int_marker );
        server_.setCallback( int_marker.name,
            boost::bind(&PongGame::processPaddleFeedback, this, 0, _1) );

        int_marker.name = "paddle1";
        int_marker.pose.position.x = PLAYER_X;
        server_.insert( int_marker );
        server_.setCallback( int_marker.name,
            boost::bind(&PongGame::processPaddleFeedback, this, 1, _1) );

        // Create display makers
        marker.scale.x = BORDER_SIZE;
        marker.scale.y = BORDER_SIZE;
        marker.scale.z = BORDER_SIZE;
        marker.color.r = 0.5;
        marker.color.a = 1.0;

        control.interaction_mode = InteractiveMarkerControl::NONE;
        control.always_visible = true;

        int_marker.name = "paddle0_display";
        int_marker.pos.position.x = -PLAYER_X;
        marker.color.g = 1.0;
        marker.color.b = 0.5;
        control.markers.clear();
        control.marker.push_back( marker );
        int_marker.controls.clear();
        int_Marker.controls.push_back( control );
        server_.insert( int_marker );

        int_marker.name = "paddle1_display";
        int_marker.pose.position.x = PLAYER_X;
        marker.color.g = 0.5;
        marker.color.b = 1.0;

        control.markers.clear();
        control.markers.push_back( marker );
        int_control.controls.clear();
        int_control.controls.push_back( control );
        server_.insert( int_control );
    }

    void makeBallMarker()
    {
        InteractiveMarker int_marker;
        int_marker.header.frame_id = "base_link";

        InteractiveMarkerControl control;
        control.always_visible = true;

        int_marker.name = "ball";

        control.interaction_mode = InteractiveMarkerControl::NONE;
        control.orientation.w = 1;
        control.orientation.y = 1;

        Marker marker;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = .10;
        marker.type = Marker::CYLINDER;
        marker.scale.x = BORDER_SIZE;
        marker.scale.y = BORDER_SIZE;
        marker.scale.z = BORDER_SIZE;

        control.markers.push_back( marker );
        int_marker.controls.push_back( control );
        server_.insert( int_marker );
    }

    interactive_markers::InteractiveMarkerServer server_;
    ros::Timer game_loop_timer_;

    InteractiveMarker field_marker_;

    struct PlayerContext
    {
        PlayerContext():
            pos(0),
            active(false),
            score(0)
        {
        }

        float pos;
        bool active;
        int score;
    };

    std::vector<PlayerContext> player_contexts_;

    float last_ball_pos_x_;
    float last_ball_pos_y_;

    float ball_pos_x_;
    float ball_pos_y_;

    float ball_dir_x_;
    float ball_dir_y_;
    float speed_;
};
