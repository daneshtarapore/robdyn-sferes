#ifndef SIMU_HPP
#define SIMU_HPP

//#define DETAIL_LOGS


#include <boost/tuple/tuple.hpp>
#include <boost/shared_ptr.hpp>
#include <ode/box.hh>

#include "hexapod.hh"

#include <fstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/filesystem.hpp>

#include <modules/nn2/neuron.hpp>
#include <modules/nn2/pf.hpp>
#include <modules/nn2/gen_dnn_ff.hpp>
#include <modules/nn2/trait.hpp>
#include "af_cppn.hpp"

#include <sferes/dbg/dbg.hpp>
#include <sferes/stc.hpp>
#include <sferes/run.hpp>
#include <sferes/misc.hpp>

#ifdef OFBV90
 #define ORIENTFB_ANGLE_SENSITIVITY 90.0
#endif
#ifdef OFBV70
 #define ORIENTFB_ANGLE_SENSITIVITY 70.0
#endif
#ifdef OFBV50
 #define ORIENTFB_ANGLE_SENSITIVITY 50.0
#endif
#ifdef OFBV36
 #define ORIENTFB_ANGLE_SENSITIVITY 36.0
#endif
#ifdef OFBV24
 #define ORIENTFB_ANGLE_SENSITIVITY 24.0
#endif
#ifdef OFBV18
 #define ORIENTFB_ANGLE_SENSITIVITY 18.0
#endif
#ifdef OFBV10
 #define ORIENTFB_ANGLE_SENSITIVITY 10.0
#endif
#ifndef ORIENTFB_ANGLE_SENSITIVITY
#define ORIENTFB_ANGLE_SENSITIVITY 180.0
#endif
template<typename NN> class Simu
{
public:
    static constexpr float step = 0.015; float end_time;
    typedef boost::shared_ptr<robot::Hexapod> robot_t;


    Simu(NN& ctrl, const robot_t& robot, std::vector<int> brokenLegs, float duration, float floorangle) :
        _brokenLegs(brokenLegs),
        _covered_distance(-10002.0f),
        _slam_duration(0.0f),
        _energy(0.0f),
        _env(new ode::Environment_hexa(floorangle)),

    #ifdef GRAPHIC
        _visitor(renderer::OsgVisitor::FOLLOW),
    #endif
        _ctrlrob(ctrl)
    {
        for(size_t leg = 0; leg < 6; ++leg)
        {
            std::vector<float> param(2, 0.0);
            _offset_time.push_back(param);
            _servo_time.push_back(param);

            _prev_contact.push_back(false);
            _contact.push_back(true);

            std::vector<float> param1(3, 0.0);
            _angles.push_back(param1);
            _prev_angles.push_back(param1);
            _angles_actual.push_back(param1);
            _prev_angles_actual.push_back(param1);
        }

        _robot = robot->clone(*_env);

#ifdef GRAPHIC
        _robot->accept(_visitor);
        // uncomment below to record frames
        /*std::string prefix = "frame";
        _visitor.enable_dump(prefix);*/
#endif

        _env->set_gravity(0, 0, -9.81);

        try
        {
            duration = 5.0;
            _make_robot_init(duration);
        }
        catch (int e)
        {
            std::cout << "An exception occurred. Exception Nr. " << e << std::endl;
            _covered_distance=-10002.0f; //!Dirty way to prevent selection on this robot
        }

#ifdef GRAPHIC
        //std::string res_name = misc::hostname() + "_" + misc::date() + "_" + misc::getpid();
        //write_contact(res_name + "contact_simu.txt");
        //write_traj(res_name + "traj_simu.txt");
        write_contact("contact_simu.txt");
        write_traj("traj_simu.txt");
#endif
    }

    ~Simu()
    {
        // we have to clean in the good order
        _robot.reset();
        _env.reset();

    }

    void next_step()
    {
        _robot->next_step(step);
        _env->next_step(step);
#ifdef GRAPHIC
        _visitor.update();
        usleep(1e4);
#endif
    }

    robot_t robot()
    {
        return _robot;
    }

    float covered_distance()
    {
        return _covered_distance;
    }

    float slam_duration()
    {
        return _slam_duration;
    }

    std::vector <float> get_rel_grf_bd(std::vector <float> &grf_eachleg, float total_force)
    {
        // total_force is the net GRF across all six legs
        std::vector<float> results;
        results.push_back(std::max(grf_eachleg[0] / total_force, 0.0f)); // the z component of the GRF sometimes takes very small negative values (< 0.01)
        results.push_back(std::max(grf_eachleg[1] / total_force, 0.0f));
        results.push_back(std::max(grf_eachleg[2] / total_force, 0.0f));
        results.push_back(std::max(grf_eachleg[3] / total_force, 0.0f));
        results.push_back(std::max(grf_eachleg[4] / total_force, 0.0f));
        results.push_back(std::max(grf_eachleg[5] / total_force, 0.0f));
        return results;
    }


    std::vector<float> get_orientation_bd(float perc_threshold)
    {
        std::vector<float> results;
        results.push_back(countofvector(pitch_vec , (perc_threshold / 100.0) * M_PI, true));
        results.push_back(countofvector(pitch_vec , (perc_threshold / 100.0) * M_PI, false));
        results.push_back(countofvector(roll_vec , (perc_threshold / 100.0) * M_PI, true));
        results.push_back(countofvector(roll_vec , (perc_threshold / 100.0) * M_PI, false));
        results.push_back(countofvector(yaw_vec , (perc_threshold / 100.0) * M_PI, true));
        results.push_back(countofvector(yaw_vec , (perc_threshold / 100.0) * M_PI, false));
        return results;
    }

    float energy()
    {
        return _energy;
    }

    float direction()  {return _direction;}
    float arrival_angle() {return _arrival_angle;}
    std::vector<float> final_pos() {return _final_pos;}

    void write_contact(std::string const name)
    {
        std::ofstream workingFile(name.c_str());

        if (workingFile)
            for (int i =0;i<_behavior_contact_0.size();i++)
                workingFile<<_behavior_contact_0[i]<<" "<<_behavior_contact_1[i]<<" "<<_behavior_contact_2[i]<<" "<<_behavior_contact_3[i]<<" "<<_behavior_contact_4[i]<<" "<<_behavior_contact_5[i]<<std::endl;
        else
            std::cout << "ERROR: Impossible to open the file." << std::endl;
    }

    void write_traj(std::string const name)
    {
        std::ofstream workingFile(name.c_str());
        if (workingFile)
            for (int i =0;i<_behavior_traj.size();i++)
                workingFile<<_behavior_traj[i][0]<<" "<<_behavior_traj[i][1]<<" "<<_behavior_traj[i][2]<<std::endl;
        else
            std::cout << "ERROR: Impossible to open the file." << std::endl;
    }

    const std::vector<Eigen::Vector3d>& get_traj()
    {
        return _behavior_traj;
    }

    const std::vector<float>& get_contact(int i)
    {
        switch (i)
        {
        case 0:
            return _behavior_contact_0;
            break;
        case 1:
            return _behavior_contact_1;
            break;
        case 2:
            return _behavior_contact_2;
            break;
        case 3:
            return _behavior_contact_3;
            break;
        case 4:
            return _behavior_contact_4;
            break;
        case 5:
            return _behavior_contact_5;
            break;
        }
        assert(false);
        return _behavior_contact_0;
    }

    bool isBroken(int leg)
    {
        for (int j=0;j<_brokenLegs.size();j++)
            if (leg==_brokenLegs[j])
                return true;
        return false;
    }

    //count the number of elements above a threshold
    template<typename V1>
    float countofvector(const V1& v1, double threshold, bool dir)
    {
        typename V1::const_iterator it1 = v1.begin();
        float res = 0.0f;
        while (it1 != v1.end())
        {
            if(dir && (float)*it1 > threshold)
                res+=1.0f;

            if(!dir && (float)*it1 < -threshold)
                res+=1.0f;

            ++it1;
        }
        return res / (float) v1.size() ;
    }

    float timer(size_t leg, size_t servo, bool prev_contact, bool contact);
    void moveRobot(robot_t rob, float t);
    std::vector<std::vector<float> > angles_forfft;
    float servo_frequencies_max;

    // Additional behavior descriptors
    std::vector<double> pitch_vec, roll_vec, yaw_vec; // orientation


protected:

    bool stabilize_robot()
    {
        robot_t rob = this->robot();

        // low gravity to slow things down (eq. smaller timestep?)
        _env->set_gravity(0, 0, -9.81);
        bool stabilized = false;
        int stab = 0;

        for (size_t s = 0; s < 1000 && !stabilized; ++s)
        {
            Eigen::Vector3d prev_pos = rob->pos();

            next_step();
            if ((rob->pos() - prev_pos).norm() < 1e-4)
                stab++;
            else
                stab = 0;
            if (stab > 30)
                stabilized = true;
        }
        _env->set_gravity(0, 0, -9.81);
        return(stabilized);
    }

    float servo_frequency(float t, float duration)
    {
        float frequency;
        if (angles_forfft.size() >= 1) // just in case a run is stopped short
        {
            std::vector<std::vector<float> > phasechanges_forfft;
            std::vector<float> tmp_phasechanges(18,0.0);
            for (int o =1; o < angles_forfft.size(); o++)
            {
                for (int p =0; p < angles_forfft[o].size(); p++)
                {

                    if(angles_forfft[o][p] == angles_forfft[o-1][p]) // && _prev_angles_actual[leg][0] == 0
                        tmp_phasechanges[p] = 0.0;
                    else if(angles_forfft[o][p] < angles_forfft[o-1][p])
                        tmp_phasechanges[p] = -1.0;
                    else if(angles_forfft[o][p] > angles_forfft[o-1][p])
                        tmp_phasechanges[p] = 1.0;
                }
                phasechanges_forfft.push_back(tmp_phasechanges);
            }

            std::vector<float> servo_frequencies(18,0.0);
            for (int o =1; o < phasechanges_forfft.size(); o++)
                for (int p =0; p < phasechanges_forfft[o].size(); p++)
                {
                    if ((phasechanges_forfft[o][p] == -1.0 && phasechanges_forfft[o-1][p] == 0.0) ||
                            (phasechanges_forfft[o][p] == -1.0 && phasechanges_forfft[o-1][p] == 1.0))
                        servo_frequencies[p] = servo_frequencies[p] + 1.0;
                }

            frequency = -1.0; // max servo frequency
            for (int p =0; p < servo_frequencies.size(); p++)
                if (servo_frequencies[p] > frequency)
                    frequency = servo_frequencies[p];

            frequency = frequency / std::min(t, duration); // in case a run is cut short, and does not last till 'duration'
        }
        else
            frequency = 0.0;

        return frequency;
    }



    void _make_robot_init(float duration)
    {
        robot_t rob = this->robot();
        Eigen::Vector3d rot=rob->rot();
        _arrival_angle= atan2( cos(rot[2])* sin(rot[1])* sin(rot[0]) + sin(rot[2])* cos(rot[0]), cos(rot[2])* cos(rot[1]))*180/M_PI;

        moveRobot(rob,0);

        float t=0;
        int index = 0;
#ifdef GRAPHIC
        while (t < duration && !_visitor.done()) //!visitor, term for graphic interface (esc aborts simulation)
#else
        while (t < duration)
#endif
        {
            //std::cout << t << " torso rot 0 " << rob->rot()[0] << " torso rot 1 " << rob->rot()[1] << " torso rot 2 " << rob->rot()[2]  << std::endl;
            //returns <pitch; roll; yaw> in radians, in the interval [-pi,+pi] radians.  // pitch and roll are interchanged because the robot is now moving along the y axis

            moveRobot(rob,t);

            if(_robot->bodies()[0]->get_in_contact() || _env->get_colision_between_legs())
            {//Death if robot main body touches ground or if legs collide
#ifdef GRAPHIC
                std::cout<<"mort subite"<<std::endl;
#endif
                _covered_distance=-10002.0;
                return;
            }


            /*== STORING DATA FOR DIFFERENT BEHAVIOR DESCRIPTORS ==*/
            pitch_vec.push_back(_robot->rot()[0]); // pitch
            roll_vec.push_back(_robot->rot()[1]);  // roll
            yaw_vec.push_back(_robot->rot()[2]);   // yaw
            /*=====================================================*/

            int nbCassee=0;
            //Log contact points every iteration to better measure behav diversity for high frequency gaits
            //if (index%2==0) //Log leg contact points at every other iteration
            for (unsigned i = 0; i < 6; ++i)
            {
                switch (i)
                {
                case 0:
                    if (isBroken(i))
                    {
                        _behavior_contact_0.push_back(0);
                        nbCassee++;
                    }
                    else
                    {
                        _behavior_contact_0.push_back( _robot->bodies()[(i-nbCassee) * 3 + 3]->get_in_contact() );
                    }
                    break;
                case 1:
                    if (isBroken(i))
                    {
                        _behavior_contact_1.push_back(0);
                        nbCassee++;
                    }
                    else
                    {
                        _behavior_contact_1.push_back( _robot->bodies()[(i-nbCassee) * 3 + 3]->get_in_contact() );
                    }
                    break;
                case 2:
                    if (isBroken(i))
                    {
                        _behavior_contact_2.push_back(0);
                        nbCassee++;
                    }
                    else
                    {
                        _behavior_contact_2.push_back( _robot->bodies()[(i-nbCassee) * 3 + 3]->get_in_contact() );
                    }
                    break;
                case 3:
                    if (isBroken(i))
                    {
                        _behavior_contact_3.push_back(0);
                        nbCassee++;
                    }
                    else
                    {
                        _behavior_contact_3.push_back( _robot->bodies()[(i-nbCassee) * 3 + 3]->get_in_contact() );
                    }
                    break;
                case 4:
                    if (isBroken(i))
                    {
                        _behavior_contact_4.push_back(0);
                        nbCassee++;
                    }
                    else
                    {
                        _behavior_contact_4.push_back( _robot->bodies()[(i-nbCassee) * 3 + 3]->get_in_contact() );
                    }
                    break;
                case 5:
                    if (isBroken(i))
                    {
                        _behavior_contact_5.push_back(0);
                        nbCassee++;
                    }
                    else
                    {
                        _behavior_contact_5.push_back( _robot->bodies()[(i-nbCassee) * 3 + 3]->get_in_contact() );
                    }
                    break;
                }
            }
            _behavior_traj.push_back(rob->pos());



            // std::cout << "x " << _robot->bodies()[10]->get_pos()[0] << " y " << _robot->bodies()[10]->get_pos()[1] << " z " << _robot->bodies()[10]->get_pos()[2] << std::endl << std::endl; // 10 is first subsegment of leg 3. we print its position


            t += step;
            next_step();


            ++index;
        }

        end_time = t;

        Eigen::Vector3d prev_pos = rob->pos();


        stabilize_robot(); // post-stabilization now removed in BOMEAN



        Eigen::Vector3d next_pos = rob->pos();

        if((fabs(prev_pos[0]-next_pos[0]) > 0.4) || (fabs(prev_pos[1]-next_pos[1]) > 0.4) || (fabs(prev_pos[2]-next_pos[2]) > 0.4)) //Using for BOMEAN the same type of distance check as used by Antoine (if(fabs(_covered_distance)>10) ... see below)
        {
            //Death if robot transitions more than 40cm during the stablization phase
#ifdef GRAPHIC
            std::cout<<"mort subite"<<std::endl;
#endif
            _covered_distance=-10002.0;
            return;
        }

        _final_pos.resize(2);
        _final_pos[0]=next_pos[0];
        _final_pos[1]=next_pos[1];


        _covered_distance = round(_final_pos[1]*100) / 100.0f; // taking only the y-component of the distance travelled

        _direction=atan2(-next_pos[0],next_pos[1])*180/M_PI;
        rot=rob->rot();
        _arrival_angle= atan2( cos(rot[2])* sin(rot[1])* sin(rot[0])
                + sin(rot[2])* cos(rot[0]), cos(rot[2])* cos(rot[1]))*180/M_PI;
        while(_arrival_angle<-180)
            _arrival_angle+=360;
        while(_arrival_angle>180)
            _arrival_angle-=360;



        // detecting if a robot has flipped over
        {
            for (size_t s = 0; s < 25 && !(_robot->bodies()[0]->get_in_contact()); ++s)
            {
                size_t leg = 0;
                for (size_t i = 0; i < _robot->servos().size(); i+=3)
                {
                    for (int j=0;j<_brokenLegs.size();j++)
                    {
                        if (leg==_brokenLegs[j])
                        {
                            leg++;
                            if (_brokenLegs.size()>j+1 && _brokenLegs[j+1]!=leg)
                                break;
                        }
                    }

                    _robot->servos()[i]->set_angle(0,   0.0);
                    _robot->servos()[i+1]->set_angle(0, 0.0);
                    _robot->servos()[i+2]->set_angle(0, 0.0);

                    ++leg;
                }
                next_step();
            }

            if(_robot->bodies()[0]->get_in_contact())
            {
                _covered_distance=-10002.0;
                return;
            }
        }



        // calculating servo frequencies
        servo_frequencies_max = servo_frequency(t, duration);
    }

    std::vector<int> _brokenLegs;
    std::vector<Eigen::Vector3d> _behavior_traj;
    std::vector<float> _behavior_contact_0;
    std::vector<float> _behavior_contact_1;
    std::vector<float> _behavior_contact_2;
    std::vector<float> _behavior_contact_3;
    std::vector<float> _behavior_contact_4;
    std::vector<float> _behavior_contact_5;
    robot_t _robot;
    std::vector<float> _final_pos;
    float _direction;
    float _arrival_angle;
    float _covered_distance;
    float _slam_duration;
    float _energy;
    boost::shared_ptr<ode::Environment_hexa> _env;
#ifdef GRAPHIC
    renderer::OsgVisitor _visitor;
#endif

    std::vector<std::vector<float> > _offset_time;
    std::vector<std::vector<float> > _servo_time;
    std::vector<bool> _prev_contact;
    std::vector<bool> _contact;


    std::vector<std::vector<float> > _angles;
    std::vector<std::vector<float> > _prev_angles;
    std::vector<std::vector<float> > _angles_actual;
    std::vector<std::vector<float> > _prev_angles_actual;

    NN& _ctrlrob;
};

// should return the time since last trigger, given as input to the cppn
// output range [0, 1]
template<typename NN> float Simu<NN> :: timer(size_t leg, size_t servo, bool prev_contact, bool contact)
{

    if(prev_contact == 0 && contact == 1)
        _servo_time[leg][servo] = 0.0;

    if(_servo_time[leg][servo] > 1.0)
        _servo_time[leg][servo] = 1.0;

    return _servo_time[leg][servo];
}



template<typename NN> void Simu<NN> :: moveRobot(robot_t rob, float t)
{
    size_t tmp_leg = 0;
    for(size_t funclastlegsegment = 3; funclastlegsegment < rob->bodies().size(); funclastlegsegment+=3)
    {
        for (int j=0;j<_brokenLegs.size();j++)
        {
            if (tmp_leg==_brokenLegs[j])
            {
                _contact[tmp_leg] = false;
                tmp_leg++;
                if (_brokenLegs.size()>j+1 && _brokenLegs[j+1]!=tmp_leg)
                    break;
            }
        }
        _contact[tmp_leg] = rob->bodies()[funclastlegsegment]->get_in_contact();
        ++tmp_leg;
    }


#ifdef DETAIL_LOGS
    static std::ofstream ofs1(std::string("output_actualjointangles.dat").c_str());
    ofs1 << t << " ";
    size_t leg1 = 0;
    for (size_t i = 0; i < rob->servos().size(); i+=3)
    {
        for (int j=0;j<_brokenLegs.size();j++)
        {
            if (leg1==_brokenLegs[j])
            {
                leg1++;
                if (_brokenLegs.size()>j+1 && _brokenLegs[j+1]!=leg1)
                    break;
            }
        }

        ofs1 << " " << _contact[leg1] << " " << rob->servos()[i]->get_angle(0) << " " << _contact[leg1] << " " << rob->servos()[i+1]->get_angle(0);
        _prev_angles_actual[leg1][0] = _angles_actual[leg1][0];
        _prev_angles_actual[leg1][1] = _angles_actual[leg1][1];
        _prev_angles_actual[leg1][2] = _angles_actual[leg1][2];
        _angles_actual[leg1][0] = rob->servos()[i]->get_angle(0);
        _angles_actual[leg1][1] = rob->servos()[i+1]->get_angle(0);
        _angles_actual[leg1][2] = rob->servos()[i+2]->get_angle(0);
        ++leg1;
    }
    ofs1 << std::endl;


    static std::ofstream leglogsfs(std::string("leglogs.dat").c_str());
    leglogsfs << t << " ";
    for(size_t leg = 0; leg <= 2; ++leg)
    {
        if(_prev_angles_actual[leg][0] == _angles_actual[leg][0]) // && _prev_angles_actual[leg][0] == 0
            leglogsfs << "0"  << " " << _contact[leg] << " ";  //std::cout << "NS";
        else if(_angles_actual[leg][0] < _prev_angles_actual[leg][0])
            leglogsfs << "-1"  << " " << _contact[leg] << " ";  //std::cout << "Swing phase" << std::endl;
        else if(_angles_actual[leg][0] > _prev_angles_actual[leg][0])
            leglogsfs << "+1"  << " " << _contact[leg] << " ";  //std::cout << "Stance phase" << std::endl;
    }
    for(size_t leg = 3; leg <= 5; ++leg)
    {
        if(_prev_angles_actual[leg][0] == _angles_actual[leg][0]) // && _prev_angles_actual[leg][0] == 0
            leglogsfs << "0"  << " " << _contact[leg] << " ";   //std::cout << "NS";
        else if(_angles_actual[leg][0] > _prev_angles_actual[leg][0])
            leglogsfs << "-1"  << " " << _contact[leg] << " ";  //std::cout << "Swing phase" << std::endl;
        else if(_angles_actual[leg][0] < _prev_angles_actual[leg][0])
            leglogsfs << "+1"  << " " << _contact[leg] << " ";  //std::cout << "Stance phase" << std::endl;
    }
    leglogsfs << std::endl;
#endif

    if(t==0.0)
    {
        //generate the offsets for each of the two servos on the six legs
        //query the cppn with <x=0, y, t=0>
        //store the offset outputted by the cppn in _offset_time

        assert(_ctrlrob.substrate().size() == 1);
        size_t n = 0;
        for(size_t leg = 0; leg < 6; ++leg)
        {
            for(size_t servo = 0; servo < 2; ++servo)
            {
                float x = _ctrlrob.substrate()[0][n].get<0>();
                float y = _ctrlrob.substrate()[0][n].get<1>();

                if(servo == 0)
                {
#ifndef ORIENTFB
                    std::vector<float> r = _ctrlrob.query(boost::make_tuple(x, y, 0.0));
#else
                    std::vector<float> r = _ctrlrob.query(boost::make_tuple(x, y, 0.0), 0.0); // the heading error is 0 at the start of the simulation
#endif

                    //cppn output [-1,+1] -> [0, 1], one time period of the periodic sawtooth wave
                    _offset_time[leg][servo] = (r[1] + 1.0)/2.0;
                }
                else
                {
                    _offset_time[leg][1] = _offset_time[leg][0];
                }
                ++n;
            }
            assert(_offset_time[leg][0] == _offset_time[leg][1]);
        }
    }

#ifdef DETAIL_LOGS
    static std::ofstream ofs(std::string("output_cpnn.dat").c_str());
    ofs << t << " ";
#endif

    size_t n = 0;
    for (size_t leg = 0; leg < 6; ++leg)
    {
        if(t <= _offset_time[leg][0])
        {
            _prev_angles[leg][0] = _angles[leg][0];
            _angles[leg][0] = 0.0; // -ve leg turned ahead, +ve leg turned behind (for legs on right side of robot; vice-versa for left side legs)

#ifdef DETAIL_LOGS
            ofs << 0.0 << " " << 0.0 << " ";
#endif
        }
        else
        {
            float x = _ctrlrob.substrate()[0][n].get<0>();
            float y = _ctrlrob.substrate()[0][n].get<1>();
            float timer_output = timer(leg, 0, _prev_contact[leg], _contact[leg]);
#ifndef ORIENTFB
            std::vector<float> r = _ctrlrob.query(boost::make_tuple(x, y, timer_output));
#else
            float custom_orient = (180/ORIENTFB_ANGLE_SENSITIVITY)*rob->rot()[2]/M_PI;
            if (custom_orient > 1.0)
              custom_orient = 1.0;
            if (custom_orient < -1.0)
              custom_orient = -1.0;
            std::vector<float> r = _ctrlrob.query(boost::make_tuple(x, y, timer_output), custom_orient); // heading (rob->rot()[2]) is normalized from [-pi, pi] to range [-1,1]
#endif

            _prev_angles[leg][0] = _angles[leg][0];

            _angles[leg][0] = r[0] * M_PI/8.0;

            _servo_time[leg][0] += step;
        }
        ++n;


        if(t <= _offset_time[leg][1])
        {
            _prev_angles[leg][1] = _angles[leg][1];
            _prev_angles[leg][2] = _angles[leg][2];
            _angles[leg][1] = 0.0; // -ve leg lifted up, +ve leg pushed down
            _angles[leg][2] = 0.0;
            \
#ifdef DETAIL_LOGS
            ofs << 0.0 << " " << 0.0 << " ";
#endif
        }
        else
        {
            float x = _ctrlrob.substrate()[0][n].get<0>();
            float y = _ctrlrob.substrate()[0][n].get<1>();
            float timer_output = timer(leg, 1, _prev_contact[leg], _contact[leg]);


#ifndef ORIENTFB
            std::vector<float> r = _ctrlrob.query(boost::make_tuple(x, y, timer_output));
#else
            float custom_orient = (180/ORIENTFB_ANGLE_SENSITIVITY)*rob->rot()[2]/M_PI;
            if (custom_orient > 1.0)
              custom_orient = 1.0;
            if (custom_orient < -1.0)
              custom_orient = -1.0;
            std::vector<float> r = _ctrlrob.query(boost::make_tuple(x, y, timer_output), custom_orient); // heading (rob->rot()[2]) is normalized from [-pi, pi] to range [-1,1]
#endif

            _prev_angles[leg][1] = _angles[leg][1];
            _prev_angles[leg][2] = _angles[leg][2];

            _angles[leg][1] =  r[0] * M_PI/4.0;
            _angles[leg][2] = -r[0] * M_PI/4.0;


            _servo_time[leg][1] += step;

        }
        ++n;
    }

#ifdef DETAIL_LOGS
    ofs << std::endl;
#endif


    // recording angles output by cppn separately for the FFT
    std::vector<float> param1(18, 0.0);
    for(size_t leg = 0; leg < 6; ++leg)
    {
        for(size_t servo = 0; servo < 2; ++servo)
        {
            param1[leg * 3 + servo] = _angles[leg][servo];
        }
    }
    angles_forfft.push_back(param1);



    size_t leg = 0;
    for (size_t i = 0; i < rob->servos().size(); i+=3)
    {
        for (int j=0;j<_brokenLegs.size();j++)
        {
            if (leg==_brokenLegs[j])
            {
                leg++;
                if (_brokenLegs.size()>j+1 && _brokenLegs[j+1]!=leg)
                    break;
            }
        }

        rob->servos()[i]->set_angle(0,   _angles[leg][0]);
        rob->servos()[i+1]->set_angle(0, _angles[leg][1]);
        rob->servos()[i+2]->set_angle(0, _angles[leg][2]);

        ++leg;
    }

    for(size_t leg = 0; leg < 6; ++leg)
        _prev_contact[leg]   = _contact[leg]; //rob->bodies()[leg*3 + 3]->get_in_contact();
}

#endif
