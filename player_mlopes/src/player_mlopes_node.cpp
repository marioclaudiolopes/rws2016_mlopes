#include <iostream>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h> 

#include <rws2016_libs/team_info.h>
#include <rws2016_msgs/GameMove.h>

using namespace std;

namespace rws2016_mlopes
{
    class Player
    {
        public:

            //Constructor with the same name as the class
            Player(string name) {this->name = name;}

            int setTeamName(int team_index = 0 /*default value*/)
            {
                switch (team_index)
                {
                    case 0: 
                        return setTeamName("red"); break;
                    case 1: 
                        return setTeamName("green"); break;
                    case 2: 
                        return setTeamName("blue");  break;
                    default: 
                        cout << "wrong team index given. Cannot set team" << endl; break;
                }
            }

            //Set team name, if given a correct team name (accessor)
            int setTeamName(string team)
            {
                if (team=="red" || team=="green" || team=="blue")
                {
                    this->team = team;
                    return 1;
                }
                else
                {
                    cout << "cannot set team name to " << team << endl;
                    return 0;
                }
            }


		double getDistance(Player& p)
		{
			//computing the distance
			std::string first_refframe=p.name;
			std::string second_refframe=name;

			ros::Duration(0.01).sleep(); //To allow the listener to hear messages
               		tf::StampedTransform st; //The pose of the player
                	try{
                	    listener.lookupTransform(first_refframe, second_refframe, ros::Time(0), st);
                	}
                	catch (tf::TransformException ex){
                		ROS_ERROR("%s",ex.what());
                		ros::Duration(0.1).sleep();
                	}

                	tf::Transform t;
                	t.setOrigin(st.getOrigin());
                	t.setRotation(st.getRotation());

			double x = t.getOrigin().x();
			double y = t.getOrigin().y();

			double distNorm = sqrt(x*x + y*y);
			return distNorm;
		}

		double getAngle(string player_name)
            	{
                //computing the distance 
                string first_refframe = name;
                string second_refframe = player_name;

                ros::Duration(0.01).sleep(); //To allow the listener to hear messages
                tf::StampedTransform st; //The pose of the player
                try{
                    listener.lookupTransform(first_refframe, second_refframe, ros::Time(0), st);
                }
                catch (tf::TransformException& ex){
                    ROS_ERROR("%s",ex.what());
                    ros::Duration(0.1).sleep();
                }

                tf::Transform t;
                t.setOrigin(st.getOrigin());
                t.setRotation(st.getRotation());

                double x = t.getOrigin().x();
                double y = t.getOrigin().y();

                double angle = atan2(y,x);
                return angle;

            	}


            //Gets team name (accessor)
            string getTeamName(void) {return team;}

           /**
             * @brief Gets the pose (calls updatePose first)
             *
             * @return the transform
             */
            tf::Transform getPose(void)
            {
                ros::Duration(0.01).sleep(); //To allow the listener to hear messages
                tf::StampedTransform st; //The pose of the player
                try{
                    listener.lookupTransform("/map", name, ros::Time(0), st);
                }
                catch (tf::TransformException ex){
                    ROS_ERROR("%s",ex.what());
                    ros::Duration(0.1).sleep();
                }

                tf::Transform t;
                t.setOrigin(st.getOrigin());
                t.setRotation(st.getRotation());
                return t;
            }

            string name; //A public atribute

        private:
            string team;
            tf::TransformListener listener; //gets transforms from the system
    };

	class Team
	{
		public:
	
		Team(std::string team, std::vector<std::string> player_names)
		{
			name=team;
			for (size_t i=0; i < player_names.size(); ++i)
			{

				boost::shared_ptr<Player> p(new Player (player_names[i]));
				p->setTeamName(name);
				players.push_back(p);
			}
		}

		void printTeamInfo(void)
		{

			cout << "Team " << name << " has the following players:" << endl;

			for(size_t i=0; i<players.size(); ++i)
				cout << players[i]->name << endl;
		}

		string name;
		std::vector<boost::shared_ptr<Player> >players;

	};


	//Class myPlayer extends class Player
	class MyPlayer: public Player
	{
   		public: 

		tf::TransformBroadcaster br;

		/**
             * @brief The teams
             */
            boost::shared_ptr<Team> my_team;
            boost::shared_ptr<Team> hunter_team;
            boost::shared_ptr<Team> prey_team;

            boost::shared_ptr<ros::Subscriber> _sub; 
		
//("green", prey_names);


   		MyPlayer(std::string name, std::string team): Player(name)
   	 	{
//   	     		setTeamName(team);

//			//Initialize position to 0,0,0
//            		tf::Transform t;
//            		t.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
//            		tf::Quaternion q; q.setRPY(0, 0, 0);
//            		t.setRotation(q);
//            		br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "/map", name));

			setTeamName(team);
            ros::NodeHandle node;

            //Initialize teams
            vector<string> myTeam_names, myHunters_names, myPreys_names;
            string myTeamId, myHuntersId, myPreysId;

            if (!team_info(node, myTeam_names, myHunters_names, myPreys_names, myTeamId, myHuntersId, myPreysId))
                ROS_ERROR("Something went wrong reading teams");

            my_team = (boost::shared_ptr<Team>) new Team(myTeamId, myTeam_names);
            hunter_team = (boost::shared_ptr<Team>) new Team(myHuntersId, myHunters_names);
            prey_team = (boost::shared_ptr<Team>) new Team(myPreysId, myPreys_names);

            my_team->printTeamInfo();
            hunter_team->printTeamInfo();
            prey_team->printTeamInfo();

            //Initialize position according to team
            ros::Duration(0.5).sleep(); //sleep to make sure the time is correct
            tf::Transform t;
            //srand((unsigned)time(NULL)); // To start the player in a random location
            struct timeval t1;      
            gettimeofday(&t1, NULL);
        srand(t1.tv_usec);
            double X=((((double)rand()/(double)RAND_MAX) ) * 2 -1) * 5 ;
            double Y=((((double)rand()/(double)RAND_MAX) ) * 2 -1) * 5 ;
            t.setOrigin( tf::Vector3(X, Y, 0.0) );
            tf::Quaternion q; q.setRPY(0, 0, 0);
            t.setRotation(q);
            br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "/map", name));

            //initialize the subscriber
            _sub = (boost::shared_ptr<ros::Subscriber>) new ros::Subscriber;
            *_sub = node.subscribe("/game_move", 1, &MyPlayer::moveCallback, this);

//			vector<string> myteam_names;
//	myteam_names.push_back("mlopes");
//	rws2016_mlopes::Team my_team("red", myteam_names);
//	my_team.printTeamInfo();

//	vector<string> prey_names;
//	prey_names.push_back("dsilva");
//	rws2016_mlopes::Team prey_team("green", prey_names);
//	prey_team.printTeamInfo();

//	vector<string> hunter_names;
//	hunter_names.push_back("torrao");
//	rws2016_mlopes::Team hunter_team("blue", hunter_names);
//	hunter_team.printTeamInfo();
			
//			vector<string> prey_names;
//			prey_names.push_back("dsilva");
//			Team prey_team("green", prey_names);
//			//prey_team = (boost::shared_ptr<Team>) new Team("green", prey_names);
//			prey_team.printTeamInfo();

//			cout << prey_team.players[0]->name << endl;
   	 	}

		/**
 		* @brief Moves MyPlayer
 		*
 		* @param displacement the liner movement of the player, bounded by [-0.1, 1]
 		* @param turn_angle the turn angle of the player, bounded by  [-M_PI/60, M_PI/60]
 		*/
 		void move(double displacement, double turn_angle)
 		{
			double dispMax=1;
			double dispMin=-0.1;
			double angBound=M_PI/30;

			// Upper Bound
			if (displacement>dispMax)
				displacement=dispMax;

			// Lower Bound
			if (displacement<dispMin)
				displacement=dispMin;

			if (turn_angle>angBound)
				turn_angle=angBound;
			else if (turn_angle<-angBound)
				turn_angle=-angBound;

 			tf::Transform transform;
 			transform.setOrigin( tf::Vector3(displacement, 0, 0.0) );

  			tf::Quaternion q;
  			q.setRPY(0, 0, turn_angle);
  			transform.setRotation(q);
	
			tf::Transform t = getPose();
                	t = t  * transform;

			

  			//Send the new transform to ROS
                	br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "/map", name));
 		}


		string getNameOfClosestPrey(double *distPrey)
            	{
                	double prey_dist = getDistance(*prey_team->players[0]);
                	string prey_name = prey_team->players[0]->name;

                	for (size_t i = 1; i < prey_team->players.size(); ++i)
                	{
                   	 	double d = getDistance(*prey_team->players[i]);

                    		if (d < prey_dist) //A new minimum
                    		{
                        		prey_dist = d;
                        		prey_name = prey_team->players[i]->name;
                    		}
                	}
			
			*distPrey = prey_dist;

                	return prey_name;
            	}

		string getNameOfClosestHunter(double *distHunt)
            	{
                	double hunter_dist = getDistance(*hunter_team->players[0]);
                	string hunter_name = hunter_team->players[0]->name;

                	for (size_t i = 1; i < hunter_team->players.size(); ++i)
                	{
                		double d = getDistance(*hunter_team->players[i]);

                    		if (d < hunter_dist) //A new minimum
                    		{
                       			hunter_dist = d;
                       			hunter_name = hunter_team->players[i]->name;
                    		}
                	}
			*distHunt = hunter_dist;

               		return hunter_name;
            	}
		
	/**
             * @brief called whenever a /game_move msg is received
             *
             * @param msg the msg with the animal values
             */
            void moveCallback(const rws2016_msgs::GameMove& msg)
            {
                ROS_INFO("player %s received game_move msg", name.c_str()); 

                //I will encode a very simple hunting behaviour:
                //
                //1. Get closest prey name
                //2. Get angle to closest prey
                //3. Compute maximum displacement
                //4. Move maximum displacement towards angle to prey (limited by min, max)

		double distClosPrey;
		double distClosHunt;

                //Step 1
                string closest_prey = getNameOfClosestPrey( &distClosPrey);
		string closest_hunt = getNameOfClosestHunter( &distClosHunt);

                ROS_INFO("Closest prey is %s", closest_prey.c_str());

                //Step 2
		double angle;
		if (distClosPrey<distClosHunt)
			angle = getAngle(closest_prey);
		else
			angle = M_PI+getAngle(closest_hunt);

                //Step 3
                double displacement = msg.dog; //I am a dog, others may choose another animal

                //Step 4
                move(displacement, angle);

            }


	};

    //class Team  ///Hidden for better visualization

} //end of namespace rws2016_mlopes

/**
 * @brief The main function
 *
 * @param argc number of command line arguments
 * @param argv values of command line arguments
 *
 * @return result
 */
int main(int argc, char** argv)
{
    //initialize ROS stuff
    ros::init(argc, argv, "mlopes");
    ros::NodeHandle node;

    //Creating an instance of class MyPlayer
    rws2016_mlopes::MyPlayer my_player("mlopes", "red");

    //Infinite loop
    ros::spin();
}

			// double ang = t.getRotation().z(); // Para rotação
