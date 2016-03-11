#include <iostream>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h> 

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

			ros::Duration(0.1).sleep(); //To allow the listener to hear messages
               		tf::StampedTransform st; //The pose of the player
                	try{
                	    listener.lookupTransform(first_refframe, second_refframe, ros::Time(0), st);
                	}
                	catch (tf::TransformException ex){
                		ROS_ERROR("%s",ex.what());
                		ros::Duration(1.0).sleep();
                	}

                	tf::Transform t;
                	t.setOrigin(st.getOrigin());
                	t.setRotation(st.getRotation());

			double x = t.getOrigin().x();
			double y = t.getOrigin().y();

			double distNorm = sqrt(x*x + y*y);
			return distNorm;
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
                ros::Duration(0.1).sleep(); //To allow the listener to hear messages
                tf::StampedTransform st; //The pose of the player
                try{
                    listener.lookupTransform("/map", name, ros::Time(0), st);
                }
                catch (tf::TransformException ex){
                    ROS_ERROR("%s",ex.what());
                    ros::Duration(1.0).sleep();
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

		boost::shared_ptr<Team> prey_team;
		
//("green", prey_names);


   		MyPlayer(std::string name, std::string team): Player(name)
   	 	{
   	     		setTeamName(team);

			//Initialize position to 0,0,0
            		tf::Transform t;
            		t.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
            		tf::Quaternion q; q.setRPY(0, 0, 0);
            		t.setRotation(q);
            		br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "/map", name));


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
			
			vector<string> prey_names;
			prey_names.push_back("dsilva");
			Team prey_team("green", prey_names);
			//prey_team = (boost::shared_ptr<Team>) new Team("green", prey_names);
			prey_team.printTeamInfo();

			cout << prey_team.players[0]->name << endl;
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
			double angBound=M_PI/60;

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


		



	};

	

    //class Team  ///Hidden for better visualization

} //end of namespace rws2016_moliveira

int main(int argc, char** argv)
{
    //initialize ROS stuff
    ros::init(argc, argv, "player_mlopes_node");
    ros::NodeHandle node;

    //Creating an instance of class MyPlayer
    rws2016_mlopes::MyPlayer my_player("mlopes", "red");




	rws2016_mlopes::Player lalmeida_player("lalmeida");

    	//Infinite loop
    	ros::Rate loop_rate(10);
    	while (ros::ok())
    	{
       		//Test the get pose method                                                                                     
        	tf::Transform t = my_player.getPose();
        	cout << "x = " << t.getOrigin().x() << " y = " << t.getOrigin().y() << endl;

		double dist_from_my_player_to_lalmeida = my_player.getDistance(lalmeida_player);
        	cout << "dist_from_my_player_to_lalmeida = " << dist_from_my_player_to_lalmeida << endl;

		//Test the move method
        	my_player.move(0.1, -M_PI/6);

        	ros::spinOnce();
        	loop_rate.sleep();
    	}

    	return 0;
}

			// double ang = t.getRotation().z(); // Para rotação
