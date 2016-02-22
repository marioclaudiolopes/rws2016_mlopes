#include <iostream>
#include <vector>
#include <boost/shared_ptr.hpp>





using namespace std;                                                                                    

namespace rws2016_mlopes
{

	class Player /*hidden for better visualization*/
	{
		public:

		//Constructor with the same name as the class
		Player(std::string name) { this->name = name;}



		int setTeamName(int team_index = 0)
		{

			if(team_index==0)
				return setTeamName("red");
			else if (team_index==1)
				return setTeamName("green");
			else if (team_index==2)
				return setTeamName("blue");
			else
				std::cout << "wrong team index given. Cannot set team" << std::endl;
		}


		//Set team name, if given a correct team name (accessor)
		int setTeamName(std::string team)
		{
        		if (team=="red" || team=="green" || team=="blue")
        		{
				this->team = team;
				return 1;
       			}
        		else
        		{
				std::cout << "cannot set team name to " << team << std::endl;
				return 0;
			}
		}

		//Gets team name (accessor)
		std::string getTeamName(void) {return team;}

		std::string name; //A public atribute

		private:
		std::string team;

	};

//Class myPlayer extends class Player
class MyPlayer: public Player /*hidden for better visualization*/
	{
   	 public: 

   	 MyPlayer(std::string name, std::string team): Player(name)
   	 {
   	     setTeamName(team);
   	 }

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
		for(size_t i=0; i<players.size(); ++i)
			cout << players[i]->name << endl;
	}

	string name;
	std::vector<boost::shared_ptr<Player> >players;

};

} //end of namespace rws2016_mlopes


int main()
{

	//Creating an instance of class Player
//    	rws2016_mlopes::MyPlayer player("mlopes","green");
//    	std::cout << "my_player.name =" << player.name << std::endl;
//    	std::cout << "my_player.team =" << player.getTeamName() << std::endl;
//	//Creating an instance of class Player
//	Player player("mlopes");
//	player.setTeamName("red");
//	player.setTeamName();

//	std::cout << "player.name = " << player.name << std::endl;
//	std::cout << "player.team = " << player.getTeam() << std::endl;

//	std::vector<int> as;
//	int myint=5;
//	as.push_back(myint);
//	as.push_back(myint);
//	as.push_back(myint);

//	for (size_t i=0; i< as.size(); ++i)
//	{
//		std::cout << "as[" << i << "]=" << as[i] << endl;
//	}	

	vector<string> players;
    	players.push_back("moliveira");
    	players.push_back("vsantos");
    	players.push_back("pdias");

    	rws2016_mlopes::Team team( "green", players);
    	team.printTeamInfo();


	return 1;                                                      
}


//class Player //Com duas entradas
//{
//	public:

//	//Constructor with the same name as the class
//	Player(std::string name, std::string team) 
//	{ 
//		this->name = name;

//		if (team=="red" || team=="green" || team=="blue")
//        	{
//			this->team = team;
//			//return 1;
//       		}
//      	else
//     	{
//			std::cout << "cannot set team name to " << team << std::endl;
//			//return 0;
//		}

//	}


//	//Gets team name (accessor)
//	std::string getTeam(void) {return team;}

//	std::string name; //A public atribute

//	private:
//	std::string team;
//};


//int main()
//{
//   
//	//Creating an instance of class Player
//	Player player("mlopes","red");
////	player.setTeamName("red");

//	std::cout << "player.name is " << player.name << std::endl;
//	std::cout << "team is " << player.getTeam() << std::endl;
//	return 1;                                                      
//}



