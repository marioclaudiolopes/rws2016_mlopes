#include <iostream>




class Player
{
	public:

	//Constructor with the same name as the class
	Player(std::string name) { this->name = name;}



	int setTeamName(int team_index = 0)
	{

		if(team_index==0)
			setTeamName("red");
		else if (team_index==1)
			setTeamName("green");
		else if (team_index==2)
			setTeamName("blue");
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
	std::string getTeam(void) {return team;}

	std::string name; //A public atribute

	private:
	std::string team;
};



int main()
{
   
	//Creating an instance of class Player
	Player player("mlopes");
	player.setTeamName("red");
	player.setTeamName();

	std::cout << "player.name = " << player.name << std::endl;
	std::cout << "player.team = " << player.getTeam() << std::endl;
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



