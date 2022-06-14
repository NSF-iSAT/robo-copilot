#include <iostream>
using namespace std;

class Player {
    private:
        string name;
        int health;
        int money;
    
    public:
        Player(string player_name, int player_health, int player_money) {
            name = player_name;
            health = player_health;
            money = player_money;
        }

        string get_name() {
            return name;
        }
        int get_health() {
            return health; 
        }
        int get_money() {
            return money;
        }

        void set_health(int new_health) {
            health = new_health;
        }
        void set_money(int new_money) {
            money = new_money;
        }
};

int fightEnemy(Player player, int enemy_power) {
    int player_attack = rand() % player.get_health();

    if (player_attack > enemy_power) {
        cout << "You won the fight!" << endl;
        return true;
    } else {
        return false;
    }
}


int main()
{
    cout << "Hello, World!" << endl;
    return 0;
}