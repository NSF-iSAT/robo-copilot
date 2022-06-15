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

class GameState {
    private:
        int current_room;
        int max_rooms;
        int player_power;
    public:
        GameState(int current_room, int max_rooms, int player_power) {
            this->current_room = current_room;
            this->max_rooms = max_rooms;
            this->player_power = player_power;
        }
        int get_current_room() {
            return current_room;
        }
        int get_max_rooms() {
            return max_rooms;
        }
        int get_player_power() {
            return player_power;
        }
        void set_current_room(int new_room) {
            current_room = new_room;
        }
        void set_player_power(int new_power) {
            player_power = new_power;
        }
}

int fightEnemy(Player player, int enemy_power) {
    int player_attack = rand() % player.get_health();

    if (player_attack > enemy_power) {
        cout << "You won the fight!" << endl;
        return true;
    } else {
        return false;
    }
}

void shopMenu(Player player, GameState game_state) {
    cout << "Welcome to the shop!" << endl;
    cout << "You have $" << player.get_money() << "." << endl;
    cout << "What would you like to do?" << endl;
    cout << "1. Buy health" << endl;
    cout << "2. Buy power" << endl;
    cout << "3. Buy armor" << endl;
    cout << "4. Buy a weapon" << endl;
    cout << "5. Leave" << endl;
    int choice;
    cin >> choice;
    switch (choice) {
        case 1:
            if (player.get_money() >= 10) {
                player.set_money(player.get_money() - 10);
                player.set_health(player.get_health() + 10);
                cout << "You bought health for $10." << endl;
            } else {
                cout << "You don't have enough money." << endl;
            }
            break;
        case 2:
            if (player.get_money() >= 10) {
                player.set_money(player.get_money() - 10);
                game_state.set_power(game_state.get_power() + 10);
                cout << "You bought power for $10." << endl;
            } else {
                cout << "You don't have enough money." << endl;
            }
            break;
        case 3:
            if (player.get_money() >= 10) {
                player.set_money(player.get_money() - 10);
                game_state.set_armor(game_state.get_armor() + 10);
                cout << "You bought armor for $10." << endl;
            } else {
                cout << "You don't have enough money." << endl;
            }
            break;
        case 4:
            if (player.get_money() >= 10) {
                player.set_money(player.get_money() - 10);
                game_state.set_weapon(game_state.get_weapon() + 10);
            } else {
                cout << "You don't have enough money." << endl;
            }
    }
}

int main()
{
    cout << "Hello, World!" << endl;
    return 0;
}







