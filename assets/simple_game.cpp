#include <iostream>
#include <cstdlib>
using namespace std;

class Player {
    private:
        int health;
        int level;
        string name;

    public:
        Player(int starting_health, int starting_level, string player_name) {
            health = starting_health;
            level = starting_level;
            name = player_name;
        }

        void setHealth(int new_health) {
            health = new_health;
        }
        void levelUp() {
            level += 1;
        }

        int getHealth() {
            return health;
        }
        int getLevel() {
            return level;
        }
        string getName() {
            return name;
        }
};

class Enemy {
    private:
        string name;
        int health;
        int power;
    public:
        Enemy(int enemy_health, int enemy_power, string enemy_name) {
            health = enemy_health;
            power = enemy_power;
            name = enemy_name;
        }
        void setHealth(int new_health) {
            health = new_health;
        }
        
        int getHealth() {
            return health;
        }
        int getPower() {
            return power;
        }
        string getName() {
            return name;
        }
};

int rollDice(int sides) {
    return rand() % sides + 1;
}

bool fightEnemy(Player player, Enemy enemy) {
    cout << "Enemy fight start!" << endl;
    cout << player.getName() << " VS. " << enemy.getName() << endl;
    
    bool still_playing = true;
    char menu_choice;
    int damage;
    int dice_roll;

    do {
        cout << "// Turn start!" << endl;
        cout << "// Your HP: " << player.getHealth() << endl;
        cout << "// Enemy HP: " << enemy.getHealth() << endl;
        cout << "Do you (A) Attack or (B) Run Away?" << endl;
        cin >> menu_choice;

        if(menu_choice == 'A' || menu_choice == 'a') {
            // Player chooses to attack!
            damage = rand() % player.getLevel();
            cout << "You attack, dealing " << damage << " points of damage!" << endl;

            enemy.setHealth(enemy.getHealth() - damage);
        } else {
            // Player chooses to run away!
            // Roll a dice to see if you get away safely...
            dice_roll = rollDice(5);
            // There is a 1 in 5 chance you don't get away safely
            if (dice_roll == 1) {
                cout << "You couldn't get away!" << endl;
            } else {
                cout << "You escaped!" << endl;
                return true;
            }
        }

        if(enemy.getHealth() > 0) {
            // If enemy is still alive, it attacks
            damage = rand() % enemy.getPower();
            cout << "The enemy attacks you, dealing " << damage << " points of damage!" << endl;
            player.setHealth(player.getHealth() - damage);
        } else {
            // Enemy defeated!
            cout << "Success! You defeated the enemy. You leveled up." << endl;
            player.levelUp();
            cout << "You are now level " << player.getLevel() << endl;
        }

    } while (player.getHealth() > 0 && enemy.getHealth() > 0);

    if (player.getHealth() <= 0) {
        cout << "GAME OVER" << endl;
        cout << "You died :(" << endl;
        cout << "You made it to level: " << player.getLevel() << endl;
        return false;
    } else {
        // Player survived the encounter
        return true;
    }
}

int main() {
    Player player1 (100, 10, "Misty");
    Enemy enemy1 (20, 20, "Sniper");

    fightEnemy(player1, enemy1);

    return 0;
}