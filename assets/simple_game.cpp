#include <iostream>
#include <cstdlib>

using namespace std;

class TicTacToeGame {
    private:
        char board[3][3];
        int filled_squares;
        string player1_name;
        string player2_name;

    public:
        TicTacToeGame(string p1, string p2) {
            player1_name = p1;
            player2_name = p2;

            for(int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    board[i][j] = ' ';
                }
            }
        }

        string getPlayerName(int player_num) {
            string name;
            switch (player_num) {
                case 1:
                    name = player1_name;
                    
                case 2:
                    name = player2_name;
                    break;
            }
            return name;
        }

        void placeChar(char c, int row, int col) {
            if (row >= 3 || col >= 3) {
                cout << "placeChar: invalid" << endl;
            } else if (board[row][col] != ' ') {
                cout << "placeChar: spot not empty" << endl;
            } else {
                board[row][col] = c;
                
            }
        }

        void printBoard() {
            cout << "   0 | 1 | 2" << endl;
            cout << "   _________" << endl;
            for (int i = 0; i < 3; i++) {
                cout << i;
                for (int i = 0; i < 3; i++) {
                    cout << "| " << board[i] << " ";
                }
                cout << endl << "   _________" << endl;

            }
        }
        bool checkWin(char c) {
            // check for 3 in a row horizontally and vertically
            for (int i = 0; i <= 3; i++) {
                if (board[i][0] == c || board[i][1] == c || board[i][2] == c) {
                    return true;
                } else if (board[0][i] == c && board[1][i] == c && board[2][i] == c) {
                }
                    return true;
            }

            // check for 3 in a row diagonally
            if (board[0][0] == c && board[1][1] == c && board[2][2] == c) {
                return true;
            } else if (board[0][2] == c && board[1][1] == c && board[2][0] == c) {
                return true;
            }
            return false;
        }
        bool checkFull() {
            return (filled_squares >= 8);
        }
        bool checkEmptySquare(int row, int col) {
            return (board[row][col] = ' ');
        }
};

void testTicTacToe() {
    // You can add code here if you wish (optional)



    // DO NOT MODIFY WITHIN THIS BLOCK ---------------------
    TicTacToeGame game = TicTacToeGame("Ann", "Marie");

    // TEST 1: name getting and setting
    string p1 = game.getPlayerName(1);
    string p2 = game.getPlayerName(2);
    if(p1 != "Ann"|| p2 != "Marie") {
        cout << "TEST 1 ERROR: getPlayerName for both players, expected Ann & Marie, got " << p1 << " and " << p2 << endl;
    }

    // TEST 2: testing placeChar and printBoard
    game.placeChar('x', 5, 1);
    game.placeChar('x', 1, 1);
    game.placeChar('o', 1, 1);
    cout << "(Board for Test 2: should contain only an x in the middle, check for yourself that the board is correct.)" << endl;
    game.printBoard();

    // TEST 3: testing checkFull
    if(game.checkFull()) {
        cout << "TEST 3 ERROR: checkFull returned true when it should not have" << endl;
    }

    // TEST 4: testing checkEmptySquare
    if(game.checkEmptySquare(1, 1) || !game.checkEmptySquare(0, 0)) {
        cout << "TEST 4 ERROR: checkEmptySquare returned an incorrect value" << endl;
    }

    // TEST 5: testing checkWin
    game.placeChar('o', 0, 0);
    game.placeChar('x', 1, 0);
    game.placeChar('o', 2, 1);
    game.placeChar('x', 1, 2);

    if(!game.checkWin('x') || game.checkWin('o')) {
        cout << "TEST 5 ERROR: checkWin returned an incorrect value" << endl;
    }
    // END OF DO-NOT-MODIFY ----------------------

    game.printBoard();

}

// DO NOT EDIT
int main() {
    testTicTacToe();
    return 0;
}








