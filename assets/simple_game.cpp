#include <iostream>
#include <vector>
#include <cstdlib>

using namespace std;

class TicTacToeGame {
    private:
        char board[3][3] = {
            {' ', ' ', ' '},
            {' ', ' ', ' '},
            {' ', ' ', ' '}
        };
        int filled_squares = 0;

    public:
        void placeChar(char c, int row, int col) {
            if (row >= 3 || col >= 3) {
                cout << "placeChar: invalid" << endl;
            } else if (board[row][col] != ' ') {
                cout << "placeChar: spot not empty" << endl;
            } else {
                board[row][col] = c;
                filled_squares += 1;
            }
        }

        void printBoard() {
            cout << "   0 | 1 | 2" << endl;
            cout << "   _________" << endl;
            for (int i = 0; i < 3; i++) {
                cout << i;
                for (int j = 0; j < 3; j++) {
                    cout << "| " << board[i][j] << " ";
                }
                cout << endl << "   _________" << endl;

            }
        }

        bool checkWin(char c) {
            // check for 3 in a row horizontally and vertically
            for (int i = 0; i < 3; i++) {
                if (board[i][0] == c && board[i][1] == c && board[i][2] == c) {
                    return true;
                } else if (board[0][i] == c && board[1][i] == c && board[2][i] == c) {
                    return true;
                }
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
            return (board[row][col] == ' ');
        }
};

void playGame() {
    TicTacToeGame game;
    int row;
    int col;
    do {
        game.printBoard();
        // get move from player
        cout << "Enter row of your move: " << endl;;
        cin >> row;

        cout << "Enter col of your move: " << endl;
        cin >> col;

        game.placeChar('x', row, col);

        if (game.checkWin('x')) {
            game.printBoard();
            cout << "You win!" << endl;
            return;
        } else if (game.checkFull()) {
            cout << "Draw!" << endl;
            return;
        }

        // make a random move
        while(true) {
            row = rand() % 3;
            col = rand() % 3;

            if(game.checkEmptySquare(row, col)) {
                game.placeChar('o', row, col);
                break;
            }
        }

        if (game.checkWin('o')) {
            game.printBoard();
            cout << "Computer wins!" << endl;
            return;
        }

    } while (!game.checkFull());
    cout << "Draw!" << endl;
}

void testGame(int test_number) {
    string expected_outcome;
    int move_count = 0;
    vector<vector<int>> move_list;
    
    switch (test_number) {
        case 1:
            move_list.insert(move_list.end(), {{1, 1}, {0, 2}, {2, 2}, {1, 0}, {0, 0}});
            move_count = 5;
            expected_outcome = "x";
            break;
        case 2:
            move_list.insert(move_list.end(), {
                {2, 2},
                {0, 0},
                {1, 0},
                {0, 2},
                {2, 1},
                {0, 1}
            });
            move_count = 6;
            expected_outcome = "o";
            break;
        case 3:
        default:
            move_list.insert(move_list.end(), {
                {1, 1},
                {0, 0},
                {0, 1},
                {1, 0},
                {1, 2},
                {0, 2},
                {2, 0},
                {2, 1},
                {2, 2}
            });
            move_count = 9;
            expected_outcome = "draw";
            break;
    }

    TicTacToeGame game;
    for(int i = 0; i < move_count; i+=2) {
        game.placeChar('x', move_list[i][0], move_list[i][1]);
        game.printBoard();
        if(i+1 >= move_count) {
            break;
        }
        game.placeChar('o', move_list[i+1][0], move_list[i+1][1]);
        game.printBoard();
    }

    bool x_win = game.checkWin('x');
    bool o_win = game.checkWin('o');
    bool is_full = game.checkFull();

    if (expected_outcome == "x") {
        if(x_win && !o_win) {
            cout << "got expected outcome: x wins, o loses" << endl;;
        } else {
            cout << "got unexpected outcome: expected x wins, o loses" << endl;
        }
    } else if (expected_outcome == "o") {
        if(o_win && !x_win) {
            cout << "got expected outcome: o wins, x loses" << endl;
        } else {
            cout << "got unexpected outcome: expected o wins, x loses" << endl;
        }
    } else {
        // expected draw
        if(is_full && !x_win && !o_win) {
            cout << "got expected outcome: DRAW, neither side wins" << endl;
        } else {
            cout << "got unexpected outcome: expected DRAW, neither side wins" << endl;
        }
    }
}

int main() {
    testGame(3);
    return 0;
}












