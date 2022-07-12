#include <iostream>
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
            if (row > 3 || col > 3) {
                cout << "placeChar: invalid" << endl;
            } else if (board[row][col] != ' ') {
                cout << "placeChar: spot not empty" << endl;
            } else {
                board[row][col] = c;
                filled_squares += 1;
            }
        }

        void printBoard() {
            cout << "_______" << endl;
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    cout << "|" << board[i][j];
                }
                cout << "|" << endl << "_______" << endl;

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
            cout << "Computer wins!" << endl;
            return;
        }
        game.printBoard();

    } while (!game.checkFull());
    cout << "Draw!" << endl;
}

int main() {
    // TicTacToeGame game;
    // game.printBoard();
    playGame();
    return 0;
}

