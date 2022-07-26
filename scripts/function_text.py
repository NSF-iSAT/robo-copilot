CODE_KEYWORD_DICT = {
    "getPlayerName" : "It looks like there's a switch statement in that get player name function. Can you explain how that should work?",
    "placeChar"     : "Looks like that place char function has a lot of conditionals. What do those do?",
    "checkWin"      : "Looks like the check Win function has a lot of conditionals. What is that function trying to do?",
    "checkFull"     : "Hm, can you explain how the checkFull function works?",
    "checkEmptySquare" : "How is the check empty square function supposed to work?"
}


FUNCTION_DICT = {
    "getPlayerName" : """
            string name;
            switch (player_num) {
                case 1:
                    name = player1_name;
                    
                case 2:
                    name = player2_name;
                    break;
            }
            return name;""",
    "placeChar" : """
            if (row >= 3 || col >= 3) {
                cout << "placeChar: invalid" << endl;
            } else if (board[row][col] != ' ') {
                cout << "placeChar: spot not empty" << endl;
            } else {
                board[row][col] = c;
                
            }
        """,
    "printBoard" : """
            cout << "   0 | 1 | 2" << endl;
            cout << "   _________" << endl;
            for (int i = 0; i < 3; i++) {
                cout << i;
                for (int i = 0; i < 3; i++) {
                    cout << "| " << board[i] << " ";
                }
                cout << endl << "   _________" << endl;

            }
    """,
    "checkWin" : """
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
    """,
    "checkFull" : "return (filled_squares >= 8);",
    "checkEmptySquare" : "return (board[row][col] = ' ');"
}