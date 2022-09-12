
CODE_KEYWORD_DICT = {
    "getPlayerName" : "It looks like there's a switch statement in that get player name function. Can you explain how that should work?",
    "placeChar"     : "Looks like that place char function has just a single if statement, then does something in the body. What do you think it's doing?",
    "checkWin"      : "Looks like the check Win function has a for loop, and lots of conditionals. What are all those for?",
    "checkFull"     : "Hm, how does the checkFull function decide whether the board is full?",
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
COMPILATION_ERROR_POOL = [
    "We got a compilation error. Can we take a look?",
    "Looks like it didn't compile. Let's see what it says.",
    "Oh, it looks like it didn't compile that time. Let's see what the error was."
]

RUNTIME_ERROR_POOL = [
    "It compiled right, but we got an error at runtime. Let's see the debugger output.",
    "Hmmm... it looks like the code had an error when it ran. Let's see."
]

OUTPUT_ERROR_POOL = [
    "Aw, our code compiled okay but ran into a test error. ",
    "It looks like we might have failed one of the built-in tests. ",
    "Our code compiled and ran, but it looks like the output it produced wasn't quite right yet. "
]

SUCCESS_POOL = [
    "You passed all the tests, that's amazing! Great work!"
]

THINKALOUD_PROMPTS_GENERIC = [
    "Can you tell me what you're thinking?",
    "What do you think the next step is?",
    "Can you explain your current process to me?"
]