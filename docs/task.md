# A C++ Task for Two
Deciding on a task for the evaluation of my system was tough. I wanted it to be hard enough that students who had taken a semester of C++ would still struggle a bit, but not so difficult that they got really frustrated or completely stuck.

I ended up looking to existing literature on debugging to see the kinds of artificially-bugged programs they used. There seemed to be some consensus that syntax errors are, well, boring -- if nothing else, the g++ error messages associated with syntax errors are either too helpful, or not helpful at all (if you've done some C/C++ programming you probably know what I mean). So they drew from more runtime-type errors instead, using other common mistakes like Boolean logic errors, conditional mistakes, infinite loops, and so on. This led me to add the following errors:

1. Duplicated loop variable in nested loop
2. Missing 'break' in a switch statement
3. Swapped row-column variables when indexing into 2D array
4. = instead of ==
5. Missing variable update
6. Incorrect loop condition (off-by-1)
7. || instead of && in conditional
8. Incorrect variable used to index into array*

The task I chose was a TicTacToe implementation program. I thought this would be good as it provides a very visual indicator of whether the program is working correctly. It also provides opportunities to include common novice sticking points like arrays, loops, and tricky conditions.

In addition to the main task code, I included a function that runs a suite of tests on the implementation. These will hopefully make the bugs' behavior evident without giving away what the particular error is, and, conversely, make it clear when bugs have been solved. The test case output can also be read by the feedback generation script, so the robot can cue into test failure (or success) and react appropriately.  While participants aren't allowed to edit the test itself, they can add their own tests if they think it will help localize an error (I'm intrigued as to whether any participants will do this).

Notably, only the first 7 errors will actually be caught by the tests I've built -- the 8th is in a function that is never called. I added this one just to see whether people catch it when debugging the code. Will it be indicative of their debugging strategy?

Now that the task is (essentially) finalized, all that's left is to tune the robot's feedback accordingly and start subjecting my colleagues to my little gauntlet. Science is fun!