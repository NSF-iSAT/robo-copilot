# A C++ Task for Two
Deciding on a task for the evaluation of my system was tough. I wanted it to be hard enough that students who had taken a semester of C++ would still struggle a bit, but not so difficult that they got really frustrated or completely stuck.

I ended up looking to existing literature on debugging to see the kinds of artificially-bugged programs they used. There seemed to be some consensus that syntax errors are, well, boring -- if nothing else, the g++ error messages associated with syntax errors are either too helpful, or not helpful at all (if you've done some C/C++ programming you probably know what I mean). So they drew from more runtime-type errors instead, using other common mistakes like Boolean logic errors, conditional mistakes, infinite loops, and so on. This led me to add the following errors:

1. Nested loop error -- for-loops both use `i` when iterating over a 2D array
2. Boolean logic error (|| instead of &&)
3. Switch-statement logic error -- missing a `break` between cases
4. Missing statement -- count variable never gets updated
5. = instead of == (arguably also a logic error, as the resulting statement always evaluates to `true`)
6. Improper indexing into a 2D array (`a[i]` instead of `a[i][j]`)
7. Missing count variable initialization
8. Misplaced return statement (outside of `if` body when it should be inside)

The task I chose was a TicTacToe implementation program. I thought this would be good as it provides a very visual indicator of whether the program is working correctly. It also provides opportunities to include common novice sticking points like arrays, loops, and tricky conditions.

In addition to the main task code, I included a function that runs a suite of tests on the implementation. These will hopefully make a given bug's behavior evident without giving away what the particular error is, and, conversely, make it clear when bugs have been solved. The test case output can also be read by the feedback generation script, so the robot can cue into test failure (or success) and react appropriately.  While participants aren't allowed to edit the test itself, they can add their own tests if they think it will help localize an error (I'm intrigued as to whether any participants will do this).

Now that the task is (essentially) finalized, all that's left is to tune the robot's feedback accordingly and start subjecting my colleagues to my little gauntlet. Science is fun!