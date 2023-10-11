# Instructions
Using the "Backtracking" and "Branch and Bound" programming techniques write a program that solves a maze in C++. You can work in couples.

The program receives from the standard input two integers M and N, followed by M lines of N boolean values (0|1) separated by a space, representing the maze.  A 1 represents a square in which it is possible to move, a 0 is a square through which it is NOT possible to pass.

The origin or start square is always the square (0,0) and the exit or goal is always the square (M-1, N-1).

The output of the program is a matrix of Boolean values (0|1) representing the path out of the maze. You must first display the solution using the backtracking technique, and then using the branch and bound technique.

Your program should be called `main.cpp` and should compile using the `g++` command in a Linux environment.

## Example

### Input:
```
4
4
1 0 0 0
1 1 0 1
0 1 0 0
1 1 1 1
```

### Output:
```
1 0 0 0
1 1 0 0
0 1 0 0
0 1 1 1

1 0 0 0
1 1 0 0
0 1 0 0
0 1 1 1
```