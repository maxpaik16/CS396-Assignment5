# Assignment 5
Max Paik's  Repo for Assignment 5 for CS 396 Artificial Life at Northwestern University Winter 2023

Goal: teach a snake how to slither along a runway

Usage: to evolve a new snake, run the following command: "python search.py"

Description of what I did: 

I placed a long block in the center of the world that serves as a kind of platform. I modified my fitness function to give very bad scores to any robots that fall off the block by checking the z coordinates of the first and last link. If the z coordinates remain elevated (the robot didn't fall off), then the fitness function gives preference to robots that travel furthest into the screen. In this way, my code trains robots to walk along the elevated platform in their world. 
