clear;
close all;

numBoids = 90;      % Number of boids to be simulated.
height = 100;       % Height of the world.
width = 100;        % Width of the world.
length = 100;        % length of the world.
iteration = 1000;    % Number of simulation times.
b_max_speed = 3;      % Maximum speed of boids.

% Create an array of Boid objects.
boids(numBoids) = Boid;


% Initialize the boids with random color, coordinate, and velocity.
for i = 1 : numBoids
    boids(i).velocity = [rand * (b_max_speed * 2) - (b_max_speed / 2), rand * (b_max_speed * 2) - (b_max_speed / 2), rand * (b_max_speed * 2) - (b_max_speed / 2)];
    boids(i).coord = [(rand * length - 1) + 1, (rand * width - 1) + 1, (rand * height - 1) + 1];
    boids(i).set_display(length, width, height);
    boids(i).set_max_speed(b_max_speed);
end


h = plot3(0,0,0);
xlim([0 100]);
ylim([0 100]);
zlim([0 100]);


arrows = [];
for i = 1 : numBoids
    arrows(i) = arrow('Start',[0,0,0],'Stop',[0,0,0],'Length',0,'BaseAngle',0);
end

% Start the simulation.
for t = 1 : iteration
    waypointslastStep = [];
    waypointsPerStep = [];
    % Simulation of boids.
    for i = 1 : numBoids
        waypointslastStep(i,:) = boids(i).coord;
        boids(i).move(boids);
        waypointsPerStep(i,:) = boids(i).coord;
    end
    
    for i = 1 : numBoids
        arrow(arrows(i),'Start',waypointslastStep(i,:),'Stop',waypointsPerStep(i,:),'Length',3,'BaseAngle',20);
    end
    pause(0.04);
end



