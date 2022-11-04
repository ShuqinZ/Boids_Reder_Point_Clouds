clear;
close all;

numBoids = 90;      % Number of boids to be simulated.
height = 200;       % Height of the world.
width = 250;        % Width of the world.
length = 250;        % length of the world.
iteration = 10000;    % Number of simulation times.
b_max_speed = 5;      % Maximum speed of boids.

% Create an array of Boid objects.
boids(numBoids) = Boid;

initialPts = readmatrix("Point Cloud Squence/90DronesRise.csv");
ptCld(:,:) = readmatrix("Point Cloud Squence/butterfly.csv");

% Initialize the boids with coordinate, and velocity.
for i = 1 : numBoids
%     boids(i).coord = [(rand * length - 1) + 1, (rand * width - 1) + 1, (rand * height - 1) + 1];
    boids(i).coord = initialPts(i,:);
    boids(i).target = ptCld(i,:);
    boids(i).set_display(length, width, height);
    boids(i).set_max_speed(b_max_speed);
    boids(i).velocity = (boids(i).target - boids(i).coord) * b_max_speed/norm((boids(i).target - boids(i).coord));
end


h = plot3(0,0,0);
hold on;
xlim([0, length]);
ylim([0 width]);
zlim([0 height]);


arrows = [];
for i = 1 : numBoids
    arrows(i) = arrow('Start',[0,0,0],'Stop',[0,0,0],'Length',0,'BaseAngle',0);
end

step = 0;
% Start the simulation.
for t = 1 : iteration
    step = step + 1;
    waypointslastStep = [];
    waypointsPerStep = [];
    count_arrived = 0;
    % Simulation of boids.
    for i = 1 : numBoids
        if boids(i).arrived
            count_arrived = count_arrived + 1;
        end
        waypointslastStep(i,:) = boids(i).coord;
        boids(i).move(boids);
        waypointsPerStep(i,:) = boids(i).coord;

    end
    
    fprintf("Step %d, %d has arrived\n", step,count_arrived);
    if step > 300
        for i = 1 : numBoids
            if boids(i).arrived
                arrows(i) = arrow(arrows(i),'Start',waypointslastStep(i,:),'Stop',waypointsPerStep(i,:),'Length',3,'BaseAngle',20, 'Color', 'r');
            else
                arrows(i) = arrow(arrows(i),'Start',waypointslastStep(i,:),'Stop',waypointsPerStep(i,:),'Length',3,'BaseAngle',20);
            end
%             fprintf("Drone %d dist To Target %f", i, norm(boids(i).coord - boids(i).target));
        end

        pause(0.01);
    end
end



