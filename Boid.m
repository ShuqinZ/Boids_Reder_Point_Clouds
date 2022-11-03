% Boids class

classdef Boid < handle
    properties
        coord = [0, 0, 0];                     % boid's position in world
        velocity = [0, 0, 0];                  % boid's velocity
        neighbors = [];                     % other boids that are close to the boid
        close_neighbors = [];               % other boids that are too close.
        height = 0;                         % Height of the world.
        width = 0;                          % Width of the world.
        length = 0;                         % Length of the world.
        max_speed = 0;                      % Maximum speed of the boid.
        check_range = 25;                   % the distance that boid will check
        rep_range = 5;                      % the distance that boid will repel one another
        target = [0, 0, 0]                  % boid's target
        priority = [1, 1, 1, 1, 1]          % Priority of rules 1 to 5
        stepsPerSec = 25;                   % Define step length, number of steps per sec
    end
    methods
        
        % Method that updates the coordinate of the boid according to its
        % new velocity. The new velocity is adjusted by its previous
        % velocity, and the 5 rules, which are cohesion, separation,
        % alignment, edge avoidance, and predator avoidance.
        % Input : Array of boids, Array of predators
        function obj = move(obj, boids)
            obj.findNeighbors(boids);
            v1 = obj.separation();
            v2 = obj.alignment();
            v3 = obj.cohesion();
            v4 = obj.avoid_edge();
%             v5 = obj.goTo_target();
            v5 = [0, 0, 0];

            % New velocity is previous velocity plus change of velocity due
            % to the rules.
            obj.velocity = obj.velocity + (obj.priority(1) * v1 + obj.priority(2) * v2 ...
                + obj.priority(3) * v3 + obj.priority(4) * v4 + obj.priority(5) * v5) / obj.stepsPerSec;

            obj.limit_speed();
            obj.coord = obj.coord + obj.velocity;

            % Reset the neighbors arrays for the next iteration.
            obj.neighbors = [];
            obj.close_neighbors = [];
        end

        % Rule 1. Avoid collisions
        % Method that calculates the velcotiy change due to
        % separation factor. First, calculate the coordinate difference
        % between the boid and its close neighbors, add them all, and
        % average them. Them. change the sign of the vector so that it
        % faces away from the other boids, and divide it by a coefficient.
        function velocity = separation(obj)
            goal_pos = [0, 0, 0];
            if numel(obj.close_neighbors) == 0
               velocity = [0, 0, 0];
            else
                for i = 1 : numel(obj.close_neighbors)
                    neighbor = obj.close_neighbors(i);
                    goal_pos = goal_pos - (neighbor.coord - obj.coord);
                end
                velocity = goal_pos / numel(obj.close_neighbors) / 2;
            end
        end
         
        % Rule 2. match speeds with neighbors and match heading
        % Method that calculates the velocity change due to alignment
        % factor. The method calculates the average velocity, divides it by
        % a coefficient, and returns the result.
        function velocity = alignment(obj)
            if numel(obj.neighbors) == 0
                velocity = [0, 0, 0];
            else
                avg_vector = [0, 0, 0];
                for i = 1 : numel(obj.neighbors)
                    neighbor = obj.neighbors(i);
                    avg_vector = avg_vector + neighbor.velocity;
                end
                velocity = avg_vector / numel(obj.neighbors) / 4;
            end
        end
        
        
        % Rule 3. Move to the center of neighbors
        % Method that calculates the velocity change due to cohesion
        % factor. The method calculates the average position of its
        % neighbors, and returns a vector that is from the boid's current
        % position to the average position divided by a coefficient.
        function velocity = cohesion(obj)
            avg_position = [0, 0, 0];
            % If there is no neighbor, then return [0, 0, 0].
            if numel(obj.neighbors) == 0
                velocity = [0, 0, 0];
            else
                for i = 1 : numel(obj.neighbors)
                    avg_position = avg_position + obj.neighbors(i).coord;
                end
                avg_position = avg_position / numel(obj.neighbors);
                velocity = (avg_position - obj.coord) / 20;
            end
        end
       
        % Rule 4. Method that makes the boid turn if it is close to the
        % border. The method checks if the boid is approaching the edge,
        % and if it is, then the method returns a vector which faces the
        % opposite direction of the boid's direction.
        function velocity = avoid_edge(obj)
            velocity = [0, 0, 0];
            border = [obj.length, obj.width, obj.height];
            
            for i = 1 : length(obj.coord)
                % If the boid is reaching the upper border
                if obj.coord(i) < border(i) / 10
                    velocity(i) = 3;
                    % If the boid is reaching the lower border
                    % returned is [-1.5, 0, 0].
                elseif obj.coord(i) > border(i) - (border(i) / 10)
                    velocity(i) = -3;
                end
            end

        end
        
        % Rule 5. Try to move to target
        % Method that makes the boid head to its target while having rule
        % 1,2 as higher priority
        function velocity = goTo_target(obj)
            velocity = [0, 0, 0];
            distance = abs(norm(obj.coord - obj.target));
            % Check if the target is in checking range
            if distance <= obj.check_range
                velocity = (obj.target - obj.coord) / 20;
            end
        end

        % method that finds other nearby Boids and remember the
        % information. The method calculates the distance between
        % itobj and all the other Boids, then store all Boids whose
        % distance is smaller than 10 in neighbors array.
        function obj = findNeighbors(obj, boids)
            for i = 1 : numel(boids)
                if boids(i) ~= obj
                    % Distance formula.4
                    distance = sqrt((obj.coord(1) - boids(i).coord(1))^2 ...
                        + (obj.coord(2) - boids(i).coord(2))^2 ...
                        + (obj.coord(3) - boids(i).coord(3))^2);
                    % All boids whose distance is less than 30 is now the
                    % boid's neighbor. Cohesion rule applies for them.
                    if distance <= obj.check_range
                        obj.neighbors = [obj.neighbors, boids(i)];
                    end
                    % All boids whose distance is less than 5 is now the
                    % boid's close neighbor. Separation rule applies for
                    % them.
                    if distance <= obj.rep_range
                        obj.close_neighbors = [obj.close_neighbors, boids(i)];
                    end
                end
            end
        end
        
        % Method that checks the speed of the boid, and reduces the speed
        % if it exceeds the limit speed.
        function obj = limit_speed(obj)
            curr_speed = sqrt(obj.velocity(1)^2 + obj.velocity(2)^2 + obj.velocity(3)^2);
            if curr_speed > obj.max_speed
                obj.velocity = obj.velocity * (obj.max_speed / curr_speed);
            end
        end
        
        % Method that sets sets the height and width of the world for boid.
        % Input: height of world, width of world
        function obj = set_display(obj, length, width, height);
            obj.height = height;
            obj.width = width;
            obj.length = length;
        end
        
        % Method that sets the maximum speed of the boid.
        % Input: maximum speed of boid
        function obj = set_max_speed(obj, max_speed)
            obj.max_speed = max_speed;
        end

        % Method that sets the target to the boid.
        % Input: target coordinate
        function obj = set_target(obj, target)
            obj.target = target;
        end
        
    end
end

