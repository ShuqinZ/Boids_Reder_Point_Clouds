% Boids class

classdef Boid < handle
    properties
        coord = [0, 0, 0];                  % boid's position in world
        velocity = [0, 0, 0];               % boid's velocity
        neighbors = [];                     % other boids that are close to the boid
        close_neighbors = [];               % other boids that are too close.
        height = 0;                         % Height of the world.
        width = 0;                          % Width of the world.
        length = 0;                         % Length of the world.
        max_speed = 0;                      % Maximum speed of the boid.

        check_range = 25;                   % Check range of boid
        rep_range = 5;                      % Repel range of boid
        target = [0, 0, 0];                 % Target assigned to the boid
        priority = [1, 1, 1, 1, 0.5];         % Priority of rules
        arrived = false;
        stepPerSec = 25;                    % 25 stpes per second defines the length of timestep
        threshold = 1;
    end
    methods
        
        % Method that updates the coordinate of the boid according to its
        % new velocity. The new velocity is adjusted by its previous
        % velocity, and the 5 rules, which are cohesion, separation,
        % alignment, edge avoidance, and predator avoidance.
        % Input : Array of boids, Array of predators
        function obj = move(obj, boids)
            if obj.arrived
                return
            end

            obj.findNeighbors(boids);

            [v1x, v1y, v1z] = obj.separation();
            [v2x, v2y, v2z] = obj.alignment();
            [v3x, v3y, v3z] = obj.cohesion();
            [v4x, v4y, v4z] = obj.avoid_edge();
            [v5x, v5y, v5z] = obj.goTo_target();
            % New velocity is previous velocity plus change of velocity due
            % to the rules.
                obj.velocity(1) = obj.velocity(1) + (v1x * obj.priority(1) + v2x * obj.priority(2) + v3x * obj.priority(3) + v4x * obj.priority(4) + v5x * obj.priority(5)) / 2;
            obj.velocity(2) = obj.velocity(2) + (v1y * obj.priority(1) + v2y * obj.priority(2) + v3y * obj.priority(3) + v4y * obj.priority(4) + v5y * obj.priority(5)) / 2;
            obj.velocity(3) = obj.velocity(3) + (v1z * obj.priority(1) + v2z * obj.priority(2) + v3z * obj.priority(3) + v4z * obj.priority(4) + v5z * obj.priority(5)) / 2;


            obj.limit_speed();
            obj.coord = obj.coord + obj.velocity/obj.stepPerSec;

            if abs(norm(obj.coord - obj.target)) <= obj.threshold
                obj.arrived = true;
            end

            % Reset the neighbors arrays for the next iteration.
            obj.neighbors = [];
            obj.close_neighbors = [];
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
                    if distance <= 0.25
                        disp("Collided");
                    end
                end
            end
        end
        

        % Rule 1. Collision avoidance. Method that calculates the velcotiy change due to
        % separation factor. First, calculate the coordinate difference
        % between the boid and its close neighbors, add them all, and
        % average them. Them. change the sign of the vector so that it
        % faces away from the other boids, and divide it by a coefficient.
        function [x, y, z] = separation(obj)
            goal_pos = [0, 0, 0];
            if numel(obj.close_neighbors) == 0
                x = 0;
                y = 0;
                z = 0;
            else
                for i = 1 : numel(obj.close_neighbors)
                    neighbor = obj.close_neighbors(i);
                    goal_pos(1) = goal_pos(1) -...
                        (neighbor.coord(1) - obj.coord(1));
                    goal_pos(2) = goal_pos(2) -...
                        (neighbor.coord(2) - obj.coord(2));
                    goal_pos(3) = goal_pos(3) -...
                        (neighbor.coord(3) - obj.coord(3));
                end
                x = goal_pos(1) / numel(obj.close_neighbors) / 1;
                y = goal_pos(2) / numel(obj.close_neighbors) / 1;
                z = goal_pos(3) / numel(obj.close_neighbors) / 1;
            end
        end


        % Rule 2. Method that calculates the velocity change due to alignment
        % factor. The method calculates the average velocity, divides it by
        % a coefficient, and returns the result.
        function [x, y, z] = alignment(obj)
            if numel(obj.neighbors) == 0
                x = 0;
                y = 0;
                z = 0;
            else
                avg_vector = [0, 0, 0];
                for i = 1 : numel(obj.neighbors)
                    neighbor = obj.neighbors(i);
                    if neighbor.arrived
                        continue;
                    end
                    avg_vector(1) = avg_vector(1) + neighbor.velocity(1);
                    avg_vector(2) = avg_vector(2) + neighbor.velocity(2);
                    avg_vector(3) = avg_vector(3) + neighbor.velocity(3);
                end
                x = avg_vector(1) / numel(obj.neighbors) / 4;
                y = avg_vector(2) / numel(obj.neighbors) / 4;
                z = avg_vector(3) / numel(obj.neighbors) / 4;
            end
        end


        % Rule 3. Method that calculates the velocity change due to cohesion
        % factor. The method calculates the average position of its
        % neighbors, and returns a vector that is from the boid's current
        % position to the average position divided by a coefficient.
        function [x, y, z] = cohesion(obj)
            avg_position = [0, 0, 0];
            % If there is no neighbor, then return [0, 0, 0].
            if numel(obj.neighbors) == 0
                x = 0;
                y = 0;
                z = 0;
            else
                for i = 1 : numel(obj.neighbors)
                    if obj.neighbors(i).arrived
                        continue;
                    end
                    avg_position = avg_position + obj.neighbors(i).coord;
                end
                avg_position = avg_position / numel(obj.neighbors);
                x = (avg_position(1) - obj.coord(1)) / 20;
                y = (avg_position(2) - obj.coord(2)) / 20;
                z = (avg_position(3) - obj.coord(3)) / 20;
            end
        end
        
                
        % Rule 4. Method that makes the boid turn if it is close to the
        % border. The method checks if the boid is approaching the edge,
        % and if it is, then the method returns a vector which faces the
        % opposite direction of the boid's direction.
        function [x, y, z] = avoid_edge(obj)
            x = 0;
            y = 0;
            z = 0;
            % If the boid is reaching the northern border, then the vector
            % returned is [1.5, 0, 0].
            if obj.coord(1) < obj.length / 10
                x = obj.max_speed * 3/4;
                % If the boid is reaching the southern border, then the vector
                % returned is [-1.5, 0, 0].
            elseif obj.coord(1) > obj.length - (obj.length / 10);
                x = -obj.max_speed * 3/4;
            end
            % If the boid is reaching the western border, then the vector
            % returned is [0, 1.5, 0].
            if obj.coord(2) < obj.width / 10
                y = obj.max_speed * 3/4;
                % If the boid is reaching the eastern border, then the vector
                % returned is [0, -1.5, 0].
            elseif obj.coord(2) > obj.width - (obj.width / 10);
                y = -obj.max_speed * 3/4;
            end

            % If the boid is reaching the western border, then the vector
            % returned is [0, 0, 1.5].
            if obj.coord(3) < obj.height / 10
                z = obj.max_speed * 3/4;
                % If the boid is reaching the eastern border, then the vector
                % returned is [0, 0, -1.5].
            elseif obj.coord(3) > obj.height - (obj.height / 10);
                z = -obj.max_speed * 3/4;
            end

        end

        % Rule 5. Try to move to target
        % Method that makes the boid head to its target while having rule
        % 1,2 as higher priority
        function [x, y, z] = goTo_target(obj)
            x = 0;
            y = 0;
            z = 0;
            distance = abs(norm(obj.coord - obj.target));
            
            % Check if the target is in checking range
            if distance > obj.check_range
                v = (obj.target - obj.coord) / 20;
                v = v * (obj.check_range/20) / norm(v);
                x = v(1);
                y = v(2);
                z = v(3);
            elseif distance <= obj.check_range
                obj.priority = [1, 0.5, 0, 1, 1];
                x = (obj.target(1) - obj.coord(1)) / 10;
                y = (obj.target(2) - obj.coord(2)) / 10;
                z = (obj.target(3) - obj.coord(3)) / 10;
            end
            if distance <= obj.rep_range
%                 obj.priority = [1, 0, 0, 1, 1];
                x = (obj.target(1) - obj.coord(1)) / 5;
                y = (obj.target(2) - obj.coord(2)) / 5;
                z = (obj.target(3) - obj.coord(3)) / 5;
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

