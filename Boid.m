% Boids class

classdef Boid < handle
    properties
        ID = 0;
        coord = [0, 0, 0];                  % boid's position in world
        velocity = [0, 0, 0];               % boid's velocity
        neighbors = [];                     % other boids that are close to the boid
        close_neighbors = [];               % other boids that are too close.
        d_height = 0;                       % Height of the display.
        d_width = 0;                        % Width of the display.
        d_length = 0;                       % Length of the display.
        max_speed = 0;                      % Maximum speed of the boid.

        check_range = 3;                   % Check range of boid
        rep_range = 1;                      % Repel range of boid
        target = [0, 0, 0];                 % Target assigned to the boid
        arrived = false;
        stepPerSec = 25;                    % 25 stpes per second defines the length of timestep
        threshold = 1;
        distTraveled = 0;
        centerPoint = [0, 0, 0];
        collided = false;
        removed = false;
    end
    methods
        
        % Method that updates the coordinate of the boid according to its
        % new velocity. The new velocity is adjusted by its previous
        % velocity, and the 5 rules, which are cohesion, separation,
        % alignment, edge avoidance, and predator avoidance.
        % Input : Array of boids, Array of predators
        function [isColliding, avoidspeed] = move(obj, boids, goto_center)
            isColliding = false;
            avoidspeed = 0;

            obj.findNeighbors(boids);
                    
            if obj.arrived
                return
            end

            if obj.collided
                isColliding = obj.collided;
                obj.collided = false;
                obj.removed = true;
            end        

            if obj.removed
                obj.arrived = true;
                obj.coord = [-100, -100, -100];
                return
            end
            
            newV = [0,0,0];
            priority = 0;
            % New velocity is previous velocity plus change of velocity due
            % to the rules.
            if ~goto_center

                while ~all(newV)
                    priority = priority + 1;
                    switch priority
                        case 1
                            [v1x, v1y, v1z] = obj.separation();
                            newV = [v1x, v1y, v1z];
   
                        case 2
                            [v4x, v4y, v4z] = obj.avoid_edge();
                            newV = [v4x, v4y, v4z];
                        otherwise
                            [v5x, v5y, v5z] = obj.goTo_target();
                            newV = [v5x, v5y, v5z];
                    end

                end
                
                obj.velocity = newV;
            else
                 while ~all(newV)
                    priority = priority + 1;
                    switch priority
                        case 1
                            [v1x, v1y, v1z] = obj.separation();
                            newV = [v1x, v1y, v1z];
                        case 2
                            [v4x, v4y, v4z] = obj.avoid_edge();
                            newV = [v4x, v4y, v4z];
                        case 3
                            [v5x, v5y, v5z] = obj.goTo_center();
                            newV = [v5x, v5y, v5z];
                        case 4
                            [v2x, v2y, v2z] = obj.alignment();
                            newV =  [v2x, v2y, v2z];
                        case 5
                            [v3x, v3y, v3z] = obj.cohesion();
                            newV = [v3x, v3y, v3z];
                        otherwise
                            newV = obj.go_around_center();

                    end

                end
                
                obj.velocity = newV;
            end

%             fprintf("Point [%.2f, %.2f, %.2f], Targetting: [%.2f, %.2f, %.2f], Distance: %.2f, V1: [%.2f, %.2f, %.2f], " + ...
%                 "V2: [%.2f, %.2f, %.2f], V3:  [%.2f, %.2f, %.2f], V4: [%.2f, %.2f, %.2f], " + ...
%                 "V5:  [%.2f, %.2f, %.2f]\n", ...
%                 obj.coord, obj.target, norm(obj.coord - obj.target), v1x, v1y, v1z, v2x, v2y, v2z, v3x, v3y, v3z, v4x, v4y, v4z, v5x, v5y, v5z);
            obj.limit_speed(true);
            obj.coord = obj.coord + obj.velocity/obj.stepPerSec;
            obj.distTraveled = obj.distTraveled + norm(obj.velocity)/obj.stepPerSec;

            if abs(norm(obj.coord - obj.target)) <= obj.threshold
                obj.arrived = true;
                obj.velocity = [0, 0, 0];
            end

            % Reset the neighbors arrays for the next iteration.
            obj.neighbors = [];
            obj.close_neighbors = [];

            if v1x ~= 0 || v1y~=0 || v1z ~= 0
                avoidspeed = norm([v1x, v1y, v1z]);
            end
        end


        % method that finds other nearby Boids and remember the
        % information. The method calculates the distance between
        % itobj and all the other Boids, then store all Boids whose
        % distance is smaller than 10 in neighbors array.
        function [obj, isColliding] = findNeighbors(obj, boids)
            for i = 1 : numel(boids)
                if boids(i) ~= obj && ~boids(i).removed
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
                    if distance <= 1
                        fprintf("Drone %d Collided with drone %d and removed\n", obj.ID, i);
                        obj.collided = true;
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
            sum_velocity = [0, 0, 0]; 

            if numel(obj.close_neighbors) == 0
                x = 0;
                y = 0;
                z = 0;
            else
                for i = 1 : numel(obj.close_neighbors)
                    neighbor = obj.close_neighbors(i);
                    if neighbor.arrived
                        continue;
                    end
                    goal_pos(1) = goal_pos(1) -...
                        (neighbor.coord(1) - obj.coord(1));
                    goal_pos(2) = goal_pos(2) -...
                        (neighbor.coord(2) - obj.coord(2));
                    goal_pos(3) = goal_pos(3) -...
                        (neighbor.coord(3) - obj.coord(3));
                    sum_velocity = neighbor.velocity + obj.velocity;
                end
                
                avg_velocity = sum_velocity /  numel(obj.close_neighbors);
                x_parl = avg_velocity(1);
                y_parl = avg_velocity(2);
                z_parl = avg_velocity(3);
                
                x_op = goal_pos(1) / numel(obj.close_neighbors) / 1;
                y_op = goal_pos(2) / numel(obj.close_neighbors) / 1;
                z_op = goal_pos(3) / numel(obj.close_neighbors) / 1;

                x = (x_parl + x_op) /2;
                y = (y_parl + y_op) /2;
                z = (z_parl + z_op) /2;
                obj.velocity = [x,y,z];
            end
        end


        % Rule 2. Method that calculates the velocity change due to alignment
        % factor. The method calculates the average velocity, divides it by
        % a coefficient, and returns the result.
        function [x, y, z] = alignment(obj)
            x = 0;
            y = 0;
            z = 0;
            movingNeighbors = 0;
            if numel(obj.neighbors) ~= 0
                sum_vector = [0, 0, 0];
                fastest_Speed = 0;
                for i = 1 : numel(obj.neighbors)
                    neighbor = obj.neighbors(i);
                    if neighbor.arrived
                        continue;
                    end
                    movingNeighbors = movingNeighbors + 1;
                    sum_vector(1) = sum_vector(1) + neighbor.velocity(1);
                    sum_vector(2) = sum_vector(2) + neighbor.velocity(2);
                    sum_vector(3) = sum_vector(3) + neighbor.velocity(3);
                    fastest_Speed = max([fastest_Speed, neighbor.velocity]);
                end
                if movingNeighbors == 0
                    return
                end
                avg_vector = sum_vector / movingNeighbors;
%                 avg_vector = fastest_Speed * avg_vector / norm(avg_vector);
                x = avg_vector(1) / 4;
                y = avg_vector(2) / 4;
                z = avg_vector(3) / 4;
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
            if obj.coord(1) < obj.d_length / 10
                x = obj.max_speed * 3/4;
                % If the boid is reaching the southern border, then the vector
                % returned is [-1.5, 0, 0].
            elseif obj.coord(1) > obj.d_length - (obj.d_length / 10);
                x = -obj.max_speed * 3/4;
            end
            % If the boid is reaching the western border, then the vector
            % returned is [0, 1.5, 0].
            if obj.coord(2) < obj.d_width / 10
                y = obj.max_speed * 3/4;
                % If the boid is reaching the eastern border, then the vector
                % returned is [0, -1.5, 0].
            elseif obj.coord(2) > obj.d_width - (obj.d_width / 10);
                y = -obj.max_speed * 3/4;
            end

            % If the boid is reaching the western border, then the vector
            % returned is [0, 0, 1.5].
            if obj.coord(3) < obj.d_height / 10
                z = obj.max_speed * 3/4;
                % If the boid is reaching the eastern border, then the vector
                % returned is [0, 0, -1.5].
            elseif obj.coord(3) > obj.d_height - (obj.d_height / 10);
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
            findTargetRange = 25;
            % Check if the target is in checking range
            if distance > findTargetRange
                v = (obj.target - obj.coord) / 20;
                v = v * (findTargetRange/20) / norm(v);
                x = v(1);
                y = v(2);
                z = v(3);
            elseif distance <= findTargetRange
                v = (obj.target - obj.coord) / 10;
                v = v * findTargetRange / norm(v);
                x = v(1);
                y = v(2);
                z = v(3);
                obj.velocity = [x, y, z];
                obj.velocity = obj.velocity * obj.max_speed/norm(obj.velocity);
            end
            if distance <= obj.rep_range
                v = (obj.target - obj.coord) / 10;
                v = v * findTargetRange / norm(v);
                x = v(1);
                y = v(2);
                z = v(3);
                obj.velocity = [x, y, z];
                obj.velocity = obj.velocity * obj.max_speed/norm(obj.velocity);
            end
        end

        % Rule 6. Move toward Center of the Display
        % Method that makes the boid head to the center of the display
        % but when it gets too close
        function [x, y, z] = goTo_center(obj)
            x = 0;
            y = 0;
            z = 0;
            findTargetRange = 25;
            distance = abs(norm(obj.coord - obj.centerPoint));
            if distance > findTargetRange
                % Check if the target is in checking range
                v = (obj.centerPoint - obj.coord) / 20;
                v = v * (findTargetRange/20) / norm(v);
                x = v(1);
                y = v(2);
                z = v(3);

            end

        end

        function vChange = go_around_center(obj)
            vChange = [0,0,0];
            % if a boid get close to the center of display, change its
            % velocity angle to simulate moving around
            speedValuve = norm(obj.velocity);
            newDirection = [1, 1, 1];

            if obj.velocity(1) ~= 0
                newDirection(1) = (0 - obj.velocity(2) * newDirection(2) - obj.velocity(3) * newDirection(3))/obj.velocity(1);

            elseif obj.velocity(2) ~= 0
                newDirection(2) = (0 - obj.velocity(1) * newDirection(1) - obj.velocity(3) * newDirection(3))/obj.velocity(2);

            elseif obj.velocity(3) ~= 0
                newDirection(3) = (0 - obj.velocity(2) * newDirection(2) - obj.velocity(1) * newDirection(1))/obj.velocity(3);
            end
            vChange = newDirection * speedValuve/norm(newDirection)/10;
           
        end

                
        
        % Method that checks the speed of the boid, and reduces the speed
        % if it exceeds the limit speed.
        function obj = limit_speed(obj, go_max)
            curr_speed = sqrt(obj.velocity(1)^2 + obj.velocity(2)^2 + obj.velocity(3)^2);
            if curr_speed > obj.max_speed || go_max
                obj.velocity = obj.velocity * (obj.max_speed / curr_speed);
            end
        end
        
        % Method that sets sets the height and width of the world for boid.
        % Input: height of world, width of world
        function obj = set_display(obj, d_length, d_width, d_height)
            obj.d_height = d_height;
            obj.d_width = d_width;
            obj.d_length = d_length;
            obj.centerPoint = [d_length/2, d_width/2, d_height/2];
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

