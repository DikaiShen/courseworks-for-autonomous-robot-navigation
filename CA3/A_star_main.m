
% Dikai Shen
% A0285139W

clc
clear
close all

load Map.mat

%% initalization
% Define the grid and obstacles
grid = Map;
start = [1,1];
goal = [10,10];
[row, col]= find(grid == 1);
obs = [row,col];

dx = [1, -1, 0, 0,1,1,-1,-1];
dy = [0, 0, 1, -1,1,-1,1,-1];
%define explore direction

% Define costs for movement
cost = ones(size(grid));
cost(grid == 1) = Inf; % Mark obstacles as impassable

%define the heuristic function
heuristic = @(node) abs(node(1) - goal(1)) + abs(node(2) - goal(2));

%define openlist to store the nodes being explored
openList = [];
%define the closelist to store explored nodes
closeList = start;
%define cell array to store the parents of each nodes
parents = {start, start};
currentNode = start;

for i = 1:8
    %start exploring for the first iteration
    child_node = [currentNode(1)+dx(i), currentNode(2)+dy(i)];
    if child_node(1) >= 1 && child_node(1) <= size(grid,1) && child_node(2) >= 1 && child_node(2) <= size(grid,2)
        %check if the node is valid
        if cost(child_node(1),child_node(2)) ~= Inf
            g = norm(start - child_node);
            h = heuristic(child_node);
            f = g+h;
            %calculate the cost for each node
            openList = [openList; child_node,f];
            parents(end+1,:) = {currentNode,child_node};
            %store the nodes to openlist and assign current node as parent
            %node for them
        end
    end
end

openList = sortrows(openList,3);
%form a priority queue for the openlist, the first element is the node with
%least cost
currentNode = openList(1,1:2);
%assign a new current node

%% A*
loop = true;
while loop
    for i = 1:8
        child_node = [currentNode(1)+dx(i), currentNode(2)+dy(i)];
        if child_node(1) >= 1 && child_node(1) <= size(grid,1) && child_node(2) >= 1 && child_node(2) <= size(grid,2)
            if cost(child_node(1),child_node(2)) ~= Inf
                if ~ismember(child_node,closeList,"rows")
                    g = norm(start - child_node);
                    h = heuristic(child_node);
                    f = g+h;
                    [inTheOpen, idx] = ismember([child_node,f],openList,'rows');
                    %check if the node is already in the openlist or not
                    if inTheOpen == true
                        if f < openList(idx,3)
                            openList(idx,3) = f;
                            %update the cost if the new cost is lower
                        end
                    else
                        openList = [openList;child_node,f];
                        parents(end+1,:) = {currentNode,child_node};
                        %add the node to openlist if it have not been added
                        %assign the current as the parent node
                    end
                end
            end
        end
    end

    closeList = [closeList; currentNode];
    openList(1,:) = [];
    %put the current in the clost list and delete it in the openlist
    openList = sortrows(openList,3);
    currentNode = openList(1,1:2);
    %assign a new current node
    if currentNode == goal
        disp('path found')
        loop = false;
        path = backtrack(parents,goal);
        %reconstruct the path by tracing the parent node
    end

    if isempty(openList) && currentNode == goal
        disp('this no path')
    end
end



%% path plot

figure;
colormap([1 1 1;0 0 0]); 
% White for open spaces, black for obstacles
imagesc(grid);
title('path illustration');


hold on;
plot(path(:, 2), path(:, 1), 'go-', 'MarkerSize', 8, 'LineWidth', 2); 
% plot the path in sets of [row, column] coordinates

for i = 1:size(path, 1)
    text(path(i, 2), path(i, 1), sprintf('(%d, %d)', path(i, 1), path(i, 2)), 'Color', 'red');
end
axis equal;


