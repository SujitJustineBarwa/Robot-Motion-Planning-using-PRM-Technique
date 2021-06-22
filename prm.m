clc; clearvars; close all;
% load and show the obstacle file
img = imread('config_space.png');
figure(1);
imshow(img);
% generate N samples/milestones
N = 100;
% k is the number of neighbors
k = 5;
nodes = [];
sz = size(img);
h = 0;
np = [];
warning('off','all');

while lt(size(nodes,1), N)
    qx = randi([1,sz(2)]);
    qy = randi([1,sz(1)]);
    if img(qy, qx) == 255 % does not lie on obstacle
        nodes = [nodes;qx, qy];
    end
    nodes = unique(nodes,'rows');
end
%% construct kdtree
ns = KDTreeSearcher(nodes);
neighbors {size(nodes,1)} = {};
for i = colon(1, size(nodes,1))
    p1 = nodes(i,:);
    idx = knnsearch(ns, p1, 'k', k);
    nodeneighbors = [];
    for j = colon(1, size(idx,2))
        if ne(idx(j), i)
            p2 = nodes(idx(j),:);
            % collision check for p1p2
            if not(collision_checker(p1, p2, img))
                nodeneighbors = [nodeneighbors; idx(j)];
            end
        end
    end
    neighbors{i} = nodeneighbors;
end

init_loc = [30 20];
goal_loc = [1420 700bi];

init_idx = knnsearch(ns, init_loc, 'k', 1);
goal_idx = knnsearch(ns, goal_loc, 'k', 1);

init_neigh = nodes(init_idx,:);
goal_neigh = nodes(goal_idx,:);

hold on;
plot(nodes(:,1),nodes(:,2),'rx');

for i = colon(1, size(nodes,1))
    nodeneighbors = neighbors{i};
    for j = colon(1, size(nodeneighbors))
        line([nodes(i,1) nodes(nodeneighbors(j),1)], ...
            [nodes(i,2) nodes(nodeneighbors(j),2)]);
    end
end

plot(init_loc(1), init_loc(2), 'gx');
plot(goal_loc(1), goal_loc(2), 'gx');
plot(init_neigh(1), init_neigh(2), 'gd');
plot(goal_neigh(1), goal_neigh(2), 'gd');
hold off;

%% search a path using A*
init_node_pos =  init_neigh;
goal_node_pos = goal_neigh;

init_g_cost = 0;
init_h_cost = norm(init_node_pos-goal_node_pos);
init_f_cost = init_g_cost + init_h_cost;

init_node.pos = init_node_pos;
init_node.gcost = init_g_cost;
init_node.hcost = init_h_cost;
init_node.fcost = init_f_cost;
init_node.idx = init_idx;

OPEN = PriorityQueue();

OPEN.push(init_f_cost, init_node);
CLOSED = [];
path=[];
%current_node_pos = [0 0]; % this is not required
count = 1;
tic;
while OPEN.size()>0
    current_node = OPEN.pop();
    
    current_node_pos = current_node.pos;
    if norm(current_node_pos - goal_node_pos)==0
        path = goal_node_pos;
        parent_node = current_node.parent;
        parent_node.pos;
        while norm(parent_node.pos - init_node_pos)>0
            path = [path;parent_node.pos];
            parent_node = parent_node.parent;
        end
        break;
    end
    
    CLOSED = [CLOSED;current_node_pos];
    %expand
    curr_neighbors = neighbors{current_node.idx};
    for i=1:size(curr_neighbors,1)
        neighbor_node_pos = nodes(curr_neighbors(i),:);
        
        %%check if closed already
        is_closed = false;
        for j=1:size(CLOSED,1)
            if norm(neighbor_node_pos - CLOSED(j,:))==0
                is_closed = true;
                break;
            end
        end
        if  ~is_closed
            neighbor_node_g_cost = current_node.gcost + ...
                norm(neighbor_node_pos-current_node_pos);
            neighbor_node_h_cost = norm(neighbor_node_pos - ...
                goal_node_pos);
            neighbor_node_f_cost = neighbor_node_g_cost + ...
                neighbor_node_h_cost;
            neighbor_node.pos = neighbor_node_pos;
            neighbor_node.gcost = neighbor_node_g_cost;
            neighbor_node.hcost = neighbor_node_h_cost;
            neighbor_node.fcost = neighbor_node_f_cost;
            neighbor_node.parent = current_node;
            neighbor_node.idx = curr_neighbors(i);
            %is in OPEN
            is_open = false;
            for j=1:OPEN.size()
                if norm(neighbor_node_pos - OPEN.getUserData(j).pos)==0
                    is_open = true;
                    break;
                end
            end
            if ~is_open
                count = count + 1;
                OPEN.push(neighbor_node.fcost,neighbor_node);
            end
        end
    end
end
f1 = toc;
disp(['Time for Euclidean Heuristic is ', num2str(f1), ' seconds']);
disp(['Nodes Expanded using Euclidean = ',num2str(count)]);
if isempty(path)
    disp('No path found. Failure');
else
    path = [goal_loc;path;init_node_pos;init_loc];
    figure(1)
    hold on;
    plot(path(:,1), path(:,2), '-kd', 'LineWidth',2);
    hold off;
end

