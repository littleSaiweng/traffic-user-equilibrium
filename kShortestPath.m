function [shortestPaths, totalCosts] = kShortestPath(netCostMatrix, source, destination, k_paths)
%netCostMatrix, OD间出行路径
% source, OD起点
% destination,OD终点
% k_paths 要找k_path条短路
% shortestPaths,共找出k_path条短路
% totalCosts，各条路径的出行距离
%%


if source > size(netCostMatrix,1) || destination > size(netCostMatrix,1)
    warning('The source or destination node are not part of netCostMatrix');
    shortestPaths=[];
    totalCosts=[];
else
    %---------------------INITIALIZATION---------------------
    k=1;
    [path,cost] = dijkstra(netCostMatrix, source, destination);
    %P is a cell array that holds all the paths found so far:
    if isempty(path)
        shortestPaths=[];
        totalCosts=[];
    else
        path_number = 1; 
        P{path_number,1} = path; P{path_number,2} = cost; 
        current_P = path_number;
        %X is a cell array of a subset of P (used by Yen's algorithm below):
        size_X=1;  
        X{size_X} = {path_number; path; cost};

        %S path_number x 1
        S(path_number) = path(1); %deviation vertex is the first node initially

        % K = 1 is the shortest path returned by dijkstra():
        shortestPaths{k} = path ;
        totalCosts(k) = cost;

        %--------------------------------------------------------
        while (k < k_paths   &&   size_X ~= 0 )
            %remove P from X
            for i=1:length(X)
                if  X{i}{1} == current_P
                    size_X = size_X - 1;
                    X(i) = [];%delete cell
                    break;
                end
            end

            %---------------------------------------
            P_ = P{current_P,1}; %P_ is current P, just to make is easier for the notations

            %Find w in (P_,w) in set S, w was the dev vertex used to found P_
            w = S(current_P);
            for i = 1: length(P_)
                if w == P_(i)
                    w_index_in_path = i;
                end
            end


            for index_dev_vertex= w_index_in_path: length(P_) - 1   %index_dev_vertex is index in P_ of deviation vertex
                temp_netCostMatrix = netCostMatrix;
                %------
                %Remove vertices in P before index_dev_vertex and there incident edges
                for i = 1: index_dev_vertex-1
                    v = P_(i);
                    temp_netCostMatrix(v,:)=inf;
                    temp_netCostMatrix(:,v)=inf;
                end
                %------
                %remove incident edge of v if v is in shortestPaths (K) U P_  with similar sub_path to P_....
                SP_sameSubPath=[];
                index =1;
                SP_sameSubPath{index}=P_;
                for i = 1: length(shortestPaths)
                    if length(shortestPaths{i}) >= index_dev_vertex
                        if P_(1:index_dev_vertex) == shortestPaths{i}(1:index_dev_vertex)
                            index = index+1;
                            SP_sameSubPath{index}=shortestPaths{i};
                        end
                    end            
                end       
                v_ = P_(index_dev_vertex);
                for j = 1: length(SP_sameSubPath)
                    next = SP_sameSubPath{j}(index_dev_vertex+1);
                    temp_netCostMatrix(v_,next)=inf;   
                end
                %------

                %get the cost of the sub path before deviation vertex v
                sub_P = P_(1:index_dev_vertex);
                cost_sub_P=0;
                for i = 1: length(sub_P)-1
                    cost_sub_P = cost_sub_P + netCostMatrix(sub_P(i),sub_P(i+1));
                end

                %call dijkstra between deviation vertex to destination node    
                [dev_p c] = dijkstra(temp_netCostMatrix, P_(index_dev_vertex), destination);
                if ~isempty(dev_p)
                    path_number = path_number + 1;
                    P{path_number,1} = [sub_P(1:end-1) dev_p] ;  %concatenate sub path- to -vertex -to- destination
                    P{path_number,2} =  cost_sub_P + c ;

                    S(path_number) = P_(index_dev_vertex);

                    size_X = size_X + 1; 
                    X{size_X} = {path_number;  P{path_number,1} ;P{path_number,2} };
                else
                    %warning('k=%d, isempty(p)==true!\n',k);
                end      
            end
            %---------------------------------------
            %Step necessary otherwise if k is bigger than number of possible paths
            %the last results will get repeated !
            if size_X > 0
                shortestXCost= X{1}{3};  %cost of path
                shortestX= X{1}{1};        %ref number of path
                for i = 2 : size_X
                    if  X{i}{3} < shortestXCost
                        shortestX= X{i}{1};
                        shortestXCost= X{i}{3};
                    end
                end
                current_P = shortestX;
                %******
                k = k+1;
                shortestPaths{k} = P{current_P,1};
                totalCosts(k) = P{current_P,2};
                %******
            else
                %k = k+1;
            end
        end
    end
     totalCosts=totalCosts';
end

