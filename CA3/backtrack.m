function path = backtrack(parents,goal)

    path = [];
    parents = flip(parents);
    searchChild = goal;

    for i = 1:size(parents,1)

        if parents{i,2} == searchChild

            path = [path;searchChild];
            searchChild = parents{i,1};
        
        end
        
            
    end


end

