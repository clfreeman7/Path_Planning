function [from,to] = forwardmap(edgeNum,nStates)
    % n_s = number of states (in our case, 16)
    edgeTable = zeros(nStates);
    transitionNum = 0;
    for iStates = 1:nStates
        for jStates = 1:nStates
            if jStates ~= iStates
                transitionNum = transitionNum+1;
                edgeTable(iStates,jStates) = transitionNum;
            end
        end
    end
    [from,to]=find(edgeTable ==edgeNum);
end