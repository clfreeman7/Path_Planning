function edgeNum = inversemap(from,to,nStates)
    % n_s = number of states (in our case, 16)
   for m = 2:nStates-1
       h(m-1) = H(f(from,to,nStates)-f(m,m,nStates));
   end
   edgeNum = f(from,to,nStates)-sum(h);
      function F = f(i,j,nStates)
        F=nStates*(i-1)+(j-1);
    end
    function h = H(x)     % discrete Heaviside step function
        if x <0
            h = 0;
        else
            h = 1;
        end
    end
end