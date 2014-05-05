function X_future = propigate(X,cmdW,cmdV, dt,g)
%propigates a position based on previously issued commands

u = [cmdV;cmdW]*dt;
X_future = feval(g,X,u);
end